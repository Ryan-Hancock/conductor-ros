package conductor

import (
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"net/url"
	"sort"
	"sync"
	"time"
)

// The trace context already crosses the wire: a message carries the span that
// published it, so a subscription callback in another process records itself
// as that span's child. Each process therefore holds half of every
// cross-process trace, and neither half is worth much alone — the per-process
// view shows a subscription with no visible cause, and the publisher's view
// stops at the publish.
//
// Stitching them is the point of a fleet view. The collector polls each
// process for the spans it has not seen, keeps them in one bounded store, and
// joins parent to child by span id across processes. What comes out is the
// thing a ROS developer cannot otherwise get: one causal chain, across four
// units, with the process boundary marked rather than hidden.

// FleetTrace is one trace, stitched.
type FleetTrace struct {
	ID        string          `json:"id"`
	Start     time.Time       `json:"start"`
	Duration  float64         `json:"duration_ms"`
	Processes []string        `json:"processes"`
	Crosses   bool            `json:"crosses"` // touches more than one process
	Spans     []FleetSpanView `json:"spans"`
}

// FleetSpanView is a span placed in the stitched tree.
type FleetSpanView struct {
	SpanRecord
	Depth    int     `json:"depth"`
	OffsetMS float64 `json:"offset_ms"`
	// Handover marks a span whose parent ran in another process: the point
	// where the message crossed the wire.
	Handover bool `json:"handover"`
	// Orphan marks a span whose parent is not in the store — usually the
	// other end has not been polled yet, or its ring has rolled over.
	Orphan bool `json:"orphan"`
}

// TracesResponse is what the fleet's /api/traces returns.
type TracesResponse struct {
	Now       time.Time    `json:"now"`
	Traces    []FleetTrace `json:"traces"`
	Recording []string     `json:"recording"` // processes whose ring is on
	Silent    []string     `json:"silent"`    // processes with tracing off
	Findings  []Finding    `json:"findings"`
}

// traceStore accumulates spans from every process. It is bounded: a robot
// runs for weeks, and the interesting trace is a recent one.
type traceStore struct {
	mu    sync.Mutex
	spans []SpanRecord
	next  int
	cap   int

	cursor    map[string]time.Time // per process: the last span start seen
	recording map[string]bool
	skew      map[string]time.Duration // process pair -> worst observed skew
}

func newTraceStore(capacity int) *traceStore {
	return &traceStore{
		spans:     make([]SpanRecord, 0, capacity),
		cap:       capacity,
		cursor:    map[string]time.Time{},
		recording: map[string]bool{},
		skew:      map[string]time.Duration{},
	}
}

func (s *traceStore) add(records []SpanRecord) {
	s.mu.Lock()
	defer s.mu.Unlock()
	for _, r := range records {
		if len(s.spans) < s.cap {
			s.spans = append(s.spans, r)
			continue
		}
		s.spans[s.next] = r
		s.next = (s.next + 1) % s.cap
	}
}

func (s *traceStore) since(process string) time.Time {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.cursor[process]
}

func (s *traceStore) advance(process string, records []SpanRecord, tracing bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.recording[process] = tracing
	for _, r := range records {
		if r.Start.After(s.cursor[process]) {
			s.cursor[process] = r.Start
		}
	}
}

func (s *traceStore) all() ([]SpanRecord, map[string]bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	out := make([]SpanRecord, len(s.spans))
	copy(out, s.spans)
	rec := make(map[string]bool, len(s.recording))
	for k, v := range s.recording {
		rec[k] = v
	}
	return out, rec
}

// collect polls one process for the spans it has not reported yet.
func (s *traceStore) collect(ctx context.Context, client *http.Client, p Peer, limit int) error {
	label := p.label()
	endpoint, err := url.JoinPath(p.URL, "/api/spans")
	if err != nil {
		return err
	}
	q := url.Values{}
	if since := s.since(label); !since.IsZero() {
		q.Set("since", since.Format(time.RFC3339Nano))
	}
	q.Set("limit", fmt.Sprint(limit))

	req, err := http.NewRequestWithContext(ctx, http.MethodGet, endpoint+"?"+q.Encode(), nil)
	if err != nil {
		return err
	}
	res, err := client.Do(req)
	if err != nil {
		return err
	}
	defer res.Body.Close()
	if res.StatusCode != http.StatusOK {
		return fmt.Errorf("HTTP %s", res.Status)
	}
	var body SpansResponse
	if err := json.NewDecoder(res.Body).Decode(&body); err != nil {
		return err
	}
	for i := range body.Spans {
		body.Spans[i].Process = label
	}
	s.add(body.Spans)
	s.advance(label, body.Spans, body.Tracing)
	return nil
}

// label is how a peer is named in the merged graph and in stitched traces.
func (p Peer) label() string {
	v := ProcessView{Peer: p}
	return v.label()
}

// traces stitches the store into causal chains, most recent first, with the
// traces that actually cross a process boundary first among them: those are
// the ones no single dashboard could have shown.
func (s *traceStore) traces(limit int) TracesResponse {
	spans, recording := s.all()
	out := TracesResponse{Now: time.Now(), Traces: []FleetTrace{}, Recording: []string{}, Silent: []string{}, Findings: []Finding{}}
	for name, on := range recording {
		if on {
			out.Recording = append(out.Recording, name)
		} else {
			out.Silent = append(out.Silent, name)
		}
	}
	sort.Strings(out.Recording)
	sort.Strings(out.Silent)

	byTrace := map[string][]SpanRecord{}
	for _, r := range spans {
		byTrace[r.Trace] = append(byTrace[r.Trace], r)
	}
	skew := map[string]time.Duration{}
	for id, group := range byTrace {
		out.Traces = append(out.Traces, stitch(id, group, skew))
	}
	sort.Slice(out.Traces, func(i, j int) bool {
		if out.Traces[i].Crosses != out.Traces[j].Crosses {
			return out.Traces[i].Crosses
		}
		return out.Traces[i].Start.After(out.Traces[j].Start)
	})
	if limit > 0 && len(out.Traces) > limit {
		out.Traces = out.Traces[:limit]
	}

	// Clocks on separate machines do not agree, and a child that starts
	// before its parent is how that shows up in a trace. Say so, with the
	// size of it, rather than drawing a chain that runs backwards.
	for pair, d := range skew {
		if d > time.Millisecond {
			out.Findings = append(out.Findings, Finding{"warning", "FLEET07",
				fmt.Sprintf("spans from %s start up to %s before the parent that caused them: those clocks disagree",
					pair, d.Round(time.Millisecond))})
		}
	}
	sortFindings(out.Findings)
	return out
}

// stitch builds one trace's causal tree from spans recorded by any number of
// processes.
func stitch(id string, spans []SpanRecord, skew map[string]time.Duration) FleetTrace {
	sort.Slice(spans, func(i, j int) bool { return spans[i].Start.Before(spans[j].Start) })

	byID := make(map[string]SpanRecord, len(spans))
	children := map[string][]SpanRecord{}
	for _, s := range spans {
		byID[s.ID] = s
	}
	var roots []SpanRecord
	for _, s := range spans {
		if s.Parent == "" {
			roots = append(roots, s)
			continue
		}
		if _, ok := byID[s.Parent]; !ok {
			roots = append(roots, s) // the other half has not arrived
			continue
		}
		children[s.Parent] = append(children[s.Parent], s)
	}

	start := spans[0].Start
	end := start
	procs := map[string]bool{}
	for _, s := range spans {
		procs[s.Process] = true
		if fin := s.Start.Add(time.Duration(s.Duration * float64(time.Millisecond))); fin.After(end) {
			end = fin
		}
	}

	trace := FleetTrace{
		ID:       id,
		Start:    start,
		Duration: float64(end.Sub(start).Microseconds()) / 1000,
		Spans:    []FleetSpanView{},
	}
	for p := range procs {
		if p != "" {
			trace.Processes = append(trace.Processes, p)
		}
	}
	sort.Strings(trace.Processes)
	trace.Crosses = len(trace.Processes) > 1

	var walk func(s SpanRecord, depth int)
	walk = func(s SpanRecord, depth int) {
		v := FleetSpanView{
			SpanRecord: s,
			Depth:      depth,
			OffsetMS:   float64(s.Start.Sub(start).Microseconds()) / 1000,
		}
		if v.OffsetMS < 0 {
			v.OffsetMS = 0
		}
		if parent, ok := byID[s.Parent]; ok {
			v.Handover = parent.Process != s.Process
			if v.Handover && s.Start.Before(parent.Start) {
				// Only cross-process pairs can disagree about the clock; one
				// process's spans are ordered by one clock.
				pair := parent.Process + " and " + s.Process
				if d := parent.Start.Sub(s.Start); d > skew[pair] {
					skew[pair] = d
				}
			}
		} else if s.Parent != "" {
			v.Orphan = true
		}
		trace.Spans = append(trace.Spans, v)

		kids := children[s.ID]
		sort.Slice(kids, func(i, j int) bool { return kids[i].Start.Before(kids[j].Start) })
		for _, k := range kids {
			walk(k, depth+1)
		}
	}
	for _, root := range roots {
		walk(root, 0)
	}
	return trace
}

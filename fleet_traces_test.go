package conductor

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"net/url"
	"strings"
	"testing"
	"time"
)

// spanServer serves a process's spans the way a traced runtime does, and
// records the cursors it was asked for.
type spanServer struct {
	*httptest.Server
	spans   []SpanRecord
	tracing bool
	asked   []string
}

func newSpanServer(t *testing.T, tracing bool, spans ...SpanRecord) *spanServer {
	t.Helper()
	s := &spanServer{spans: spans, tracing: tracing}
	s.Server = httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/api/spans" {
			http.NotFound(w, r)
			return
		}
		since := r.URL.Query().Get("since")
		s.asked = append(s.asked, since)
		out := SpansResponse{Tracing: s.tracing, Now: time.Now(), Spans: []SpanRecord{}}
		for _, sp := range s.spans {
			if since != "" {
				if at, err := time.Parse(time.RFC3339Nano, since); err == nil && !sp.Start.After(at) {
					continue
				}
			}
			out.Spans = append(out.Spans, sp)
		}
		json.NewEncoder(w).Encode(out)
	}))
	t.Cleanup(s.Server.Close)
	return s
}

var traceStart = time.Date(2026, 7, 29, 12, 0, 0, 0, time.UTC)

func span(id, parent, node, kind, name string, offset time.Duration, dur float64) SpanRecord {
	return SpanRecord{
		Trace: "abc123", ID: id, Parent: parent,
		Node: node, Kind: kind, Name: name,
		Start: traceStart.Add(offset), Duration: dur,
	}
}

func collectFrom(t *testing.T, store *traceStore, peers ...Peer) {
	t.Helper()
	client := &http.Client{Timeout: time.Second}
	for _, p := range peers {
		if err := store.collect(context.Background(), client, p, 100); err != nil {
			t.Fatalf("collect from %s: %v", p.Name, err)
		}
	}
}

// The point of the whole thing: a timer in one process publishes, the
// subscription it causes runs in another, and the two halves join into one
// chain.
func TestTracesStitchAcrossProcesses(t *testing.T) {
	// localizer: a timer that published.
	a := newSpanServer(t, true, span("aa", "", "localizer", "timer", "Clock", 0, 0.5))
	// navigator: the subscription that timer caused, and its own publish.
	b := newSpanServer(t, true, span("bb", "aa", "navigator", "subscription", "amcl_pose", 300*time.Microsecond, 0.2))
	// safety_monitor: caused in turn by the navigator's publish.
	c := newSpanServer(t, true, span("cc", "bb", "safety_monitor", "subscription", "cmd_vel", 600*time.Microsecond, 0.1))

	store := newTraceStore(100)
	collectFrom(t, store,
		Peer{Name: "localizer", Host: "robot-1", URL: a.URL},
		Peer{Name: "navigator", Host: "robot-1", URL: b.URL},
		Peer{Name: "safety_monitor", Host: "robot-1", URL: c.URL})

	res := store.traces(10)
	if len(res.Traces) != 1 {
		t.Fatalf("%d traces, want the three halves joined into one", len(res.Traces))
	}
	tr := res.Traces[0]
	if !tr.Crosses || len(tr.Processes) != 3 {
		t.Fatalf("trace touches %v, want three processes", tr.Processes)
	}
	if len(tr.Spans) != 3 {
		t.Fatalf("%d spans in the stitched trace", len(tr.Spans))
	}
	// Depth-first from the root, so it reads as the causal chain.
	var chain []string
	for _, s := range tr.Spans {
		chain = append(chain, s.Node)
	}
	if got := strings.Join(chain, " -> "); got != "localizer -> navigator -> safety_monitor" {
		t.Fatalf("chain = %s", got)
	}
	for i, want := range []int{0, 1, 2} {
		if tr.Spans[i].Depth != want {
			t.Errorf("span %d has depth %d, want %d", i, tr.Spans[i].Depth, want)
		}
	}
	// The two handovers are the wire crossings, and the root is not one.
	if tr.Spans[0].Handover || !tr.Spans[1].Handover || !tr.Spans[2].Handover {
		t.Fatalf("handovers = %v", []bool{tr.Spans[0].Handover, tr.Spans[1].Handover, tr.Spans[2].Handover})
	}
	if tr.Spans[1].Process != "robot-1/navigator" {
		t.Fatalf("span process = %q", tr.Spans[1].Process)
	}
	if tr.Spans[2].OffsetMS < 0.5 || tr.Spans[2].OffsetMS > 0.7 {
		t.Fatalf("offset = %v ms, want ~0.6 from the trace start", tr.Spans[2].OffsetMS)
	}
}

// A span whose parent has not been collected is shown, marked, rather than
// dropped or promoted silently.
func TestTracesMarkOrphans(t *testing.T) {
	b := newSpanServer(t, true, span("bb", "aa", "navigator", "subscription", "amcl_pose", 0, 0.2))
	store := newTraceStore(100)
	collectFrom(t, store, Peer{Name: "navigator", Host: "robot-1", URL: b.URL})

	tr := store.traces(10).Traces[0]
	if len(tr.Spans) != 1 || !tr.Spans[0].Orphan {
		t.Fatalf("span = %+v, want it marked as an orphan", tr.Spans[0])
	}
	if tr.Crosses {
		t.Fatal("one process is not a crossing")
	}
}

// Clocks on separate machines disagree, and a child that starts before its
// parent is how that shows up. Say so rather than drawing time running
// backwards.
func TestTracesReportClockSkew(t *testing.T) {
	a := newSpanServer(t, true, span("aa", "", "localizer", "timer", "Clock", 0, 0.5))
	// The navigator's clock is 40ms behind: its span starts "before" its cause.
	b := newSpanServer(t, true, span("bb", "aa", "navigator", "subscription", "amcl_pose", -40*time.Millisecond, 0.2))

	store := newTraceStore(100)
	collectFrom(t, store,
		Peer{Name: "localizer", Host: "robot-1", URL: a.URL},
		Peer{Name: "navigator", Host: "robot-2", URL: b.URL})

	res := store.traces(10)
	var skew *Finding
	for i := range res.Findings {
		if res.Findings[i].Code == "FLEET07" {
			skew = &res.Findings[i]
		}
	}
	if skew == nil {
		t.Fatalf("findings = %v, want FLEET07", res.Findings)
	}
	for _, want := range []string{"robot-1/localizer", "robot-2/navigator", "40ms"} {
		if !strings.Contains(skew.Msg, want) {
			t.Errorf("finding %q does not mention %q", skew.Msg, want)
		}
	}
	// And the chain is still drawn forwards.
	for _, s := range res.Traces[0].Spans {
		if s.OffsetMS < 0 {
			t.Errorf("span %s has a negative offset %v", s.Node, s.OffsetMS)
		}
	}
}

// The cursor is what keeps polling cheap: the second poll asks only for what
// came after the first.
func TestTraceCollectionUsesACursor(t *testing.T) {
	s := newSpanServer(t, true, span("aa", "", "localizer", "timer", "Clock", 0, 0.5))
	store := newTraceStore(100)
	peer := Peer{Name: "localizer", Host: "robot-1", URL: s.URL}

	collectFrom(t, store, peer)
	if s.asked[0] != "" {
		t.Fatalf("first poll asked since=%q, want the whole ring", s.asked[0])
	}
	collectFrom(t, store, peer)
	if len(s.asked) != 2 || s.asked[1] == "" {
		t.Fatalf("second poll asked since=%q, want a cursor", s.asked[len(s.asked)-1])
	}
	if got, _ := store.all(); len(got) != 1 {
		t.Fatalf("%d spans stored, want the one span once", len(got))
	}
}

// Which processes are recording is part of the answer: a trace view that is
// empty because tracing is off should say so.
func TestTracesReportWhoIsRecording(t *testing.T) {
	on := newSpanServer(t, true, span("aa", "", "localizer", "timer", "Clock", 0, 0.5))
	off := newSpanServer(t, false)

	store := newTraceStore(100)
	collectFrom(t, store,
		Peer{Name: "localizer", Host: "robot-1", URL: on.URL},
		Peer{Name: "navigator", Host: "robot-1", URL: off.URL})

	res := store.traces(10)
	if len(res.Recording) != 1 || res.Recording[0] != "robot-1/localizer" {
		t.Errorf("recording = %v", res.Recording)
	}
	if len(res.Silent) != 1 || res.Silent[0] != "robot-1/navigator" {
		t.Errorf("silent = %v", res.Silent)
	}
}

// The store is bounded: a robot runs for weeks.
func TestTraceStoreIsBounded(t *testing.T) {
	store := newTraceStore(4)
	for i := 0; i < 20; i++ {
		store.add([]SpanRecord{span("id", "", "n", "timer", "T", time.Duration(i)*time.Millisecond, 0.1)})
	}
	got, _ := store.all()
	if len(got) != 4 {
		t.Fatalf("%d spans kept, want the cap of 4", len(got))
	}
}

// The per-process endpoint honours the cursor and the limit, and reports
// whether it is recording at all.
func TestSpansEndpoint(t *testing.T) {
	a := newTestApp(t, &dashNode{})
	srv := serveDashboard("127.0.0.1:0", a, "inproc", 32)
	if srv == nil {
		t.Fatal("dashboard did not start")
	}
	t.Cleanup(func() { srv.Close() })
	time.Sleep(60 * time.Millisecond)

	call := func(query string) SpansResponse {
		t.Helper()
		rec := httptest.NewRecorder()
		srv.Handler.ServeHTTP(rec, httptest.NewRequest(http.MethodGet, "/api/spans"+query, nil))
		var res SpansResponse
		if err := json.Unmarshal(rec.Body.Bytes(), &res); err != nil {
			t.Fatalf("decode: %v (%s)", err, rec.Body.String())
		}
		return res
	}

	all := call("")
	if !all.Tracing {
		t.Fatal("tracing should be on: the dashboard was given a ring")
	}
	if len(all.Spans) == 0 {
		t.Fatal("no spans recorded")
	}
	if got := call("?limit=1"); len(got.Spans) != 1 {
		t.Fatalf("limit=1 returned %d spans", len(got.Spans))
	}
	// The cursor excludes what has already been seen. The node is ticking at
	// 100hz, so newer spans may well have arrived — none of them older.
	last := all.Spans[len(all.Spans)-1]
	cursor := url.Values{"since": {last.Start.Format(time.RFC3339Nano)}}
	for _, sp := range call("?" + cursor.Encode()).Spans {
		if !sp.Start.After(last.Start) {
			t.Fatalf("span at %s was returned again for a cursor at %s", sp.Start, last.Start)
		}
	}
	// And the same cursor pasted by hand, where the offset's '+' decodes as
	// a space, must not silently replay the ring.
	if got := call("?since=" + last.Start.Format(time.RFC3339Nano)); len(got.Spans) > 0 {
		for _, sp := range got.Spans {
			if !sp.Start.After(last.Start) {
				t.Fatalf("an unescaped cursor replayed span %s", sp.Start)
			}
		}
	}

	// A span carries what stitching needs.
	if last.Trace == "" || last.ID == "" || last.Node == "" {
		t.Fatalf("span %+v is missing its identity", last)
	}
}

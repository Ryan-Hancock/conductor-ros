package conductor

import (
	"encoding/hex"
	"sort"
	"time"
)

// A trace already crosses processes: the context travels in the message
// attachment, so a callback in one unit is recorded as the child of the
// publish that caused it in another. What was missing is somewhere to put
// both halves. These are the records the fleet view collects to do that.

// SpanRecord is one finished span in the form the API serves: hex ids, so a
// span from one process can be matched to its parent in another.
type SpanRecord struct {
	Trace    string    `json:"trace"`
	ID       string    `json:"id"`
	Parent   string    `json:"parent,omitempty"`
	Node     string    `json:"node"`
	Kind     string    `json:"kind"`
	Name     string    `json:"name"`
	Start    time.Time `json:"start"`
	Duration float64   `json:"duration_ms"`
	Err      string    `json:"err,omitempty"`

	// Process is filled in by whoever collected the record; a process
	// reporting its own spans leaves it empty.
	Process string `json:"process,omitempty"`
}

// SpansResponse is what /api/spans returns.
type SpansResponse struct {
	Tracing bool         `json:"tracing"`
	Now     time.Time    `json:"now"`
	Spans   []SpanRecord `json:"spans"`
}

func spanRecord(s Span) SpanRecord {
	r := SpanRecord{
		Trace:    s.Context.TraceIDString(),
		ID:       s.Context.SpanIDString(),
		Node:     s.Node,
		Kind:     string(s.Kind),
		Name:     s.Name,
		Start:    s.Start,
		Duration: float64(s.Duration.Microseconds()) / 1000,
	}
	if s.ParentID != ([8]byte{}) {
		r.Parent = hex.EncodeToString(s.ParentID[:])
	}
	if s.Err != nil {
		r.Err = s.Err.Error()
	}
	return r
}

// records returns the ring's spans that started after since, oldest first and
// capped at limit. The cursor is what keeps a fleet poll cheap: a collector
// asks only for what it has not seen, so the cost of watching a robot does
// not grow with the size of its ring.
func (r *traceRing) records(since time.Time, limit int) []SpanRecord {
	all := r.all()
	out := make([]SpanRecord, 0, len(all))
	for _, s := range all {
		if !since.IsZero() && !s.Start.After(since) {
			continue
		}
		out = append(out, spanRecord(s))
	}
	sort.Slice(out, func(i, j int) bool { return out[i].Start.Before(out[j].Start) })
	if limit > 0 && len(out) > limit {
		// Keep the most recent: a collector that has fallen behind wants the
		// present, not the backlog.
		out = out[len(out)-limit:]
	}
	return out
}

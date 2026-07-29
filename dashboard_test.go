package conductor

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"
)

// dashNode exercises every endpoint kind the inventory reports.
type dashNode struct {
	Tick  Timer                  `rate:"100hz"`
	Out   Pub[ping]              `topic:"pings" qos:"reliable"`
	Limit Param[float64]         `name:"limit" default:"1.5"`
	Echo  Svc[dashReq, dashResp] `service:"echo"`
}

type dashReq struct{ N int }
type dashResp struct{ N int }

func (d *dashNode) OnTick()                            { d.Out.Publish(ping{V: 1}) }
func (d *dashNode) OnEcho(r dashReq) (dashResp, error) { return dashResp{N: r.N}, nil }

// newDashboard wires an app and its dashboard without starting a server.
func newDashboard(t *testing.T, traceDepth int, nodes ...any) *dashboard {
	t.Helper()
	a := newTestApp(t, nodes...)
	d := &dashboard{app: a, transport: "inproc", started: time.Now()}
	if traceDepth > 0 {
		d.ring = newTraceRing(traceDepth)
		AddExporter(d.ring)
	}
	return d
}

func TestDashboardStateDescribesTheWiredApplication(t *testing.T) {
	d := newDashboard(t, 0, &dashNode{}, &Counter{})
	time.Sleep(60 * time.Millisecond)
	s := d.state()

	if len(s.Nodes) != 2 {
		t.Fatalf("nodes = %d, want 2", len(s.Nodes))
	}
	var dash *NodeView
	for i := range s.Nodes {
		if s.Nodes[i].Name == "dash_node" {
			dash = &s.Nodes[i]
		}
	}
	if dash == nil {
		t.Fatalf("dash_node missing from %v", s.Nodes)
	}
	if dash.State != "active" {
		t.Errorf("state = %q, want active", dash.State)
	}
	if dash.Processed == 0 {
		t.Error("no callbacks counted")
	}
	if dash.MailboxCap == 0 {
		t.Error("mailbox capacity not reported")
	}

	// Every declaration on the node appears in the inventory, with the
	// live count attached.
	kinds := map[EndpointKind]Endpoint{}
	for _, e := range dash.Endpoints {
		kinds[e.Kind] = e
	}
	for _, want := range []EndpointKind{EndpointTimer, EndpointPub, EndpointService} {
		if _, ok := kinds[want]; !ok {
			t.Errorf("endpoint kind %q missing from %+v", want, dash.Endpoints)
		}
	}
	if got := kinds[EndpointPub]; got.Name != "pings" || got.QoS != "reliable" || got.Counts == 0 {
		t.Errorf("publisher endpoint = %+v", got)
	}
	if got := kinds[EndpointTimer]; got.Rate != "100hz" || got.Counts == 0 {
		t.Errorf("timer endpoint = %+v", got)
	}

	// Parameters are reported with their current value, not the default.
	if len(dash.Params) != 1 || dash.Params[0].Name != "limit" || dash.Params[0].Value != "1.5" {
		t.Errorf("params = %+v", dash.Params)
	}

	// The topic view is the graph: one publisher, one subscriber.
	var pings *TopicView
	for i := range s.Topics {
		if s.Topics[i].Name == "pings" {
			pings = &s.Topics[i]
		}
	}
	if pings == nil {
		t.Fatalf("topic pings missing from %+v", s.Topics)
	}
	if len(pings.Pubs) != 1 || pings.Pubs[0] != "dash_node" || len(pings.Subs) != 1 || pings.Subs[0] != "counter" {
		t.Errorf("topic view = %+v", pings)
	}
	if pings.Sent == 0 || pings.Recvd == 0 {
		t.Errorf("topic counters = %d sent, %d received", pings.Sent, pings.Recvd)
	}
	if len(s.App.Bringup) != 2 || s.App.Bringup[0] != "dash_node" {
		t.Errorf("bringup order = %v, want the publisher first", s.App.Bringup)
	}
	if len(s.Metrics) == 0 {
		t.Error("no metrics in the snapshot")
	}
}

func TestDashboardSetsParameters(t *testing.T) {
	n := &dashNode{}
	d := newDashboard(t, 0, n, &Counter{})

	if err := d.setParam("dash_node", "limit", "0.25"); err != nil {
		t.Fatal(err)
	}
	if got := n.Limit.Get(); got != 0.25 {
		t.Errorf("Limit.Get() = %v, want 0.25 — the dashboard must go through the same handle as ros2 param set", got)
	}
	// Types are enforced exactly as the parameter services enforce them.
	if err := d.setParam("dash_node", "limit", "fast"); err == nil {
		t.Error("expected a type error for a non-numeric value")
	}
	if err := d.setParam("dash_node", "nope", "1"); err == nil {
		t.Error("expected an error for an unknown parameter")
	}
	if err := d.setParam("elsewhere", "limit", "1"); err == nil {
		t.Error("expected an error for a node in another process")
	}
}

func TestDashboardDrivesLifecycle(t *testing.T) {
	d := newDashboard(t, 0, &dashNode{}, &Counter{})
	if err := d.transition("counter", "deactivate"); err != nil {
		t.Fatal(err)
	}
	if got := findNode(t, d.state(), "counter").State; got != "inactive" {
		t.Errorf("state = %q, want inactive", got)
	}
	if err := d.transition("counter", "activate"); err != nil {
		t.Fatal(err)
	}
	if got := findNode(t, d.state(), "counter").State; got != "active" {
		t.Errorf("state = %q, want active", got)
	}
	if err := d.transition("counter", "explode"); err == nil {
		t.Error("expected an error for an unknown transition")
	}
}

// The trace view's whole point is the causal chain, so the timer span that
// published must come out as the parent of the subscription span it caused.
func TestDashboardTracesAreCausalChains(t *testing.T) {
	d := newDashboard(t, 200, &dashNode{}, &Counter{})
	time.Sleep(80 * time.Millisecond)

	traces := d.ring.traces(10)
	if len(traces) == 0 {
		t.Fatal("no traces recorded")
	}
	var chain *TraceView
	for i := range traces {
		if len(traces[i].Spans) > 1 {
			chain = &traces[i]
			break
		}
	}
	if chain == nil {
		t.Fatalf("no multi-span trace among %d traces", len(traces))
	}
	if chain.Spans[0].Kind != string(SpanTimer) || chain.Spans[0].Depth != 0 {
		t.Errorf("trace does not start at the timer: %+v", chain.Spans[0])
	}
	child := chain.Spans[1]
	if child.Depth != 1 || child.Parent != chain.Spans[0].ID {
		t.Errorf("second span is not a child of the first: %+v", child)
	}
	if child.Node != "counter" || child.Kind != string(SpanSubscription) {
		t.Errorf("expected the subscription it caused, got %+v", child)
	}
	if child.OffsetMS < 0 {
		t.Errorf("child starts before the trace: %v", child.OffsetMS)
	}
}

func TestTraceRingIsBounded(t *testing.T) {
	r := newTraceRing(3)
	for i := 0; i < 10; i++ {
		r.Export(Span{Node: "n", Kind: SpanTimer, Context: TraceContext{TraceID: [16]byte{byte(i)}}})
	}
	if got := len(r.all()); got != 3 {
		t.Errorf("ring holds %d spans, want its capacity of 3", got)
	}
}

func TestDashboardHTTPEndpoints(t *testing.T) {
	a := newTestApp(t, &dashNode{}, &Counter{})
	srv := serveDashboard("127.0.0.1:0", a, "inproc", 0)
	if srv == nil {
		t.Fatal("dashboard did not start")
	}
	t.Cleanup(func() { srv.Close() })
	h := srv.Handler

	// The page is self-contained: an artifact fetched from a CDN would not
	// load on a robot with no route to the internet.
	rec := httptest.NewRecorder()
	h.ServeHTTP(rec, httptest.NewRequest("GET", "/", nil))
	if rec.Code != 200 {
		t.Fatalf("GET / = %d", rec.Code)
	}
	page := rec.Body.String()
	if !strings.Contains(page, "<title>conductor</title>") {
		t.Error("dashboard page is not the expected document")
	}
	for _, bad := range []string{"http://", "https://", "src=\"//"} {
		if strings.Contains(page, bad) {
			t.Errorf("the page references an external resource (%q); it must be self-contained", bad)
		}
	}

	rec = httptest.NewRecorder()
	h.ServeHTTP(rec, httptest.NewRequest("GET", "/api/state", nil))
	if rec.Code != 200 {
		t.Fatalf("GET /api/state = %d", rec.Code)
	}
	var state DashboardState
	if err := json.Unmarshal(rec.Body.Bytes(), &state); err != nil {
		t.Fatalf("state is not valid JSON: %v", err)
	}
	if len(state.Nodes) != 2 {
		t.Errorf("state has %d nodes", len(state.Nodes))
	}

	// Parameter changes are a POST, and a GET must not change anything.
	rec = httptest.NewRecorder()
	h.ServeHTTP(rec, httptest.NewRequest("GET", "/api/params", nil))
	if rec.Code != http.StatusMethodNotAllowed {
		t.Errorf("GET /api/params = %d, want 405", rec.Code)
	}

	body := bytes.NewBufferString(`{"node":"dash_node","name":"limit","value":"2.5"}`)
	rec = httptest.NewRecorder()
	h.ServeHTTP(rec, httptest.NewRequest("POST", "/api/params", body))
	if rec.Code != 200 {
		t.Errorf("POST /api/params = %d: %s", rec.Code, rec.Body)
	}

	rec = httptest.NewRecorder()
	h.ServeHTTP(rec, httptest.NewRequest("GET", "/metrics", nil))
	if rec.Code != 200 || !strings.Contains(rec.Body.String(), "conductor_") {
		t.Errorf("the dashboard should also expose /metrics: %d", rec.Code)
	}
}

func findNode(t *testing.T, s DashboardState, name string) NodeView {
	t.Helper()
	for _, n := range s.Nodes {
		if n.Name == name {
			return n
		}
	}
	t.Fatalf("no node %q in state", name)
	return NodeView{}
}

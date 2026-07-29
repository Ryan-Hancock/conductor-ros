package conductor

import (
	"embed"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"log/slog"
	"net/http"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"sync"
	"time"
)

// The dashboard is the local development portal: the application graph, live
// message rates, lifecycle state, parameters and recent traces, in a browser.
//
// It exists because the runtime already holds everything worth showing — it
// wired every endpoint, it owns every callback, it propagates the trace
// context — and a ROS developer's alternative is `ros2 topic hz` in one
// terminal, `ros2 param get` in another, and no traces at all.
//
// It is served by the application process itself: no collector, no database,
// nothing to deploy alongside. What it shows is this process, which is the
// honest scope — in a per-node deployment each unit serves its own.

//go:embed dashboard.html
var dashboardFS embed.FS

// DashboardState is the whole view, refreshed by polling. It is a snapshot:
// counters are absolute, and rates are derived by the page from successive
// snapshots, so no rate window is baked into the runtime.
type DashboardState struct {
	App      AppView       `json:"app"`
	Nodes    []NodeView    `json:"nodes"`
	Topics   []TopicView   `json:"topics"`
	Missions []MissionView `json:"missions"`
	Frames   []FrameView   `json:"frames"`
	Metrics  []Metric      `json:"metrics"`
	Traces   []TraceView   `json:"traces"`
	Now      time.Time     `json:"now"`
}

// MissionView is a node's task machine: the declared steps, and which one is
// running. The dashboard draws the same machine `conductor check` prints and
// mission.dot renders, with the live position marked on it.
type MissionView struct {
	Node    string            `json:"node"`
	Name    string            `json:"name"`
	Status  string            `json:"status"`
	Step    string            `json:"step"`
	Attempt int               `json:"attempt"`
	Elapsed float64           `json:"elapsed_seconds"`
	Err     string            `json:"err,omitempty"`
	Steps   []MissionStepView `json:"steps"`
}

type MissionStepView struct {
	Name    string `json:"name"`
	Next    string `json:"next"`
	Fail    string `json:"fail,omitempty"`
	Timeout string `json:"timeout,omitempty"`
	Retry   int    `json:"retry,omitempty"`
	Entries uint64 `json:"entries"`
	Current bool   `json:"current"`
}

// FrameView is one link of the declared transform tree.
type FrameView struct {
	Parent  string     `json:"parent"`
	Child   string     `json:"child"`
	XYZ     [3]float64 `json:"xyz"`
	RPY     [3]float64 `json:"rpy"`
	Dynamic bool       `json:"dynamic"`
	By      string     `json:"by,omitempty"`
}

type AppView struct {
	Name      string    `json:"name"`
	Transport string    `json:"transport"`
	Host      string    `json:"host"`
	PID       int       `json:"pid"`
	Started   time.Time `json:"started"`
	Uptime    float64   `json:"uptime_seconds"`
	Bringup   []string  `json:"bringup_order"`
	Tracing   bool      `json:"tracing"`
}

type NodeView struct {
	Name         string      `json:"name"`
	State        string      `json:"state"`
	Processed    uint64      `json:"processed"`
	Dropped      uint64      `json:"dropped"`
	MailboxDepth int         `json:"mailbox_depth"`
	MailboxCap   int         `json:"mailbox_cap"`
	Endpoints    []Endpoint  `json:"endpoints"`
	Params       []ParamView `json:"params"`
	DependsOn    []string    `json:"depends_on"`
}

type ParamView struct {
	Name  string `json:"name"`
	Type  string `json:"type"`
	Value string `json:"value"`
}

// TopicView is the graph edge: who publishes, who subscribes, and whether
// the other end is outside this process.
type TopicView struct {
	Name  string   `json:"name"`
	Type  string   `json:"type"`
	QoS   string   `json:"qos"`
	Pubs  []string `json:"pubs"`
	Subs  []string `json:"subs"`
	Sent  uint64   `json:"sent"`
	Recvd uint64   `json:"received"`
}

type TraceView struct {
	ID       string     `json:"id"`
	Start    time.Time  `json:"start"`
	Duration float64    `json:"duration_ms"`
	Spans    []SpanView `json:"spans"`
}

type SpanView struct {
	ID       string  `json:"id"`
	Parent   string  `json:"parent"`
	Node     string  `json:"node"`
	Kind     string  `json:"kind"`
	Name     string  `json:"name"`
	OffsetMS float64 `json:"offset_ms"`
	Duration float64 `json:"duration_ms"`
	Err      string  `json:"err,omitempty"`
	Depth    int     `json:"depth"`
}

// traceRing keeps the most recent spans for the dashboard's trace view. It is
// bounded: a robot runs for weeks, and the interesting trace is a recent one.
type traceRing struct {
	mu    sync.Mutex
	spans []Span
	next  int
	cap   int
}

func newTraceRing(capacity int) *traceRing {
	return &traceRing{spans: make([]Span, 0, capacity), cap: capacity}
}

func (r *traceRing) Export(s Span) {
	r.mu.Lock()
	defer r.mu.Unlock()
	if len(r.spans) < r.cap {
		r.spans = append(r.spans, s)
		return
	}
	r.spans[r.next] = s
	r.next = (r.next + 1) % r.cap
}

func (r *traceRing) all() []Span {
	r.mu.Lock()
	defer r.mu.Unlock()
	out := make([]Span, len(r.spans))
	copy(out, r.spans)
	return out
}

// traces groups recorded spans into traces, most recent first, and orders
// each trace's spans as a causal tree: this is the view that answers "what
// did that lidar frame set off?".
func (r *traceRing) traces(limit int) []TraceView {
	byTrace := map[string][]Span{}
	for _, s := range r.all() {
		id := s.Context.TraceIDString()
		byTrace[id] = append(byTrace[id], s)
	}
	out := make([]TraceView, 0, len(byTrace))
	for id, spans := range byTrace {
		sort.Slice(spans, func(i, j int) bool { return spans[i].Start.Before(spans[j].Start) })
		start := spans[0].Start
		end := start
		for _, s := range spans {
			if fin := s.Start.Add(s.Duration); fin.After(end) {
				end = fin
			}
		}
		out = append(out, TraceView{
			ID:       id,
			Start:    start,
			Duration: float64(end.Sub(start).Microseconds()) / 1000,
			Spans:    orderSpans(spans, start),
		})
	}
	sort.Slice(out, func(i, j int) bool { return out[i].Start.After(out[j].Start) })
	if len(out) > limit {
		out = out[:limit]
	}
	return out
}

// orderSpans walks the parent/child relations depth-first so the rendered
// list reads as the causal chain rather than as arrival order.
func orderSpans(spans []Span, start time.Time) []SpanView {
	children := map[string][]Span{}
	present := map[string]bool{}
	for _, s := range spans {
		present[hex.EncodeToString(s.Context.SpanID[:])] = true
	}
	var roots []Span
	for _, s := range spans {
		parent := hex.EncodeToString(s.ParentID[:])
		if s.ParentID == [8]byte{} || !present[parent] {
			roots = append(roots, s)
			continue
		}
		children[parent] = append(children[parent], s)
	}
	var out []SpanView
	var walk func(s Span, depth int)
	walk = func(s Span, depth int) {
		id := hex.EncodeToString(s.Context.SpanID[:])
		v := SpanView{
			ID:       id,
			Parent:   hex.EncodeToString(s.ParentID[:]),
			Node:     s.Node,
			Kind:     string(s.Kind),
			Name:     s.Name,
			OffsetMS: float64(s.Start.Sub(start).Microseconds()) / 1000,
			Duration: float64(s.Duration.Microseconds()) / 1000,
			Depth:    depth,
		}
		if s.Err != nil {
			v.Err = s.Err.Error()
		}
		out = append(out, v)
		kids := children[id]
		sort.Slice(kids, func(i, j int) bool { return kids[i].Start.Before(kids[j].Start) })
		for _, k := range kids {
			walk(k, depth+1)
		}
	}
	for _, root := range roots {
		walk(root, 0)
	}
	return out
}

// dashboard is the server; it holds the pieces of runtime state the views
// are built from.
type dashboard struct {
	app       *app
	transport string
	started   time.Time
	ring      *traceRing
}

// state assembles the current view.
func (d *dashboard) state() DashboardState { return d.snapshot(true) }

// summary is the same view without the metric table and the trace ring: what
// the fleet view fans out for, several times a second, across a deployment.
// Sending the whole thing to every aggregator would make the poll cost of a
// robot grow with the number of people watching it.
func (d *dashboard) summary() DashboardState { return d.snapshot(false) }

func (d *dashboard) snapshot(full bool) DashboardState {
	rt := d.app.rt
	endpoints := rt.endpointsSnapshot()
	byNode := map[string][]Endpoint{}
	for _, e := range endpoints {
		byNode[e.Node] = append(byNode[e.Node], e)
	}

	names := make([]string, 0, len(rt.nodes))
	for _, nr := range rt.nodes {
		names = append(names, nr.name)
	}
	order, _ := BringupOrder(names, rt.deps)

	nodes := make([]NodeView, 0, len(rt.nodes))
	for _, nr := range rt.nodes {
		params := make([]ParamView, 0, len(nr.params))
		for _, h := range nr.params {
			params = append(params, ParamView{
				Name:  h.name,
				Type:  h.typeOf.String(),
				Value: fmt.Sprint(h.get()),
			})
		}
		var deps []string
		if nd := rt.deps[nr.name]; nd != nil {
			for name := range nd.consumes {
				for other, od := range rt.deps {
					if other != nr.name && od.provides[name] {
						deps = appendUnique(deps, other)
					}
				}
			}
			sort.Strings(deps)
		}
		eps := byNode[nr.name]
		if eps == nil {
			eps = []Endpoint{}
		}
		nodes = append(nodes, NodeView{
			Name:         nr.name,
			State:        nr.lifecycle.State().String(),
			Processed:    nr.processed.Load(),
			Dropped:      nr.dropped.Load(),
			MailboxDepth: len(nr.mailbox),
			MailboxCap:   cap(nr.mailbox),
			Endpoints:    eps,
			Params:       params,
			DependsOn:    orEmpty(deps),
		})
	}
	sort.Slice(nodes, func(i, j int) bool { return nodes[i].Name < nodes[j].Name })

	host, _ := os.Hostname()
	name := "conductor"
	if exe, err := os.Executable(); err == nil {
		name = filepath.Base(exe)
	}
	metrics, traces := []Metric{}, []TraceView{}
	if full {
		metrics, traces = MetricsSnapshot(), d.tracesOrNil()
	}
	return DashboardState{
		App: AppView{
			Name:      name,
			Transport: d.transport,
			Host:      host,
			PID:       os.Getpid(),
			Started:   d.started,
			Uptime:    time.Since(d.started).Seconds(),
			Bringup:   order,
			Tracing:   d.ring != nil,
		},
		Nodes:    nodes,
		Topics:   topicViews(endpoints),
		Missions: missionViews(rt),
		Frames:   frameViews(rt.frames),
		Metrics:  metrics,
		Traces:   traces,
		Now:      time.Now(),
	}
}

// missionViews describes every task machine in this process, live.
func missionViews(rt *runtimeState) []MissionView {
	out := []MissionView{}
	for _, nr := range rt.nodes {
		r := nr.mission
		if r == nil {
			continue
		}
		s := r.snapshot()
		v := MissionView{
			Node:    nr.name,
			Name:    r.name,
			Status:  string(s.Status),
			Step:    s.Step,
			Attempt: s.Attempt,
			Steps:   make([]MissionStepView, 0, len(r.order)),
		}
		if !s.Since.IsZero() {
			v.Elapsed = time.Since(s.Since).Seconds()
		}
		if s.Err != nil {
			v.Err = s.Err.Error()
		}
		for _, def := range r.order {
			v.Steps = append(v.Steps, MissionStepView{
				Name:    def.name,
				Next:    def.next,
				Fail:    def.fail,
				Timeout: durationOrEmpty(def.timeout),
				Retry:   def.retry,
				Entries: def.entries.Load(),
				Current: s.Status == MissionRunning && def.name == s.Step,
			})
		}
		out = append(out, v)
	}
	sort.Slice(out, func(i, j int) bool { return out[i].Node < out[j].Node })
	return out
}

func frameViews(t *FrameTree) []FrameView {
	out := []FrameView{}
	if t == nil {
		return out
	}
	for _, tf := range t.Transforms {
		out = append(out, FrameView{
			Parent: tf.Parent, Child: tf.Child, XYZ: tf.XYZ, RPY: tf.RPY,
			Dynamic: tf.Dynamic, By: tf.By,
		})
	}
	return out
}

func durationOrEmpty(d time.Duration) string {
	if d == 0 {
		return ""
	}
	return d.String()
}

func (d *dashboard) tracesOrNil() []TraceView {
	if d.ring == nil {
		return []TraceView{}
	}
	return d.ring.traces(40)
}

// topicViews turns the endpoint inventory into the topic graph, which is the
// same projection `conductor check` prints — from live state.
func topicViews(endpoints []Endpoint) []TopicView {
	index := map[string]*TopicView{}
	get := func(name string) *TopicView {
		t, ok := index[name]
		if !ok {
			t = &TopicView{Name: name}
			index[name] = t
		}
		return t
	}
	for _, e := range endpoints {
		switch e.Kind {
		case EndpointPub:
			t := get(e.Name)
			t.Pubs = appendUnique(t.Pubs, e.Node)
			t.Type, t.QoS = e.Type, e.QoS
			t.Sent += e.Counts
		case EndpointSub:
			t := get(e.Name)
			t.Subs = appendUnique(t.Subs, e.Node)
			t.Type, t.QoS = e.Type, e.QoS
			t.Recvd += e.Counts
		}
	}
	out := make([]TopicView, 0, len(index))
	for _, t := range index {
		// Empty, never null: the page treats these as lists, and a topic
		// with no local publisher (an external one) is the normal case.
		t.Pubs, t.Subs = orEmpty(t.Pubs), orEmpty(t.Subs)
		out = append(out, *t)
	}
	sort.Slice(out, func(i, j int) bool { return out[i].Name < out[j].Name })
	return out
}

// orEmpty keeps JSON list fields as [] rather than null, so the page can
// treat every list as a list.
func orEmpty(s []string) []string {
	if s == nil {
		return []string{}
	}
	return s
}

func appendUnique(list []string, v string) []string {
	for _, x := range list {
		if x == v {
			return list
		}
	}
	return append(list, v)
}

// setParam applies a parameter change from the dashboard, through the same
// handle the ROS parameter services use — so a value set here behaves exactly
// like one set with `ros2 param set`, type checking included.
func (d *dashboard) setParam(node, name, value string) error {
	for _, nr := range d.app.rt.nodes {
		if nr.name != node {
			continue
		}
		for _, h := range nr.params {
			if h.name == name {
				return h.set(value)
			}
		}
		return fmt.Errorf("node %q has no parameter %q", node, name)
	}
	return fmt.Errorf("no node %q in this process", node)
}

// serveDashboard starts the portal. traceDepth > 0 records that many recent
// spans for the trace view; it implies tracing, which costs a span per
// callback, so it is opt-in with the dashboard rather than always on.
func serveDashboard(addr string, a *app, transport string, traceDepth int) *http.Server {
	d := &dashboard{app: a, transport: transport, started: time.Now()}
	if traceDepth > 0 {
		d.ring = newTraceRing(traceDepth)
		AddExporter(d.ring)
	}

	mux := http.NewServeMux()
	page, err := dashboardFS.ReadFile("dashboard.html")
	if err != nil {
		slog.Error("conductor: dashboard asset missing", "err", err)
		return nil
	}
	mux.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/" {
			http.NotFound(w, r)
			return
		}
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		w.Write(page)
	})
	mux.HandleFunc("/api/state", func(w http.ResponseWriter, r *http.Request) {
		writeJSON(w, d.state())
	})
	// What a fleet aggregator polls: the same view, minus the parts only a
	// human reading this one process needs.
	mux.HandleFunc("/api/summary", func(w http.ResponseWriter, r *http.Request) {
		writeJSON(w, d.summary())
	})
	mux.HandleFunc("/api/params", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "POST a {node, name, value} object", http.StatusMethodNotAllowed)
			return
		}
		var req struct{ Node, Name, Value string }
		if err := json.NewDecoder(http.MaxBytesReader(w, r.Body, 1<<16)).Decode(&req); err != nil {
			http.Error(w, err.Error(), http.StatusBadRequest)
			return
		}
		if err := d.setParam(req.Node, req.Name, req.Value); err != nil {
			http.Error(w, err.Error(), http.StatusBadRequest)
			return
		}
		writeJSON(w, map[string]any{"ok": true})
	})
	mux.HandleFunc("/api/lifecycle", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "POST a {node, transition} object", http.StatusMethodNotAllowed)
			return
		}
		var req struct{ Node, Transition string }
		if err := json.NewDecoder(http.MaxBytesReader(w, r.Body, 1<<16)).Decode(&req); err != nil {
			http.Error(w, err.Error(), http.StatusBadRequest)
			return
		}
		if err := d.transition(req.Node, req.Transition); err != nil {
			http.Error(w, err.Error(), http.StatusBadRequest)
			return
		}
		writeJSON(w, map[string]any{"ok": true})
	})
	mux.HandleFunc("/metrics", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/plain; version=0.0.4")
		fmt.Fprint(w, MetricsText())
	})

	srv := &http.Server{Addr: addr, Handler: mux, ReadHeaderTimeout: 5 * time.Second}
	go func() {
		if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			slog.Error("conductor: dashboard server", "err", err)
		}
	}()
	slog.Info("conductor: dashboard available", "addr", "http://"+addr+"/")
	return srv
}

// transition drives a lifecycle transition by name, the same one `ros2
// lifecycle set` would.
func (d *dashboard) transition(node, name string) error {
	var t Transition
	switch strings.ToLower(name) {
	case "configure":
		t = TransitionConfigure
	case "activate":
		t = TransitionActivate
	case "deactivate":
		t = TransitionDeactivate
	case "cleanup":
		t = TransitionCleanup
	default:
		return fmt.Errorf("unknown transition %q (configure, activate, deactivate, cleanup)", name)
	}
	for _, nr := range d.app.rt.nodes {
		if nr.name != node {
			continue
		}
		if ok, err := nr.lifecycle.transition(t); !ok {
			return fmt.Errorf("%s: %w", node, err)
		}
		return nil
	}
	return fmt.Errorf("no node %q in this process", node)
}

func writeJSON(w http.ResponseWriter, v any) {
	w.Header().Set("Content-Type", "application/json")
	enc := json.NewEncoder(w)
	enc.SetIndent("", "  ")
	if err := enc.Encode(v); err != nil {
		slog.Error("conductor: dashboard encode", "err", err)
	}
}

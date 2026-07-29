package conductor

import (
	"context"
	"embed"
	"encoding/json"
	"fmt"
	"log/slog"
	"net"
	"net/http"
	"net/url"
	"sort"
	"strings"
	"sync"
	"time"
)

// A zenoh deployment is one process per node, and each of them serves an
// honest view of itself: the nodes it runs, and everything else marked as
// "(ROS graph)". That is exactly the picture ROS 2 already gives you, one
// terminal at a time.
//
// The fleet view fans out over those processes and merges what they report
// into one graph. The merge is where the value is, because it can see what
// no single process can:
//
//   - a topic whose subscriber is up and whose publisher is not;
//   - two processes that disagree about a topic's type or QoS, which is what
//     a half-finished deploy looks like from the outside;
//   - two robots running different transform trees, i.e. one stale
//     calibration;
//   - a unit that is simply not answering.
//
// It is a client of the same HTTP API the single-process dashboard serves, so
// there is no second source of truth and nothing new on the wire.

//go:embed fleet.html
var fleetFS embed.FS

// Peer is one conductor process to aggregate: where to reach its dashboard,
// and what to call it.
type Peer struct {
	Name string `json:"name"` // label, usually the node it runs
	Host string `json:"host"` // machine label, for a fleet of robots
	URL  string `json:"url"`  // base URL of that process's dashboard
}

// FleetState is the merged view of every peer.
type FleetState struct {
	Now       time.Time     `json:"now"`
	Processes []ProcessView `json:"processes"`
	Order     []string      `json:"bringup_order"`
	Topics    []FleetTopic  `json:"topics"`
	Missions  []MissionView `json:"missions"`
	Frames    []FrameView   `json:"frames"`
	Findings  []Finding     `json:"findings"`
	Reachable int           `json:"reachable"`

	// framesFrom is the peer whose transform tree the merged one came from,
	// so a disagreement can name both sides.
	framesFrom string
}

// ProcessView is one peer's answer, or the reason there wasn't one.
type ProcessView struct {
	Peer
	Label     string      `json:"label"` // how this process names its nodes in the merged graph
	OK        bool        `json:"ok"`
	Err       string      `json:"err,omitempty"`
	LatencyMS float64     `json:"latency_ms"`
	App       AppView     `json:"app"`
	Nodes     []NodeView  `json:"nodes"`
	Frames    []FrameView `json:"frames"`

	// The peer's own topic and mission views feed the merge; the merged
	// ones replace them, so they are not served again per process.
	topics   []TopicView
	missions []MissionView
}

// FleetTopic is a topic as the whole deployment sees it: publishers and
// subscribers named by process, so a cross-process edge is visible as an edge
// rather than as "(ROS graph)" at both ends.
type FleetTopic struct {
	Name     string          `json:"name"`
	Type     string          `json:"type"`
	QoS      string          `json:"qos"`
	Pubs     []FleetEndpoint `json:"pubs"`
	Subs     []FleetEndpoint `json:"subs"`
	Sent     uint64          `json:"sent"`
	Recvd    uint64          `json:"received"`
	External bool            `json:"external"` // no local endpoint on either side
}

type FleetEndpoint struct {
	Process string `json:"process"`
	Node    string `json:"node"`
	// Key is the node's name in the merged graph, so an edge can be drawn
	// without the page re-deriving how a process names its nodes.
	Key    string `json:"key"`
	Counts uint64 `json:"counts"`
}

// Finding is something only the merged view can notice.
type Finding struct {
	Severity string `json:"severity"` // "error" or "warning"
	Code     string `json:"code"`
	Msg      string `json:"msg"`
}

// FleetOptions is what the merge knows that the processes do not.
type FleetOptions struct {
	// External topics are declared in conductor.json as belonging to nodes
	// outside this application. Without them the merge would report a
	// driver's topic as unpublished, when the truth is that publishing it
	// was never ours to do.
	External map[string]bool
}

// Fleet queries every peer concurrently and merges the answers. A peer that
// does not answer is reported as down rather than dropped: a fleet view whose
// value is knowing what is missing must not quietly show less.
func Fleet(ctx context.Context, peers []Peer, timeout time.Duration) FleetState {
	return FleetWith(ctx, peers, timeout, FleetOptions{})
}

// FleetWith is Fleet with what the application declares statically.
func FleetWith(ctx context.Context, peers []Peer, timeout time.Duration, opts FleetOptions) FleetState {
	client := &http.Client{Timeout: timeout, Transport: &http.Transport{
		Proxy:               nil, // a robot on a LAN is not behind the shell's proxy
		DialContext:         (&net.Dialer{Timeout: timeout}).DialContext,
		TLSHandshakeTimeout: timeout,
	}}

	views := make([]ProcessView, len(peers))
	var wg sync.WaitGroup
	for i, p := range peers {
		wg.Add(1)
		go func(i int, p Peer) {
			defer wg.Done()
			views[i] = fetchPeer(ctx, client, p)
		}(i, p)
	}
	wg.Wait()

	return mergeFleet(views, opts)
}

// fetchPeer reads one process's summary.
func fetchPeer(ctx context.Context, client *http.Client, p Peer) ProcessView {
	// Empty, never null: a process that is down is still rendered, and the
	// page treats every list as a list.
	out := ProcessView{Peer: p, Nodes: []NodeView{}, Frames: []FrameView{}}
	endpoint, err := url.JoinPath(p.URL, "/api/summary")
	if err != nil {
		out.Err = err.Error()
		return out
	}
	req, err := http.NewRequestWithContext(ctx, http.MethodGet, endpoint, nil)
	if err != nil {
		out.Err = err.Error()
		return out
	}
	start := time.Now()
	res, err := client.Do(req)
	if err != nil {
		out.Err = trimURL(err.Error())
		return out
	}
	defer res.Body.Close()
	out.LatencyMS = float64(time.Since(start).Microseconds()) / 1000
	if res.StatusCode != http.StatusOK {
		out.Err = fmt.Sprintf("HTTP %s", res.Status)
		return out
	}
	var state DashboardState
	if err := json.NewDecoder(res.Body).Decode(&state); err != nil {
		out.Err = "not a conductor dashboard: " + err.Error()
		return out
	}
	out.OK = true
	out.App, out.Nodes, out.Frames = state.App, state.Nodes, state.Frames
	if out.Nodes == nil {
		out.Nodes = []NodeView{}
	}
	if out.Frames == nil {
		out.Frames = []FrameView{}
	}
	out.topics, out.missions = state.Topics, state.Missions
	return out
}

// trimURL keeps a transport error readable: net/http prefixes the whole
// request line, which is noise in a table of hosts.
func trimURL(msg string) string {
	if i := strings.LastIndex(msg, ": "); i > 0 && strings.HasPrefix(msg, "Get ") {
		return msg[i+2:]
	}
	return msg
}

// mergeFleet builds the union graph and the findings.
func mergeFleet(views []ProcessView, opts FleetOptions) FleetState {
	out := FleetState{
		Now:       time.Now(),
		Processes: views,
		Order:     []string{},
		Topics:    []FleetTopic{},
		Missions:  []MissionView{},
		Frames:    []FrameView{},
		Findings:  []Finding{},
	}
	topics := map[string]*FleetTopic{}
	var order []string
	deps := map[string]*nodeDeps{}
	seenNode := map[string]bool{}

	add := func(name string) *FleetTopic {
		t, ok := topics[name]
		if !ok {
			t = &FleetTopic{
				Name: name, External: opts.External[name],
				Pubs: []FleetEndpoint{}, Subs: []FleetEndpoint{},
			}
			topics[name] = t
		}
		return t
	}

	for i := range views {
		v := &views[i]
		v.Label = v.label()
		if !v.OK {
			out.Findings = append(out.Findings, Finding{"error", "FLEET01",
				fmt.Sprintf("%s is not answering on %s: %s", v.label(), v.URL, v.Err)})
			// A process that is not answering still belongs in the graph:
			// the point of the view is to show the hole, not to close it.
			if !seenNode[v.label()] {
				seenNode[v.label()] = true
				order = append(order, v.label())
				deps[v.label()] = newNodeDeps()
			}
			continue
		}
		out.Reachable++

		for _, n := range v.Nodes {
			key := nodeKey(v.label(), n.Name)
			if !seenNode[key] {
				seenNode[key] = true
				order = append(order, key)
				deps[key] = newNodeDeps()
			}
			if n.State != "active" {
				out.Findings = append(out.Findings, Finding{"warning", "FLEET02",
					fmt.Sprintf("%s is %s, not active", key, n.State)})
			}
			if n.Dropped > 0 {
				out.Findings = append(out.Findings, Finding{"warning", "FLEET03",
					fmt.Sprintf("%s has dropped %d message(s): its mailbox is overrunning", key, n.Dropped)})
			}
		}

		for _, t := range v.topics {
			ft := add(t.Name)
			mergeTopicFacts(&out, v, ft, t)
			for _, node := range t.Pubs {
				ft.Pubs = append(ft.Pubs, FleetEndpoint{
					Process: v.label(), Node: node, Key: nodeKey(v.label(), node), Counts: t.Sent})
				ft.Sent += t.Sent
				nodeDepsFor(deps, v.label(), node).provides[t.Name] = true
			}
			for _, node := range t.Subs {
				ft.Subs = append(ft.Subs, FleetEndpoint{
					Process: v.label(), Node: node, Key: nodeKey(v.label(), node), Counts: t.Recvd})
				ft.Recvd += t.Recvd
				nodeDepsFor(deps, v.label(), node).consumes[t.Name] = true
			}
		}

		for _, m := range v.missions {
			m.Node = nodeKey(v.label(), m.Node)
			out.Missions = append(out.Missions, m)
		}
		mergeFrames(&out, v)
	}

	for _, t := range topics {
		sort.Slice(t.Pubs, func(i, j int) bool { return t.Pubs[i].Node < t.Pubs[j].Node })
		sort.Slice(t.Subs, func(i, j int) bool { return t.Subs[i].Node < t.Subs[j].Node })
		if len(t.Pubs) == 0 && len(t.Subs) > 0 && !t.External {
			// Nothing in the fleet publishes it and the application does not
			// declare it as coming from outside — so either a process is
			// down, or the graph is wrong.
			out.Findings = append(out.Findings, Finding{"error", "FLEET04",
				fmt.Sprintf("topic %q: %d subscriber(s) in this deployment and no publisher answering",
					t.Name, len(t.Subs))})
		}
		out.Topics = append(out.Topics, *t)
	}
	sort.Slice(out.Topics, func(i, j int) bool { return out.Topics[i].Name < out.Topics[j].Name })
	sort.Slice(out.Missions, func(i, j int) bool { return out.Missions[i].Node < out.Missions[j].Node })

	out.Order, _ = BringupOrder(order, deps)
	sortFindings(out.Findings)
	return out
}

// mergeTopicFacts records a topic's type and QoS, and reports peers that
// disagree — which is what a deployment caught halfway between two releases
// looks like from the outside.
func mergeTopicFacts(out *FleetState, v *ProcessView, ft *FleetTopic, t TopicView) {
	facts := []struct {
		kind string
		seen *string
		got  string
	}{
		{"type", &ft.Type, t.Type},
		{"qos", &ft.QoS, t.QoS},
	}
	for _, f := range facts {
		if f.got == "" {
			continue
		}
		if *f.seen == "" {
			*f.seen = f.got
			continue
		}
		if *f.seen != f.got {
			out.Findings = append(out.Findings, Finding{"error", "FLEET05",
				fmt.Sprintf("topic %q: processes disagree on %s (%s vs %s from %s) — a partial deploy?",
					ft.Name, f.kind, *f.seen, f.got, v.label())})
		}
	}
}

// mergeFrames keeps one transform tree for the fleet and reports any peer
// whose tree differs: two robots with different calibration, or one that
// missed a release.
func mergeFrames(out *FleetState, v *ProcessView) {
	if len(v.Frames) == 0 {
		return
	}
	if len(out.Frames) == 0 {
		out.Frames = v.Frames
		out.framesFrom = v.label()
		return
	}
	if !sameFrames(out.Frames, v.Frames) {
		out.Findings = append(out.Findings, Finding{"error", "FLEET06",
			fmt.Sprintf("%s reports a different transform tree from %s: one of them is running an older release",
				v.label(), out.framesFrom)})
	}
}

func sameFrames(a, b []FrameView) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

func sortFindings(f []Finding) {
	sort.SliceStable(f, func(i, j int) bool { return f[i].Severity == "error" && f[j].Severity != "error" })
}

// nodeDepsFor is the merged graph's edge collector, tolerant of a process
// that reports an endpoint on a node it did not list.
func nodeDepsFor(deps map[string]*nodeDeps, process, node string) *nodeDeps {
	key := nodeKey(process, node)
	d, ok := deps[key]
	if !ok {
		d = newNodeDeps()
		deps[key] = d
	}
	return d
}

// nodeKey names a node in the merged graph. A per-node deployment labels each
// process after the node it runs, so repeating it would read
// "patrol-1/navigator/navigator".
func nodeKey(process, node string) string {
	if process == node || strings.HasSuffix(process, "/"+node) {
		return process
	}
	return process + "/" + node
}

// label is how a process is named in the merged graph.
func (p *ProcessView) label() string {
	switch {
	case p.Host != "" && p.Name != "":
		return p.Host + "/" + p.Name
	case p.Name != "":
		return p.Name
	case p.Host != "":
		return p.Host
	}
	return p.URL
}

// ServeFleet starts the fleet portal: it polls the given peers on every
// request and serves the merged view. It is a plain HTTP client of the
// per-process dashboards, so it runs anywhere that can reach them — a laptop,
// or one of the robot's own processes.
func ServeFleet(addr string, peers []Peer, timeout time.Duration, opts FleetOptions) (*http.Server, error) {
	page, err := fleetFS.ReadFile("fleet.html")
	if err != nil {
		return nil, err
	}
	if timeout <= 0 {
		timeout = 2 * time.Second
	}

	mux := http.NewServeMux()
	mux.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/" {
			http.NotFound(w, r)
			return
		}
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		w.Write(page)
	})
	mux.HandleFunc("/api/fleet", func(w http.ResponseWriter, r *http.Request) {
		ctx, cancel := context.WithTimeout(r.Context(), timeout+time.Second)
		defer cancel()
		writeJSON(w, FleetWith(ctx, peers, timeout, opts))
	})

	srv := &http.Server{Addr: addr, Handler: mux, ReadHeaderTimeout: 5 * time.Second}
	ln, err := net.Listen("tcp", addr)
	if err != nil {
		return nil, err
	}
	go func() {
		if err := srv.Serve(ln); err != nil && err != http.ErrServerClosed {
			slog.Error("conductor: fleet server", "err", err)
		}
	}()
	return srv, nil
}

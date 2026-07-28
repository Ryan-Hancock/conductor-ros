// Package graph builds an application's topic graph from the scanned model
// and validates it: every subscription has a publisher, message types agree
// across each topic, QoS pairings are compatible, and every declared handler
// method exists — the class of failures ROS 2 surfaces only at runtime (or
// not at all), moved to build time.
package graph

import (
	"fmt"
	"sort"

	"conductor.dev/conductor"
	"conductor.dev/conductor/internal/scan"
)

type Severity string

const (
	Error   Severity = "error"
	Warning Severity = "warning"
)

type Issue struct {
	Severity Severity
	Code     string
	Msg      string
	Pos      string // file:line, empty for app-level issues
}

func (i Issue) String() string {
	if i.Pos != "" {
		return fmt.Sprintf("%s %s: %s (%s)", i.Severity, i.Code, i.Msg, i.Pos)
	}
	return fmt.Sprintf("%s %s: %s", i.Severity, i.Code, i.Msg)
}

type Endpoint struct {
	Node     string // node name, or "(external)"
	External bool
	GoType   string
	RosType  string
	QoS      string
	Pos      string
}

type Topic struct {
	Name string
	Pubs []Endpoint
	Subs []Endpoint
}

// RosType returns the topic's ROS interface name if any endpoint knows it.
func (t *Topic) RosType() string {
	for _, e := range append(append([]Endpoint{}, t.Pubs...), t.Subs...) {
		if e.RosType != "" {
			return e.RosType
		}
	}
	return ""
}

type Graph struct {
	App      *scan.App
	Topics   []*Topic
	Services []*Service
	Actions  []*ActionName
}

// ActionName collects the endpoints of one action name.
type ActionName struct {
	Name    string
	Servers []SvcEndpoint
	Clients []SvcEndpoint
}

// Service collects the endpoints of one service name.
type Service struct {
	Name    string
	Servers []SvcEndpoint
	Clients []SvcEndpoint
}

type SvcEndpoint struct {
	Node     string // node name, or "(external)"
	External bool
	ReqType  string // Go types for internal endpoints; empty for external
	ResType  string
	RosType  string // ROS service type for external endpoints
	Pos      string
}

// Build assembles the topic graph, including declared external endpoints.
func Build(app *scan.App) *Graph {
	topics := map[string]*Topic{}
	get := func(name string) *Topic {
		t, ok := topics[name]
		if !ok {
			t = &Topic{Name: name}
			topics[name] = t
		}
		return t
	}
	for _, n := range app.Nodes {
		for _, p := range n.Pubs {
			if p.Topic == "" {
				continue
			}
			get(p.Topic).Pubs = append(get(p.Topic).Pubs, endpoint(app, n, p))
		}
		for _, s := range n.Subs {
			if s.Topic == "" {
				continue
			}
			get(s.Topic).Subs = append(get(s.Topic).Subs, endpoint(app, n, s))
		}
	}
	services := map[string]*Service{}
	getSvc := func(name string) *Service {
		s, ok := services[name]
		if !ok {
			s = &Service{Name: name}
			services[name] = s
		}
		return s
	}
	for _, n := range app.Nodes {
		for _, s := range n.Services {
			if s.Service == "" {
				continue
			}
			getSvc(s.Service).Servers = append(getSvc(s.Service).Servers, svcEndpoint(n, s))
		}
		for _, c := range n.Clients {
			if c.Service == "" {
				continue
			}
			getSvc(c.Service).Clients = append(getSvc(c.Service).Clients, svcEndpoint(n, c))
		}
	}
	acts := map[string]*ActionName{}
	getAct := func(name string) *ActionName {
		a, ok := acts[name]
		if !ok {
			a = &ActionName{Name: name}
			acts[name] = a
		}
		return a
	}
	for _, n := range app.Nodes {
		for _, a := range n.Actions {
			if a.Action == "" {
				continue
			}
			getAct(a.Action).Servers = append(getAct(a.Action).Servers, actEndpoint(n, a))
		}
		for _, a := range n.ActionClients {
			if a.Action == "" {
				continue
			}
			getAct(a.Action).Clients = append(getAct(a.Action).Clients, actEndpoint(n, a))
		}
	}
	for _, e := range app.Externals {
		switch e.Role {
		case "publisher":
			get(e.Topic).Pubs = append(get(e.Topic).Pubs, Endpoint{Node: "(external)", External: true, RosType: e.Type, QoS: e.QoS})
		case "subscriber":
			get(e.Topic).Subs = append(get(e.Topic).Subs, Endpoint{Node: "(external)", External: true, RosType: e.Type, QoS: e.QoS})
		case "server":
			getSvc(e.Topic).Servers = append(getSvc(e.Topic).Servers, SvcEndpoint{Node: "(external)", External: true, RosType: e.Type})
		case "client":
			getSvc(e.Topic).Clients = append(getSvc(e.Topic).Clients, SvcEndpoint{Node: "(external)", External: true, RosType: e.Type})
		case "action_server":
			getAct(e.Topic).Servers = append(getAct(e.Topic).Servers, SvcEndpoint{Node: "(external)", External: true, RosType: e.Type})
		case "action_client":
			getAct(e.Topic).Clients = append(getAct(e.Topic).Clients, SvcEndpoint{Node: "(external)", External: true, RosType: e.Type})
		}
	}
	g := &Graph{App: app}
	for _, t := range topics {
		g.Topics = append(g.Topics, t)
	}
	sort.Slice(g.Topics, func(i, j int) bool { return g.Topics[i].Name < g.Topics[j].Name })
	for _, s := range services {
		g.Services = append(g.Services, s)
	}
	sort.Slice(g.Services, func(i, j int) bool { return g.Services[i].Name < g.Services[j].Name })
	for _, a := range acts {
		g.Actions = append(g.Actions, a)
	}
	sort.Slice(g.Actions, func(i, j int) bool { return g.Actions[i].Name < g.Actions[j].Name })
	return g
}

func actEndpoint(n *scan.Node, e scan.ActionEndpoint) SvcEndpoint {
	return SvcEndpoint{
		Node:    n.Name,
		ReqType: e.GoalType,
		ResType: e.ResultType,
		Pos:     fmt.Sprintf("%s:%d", e.File, e.Line),
	}
}

func svcEndpoint(n *scan.Node, e scan.ServiceEndpoint) SvcEndpoint {
	return SvcEndpoint{
		Node:    n.Name,
		ReqType: e.ReqType,
		ResType: e.ResType,
		Pos:     fmt.Sprintf("%s:%d", e.File, e.Line),
	}
}

func endpoint(app *scan.App, n *scan.Node, e scan.Endpoint) Endpoint {
	return Endpoint{
		Node:    n.Name,
		GoType:  e.GoType,
		RosType: app.Messages[e.GoType],
		QoS:     e.QoS,
		Pos:     fmt.Sprintf("%s:%d", e.File, e.Line),
	}
}

// Validate builds the graph and returns it with all issues found, errors first.
func Validate(app *scan.App) (*Graph, []Issue) {
	var issues []Issue
	add := func(sev Severity, code, pos, format string, args ...any) {
		issues = append(issues, Issue{Severity: sev, Code: code, Pos: pos, Msg: fmt.Sprintf(format, args...)})
	}

	for _, n := range app.Nodes {
		for _, s := range n.Subs {
			pos := fmt.Sprintf("%s:%d", s.File, s.Line)
			if s.Topic == "" {
				add(Error, "CND001", pos, "node %s: field %s is missing a topic tag", n.Name, s.Field)
			}
			if _, ok := conductor.QoSProfile(s.QoS); !ok {
				add(Error, "CND002", pos, "node %s: unknown qos profile %q on %s", n.Name, s.QoS, s.Field)
			}
			checkHandler(add, n, s.Field, scan.MethodSig{Params: 1}, pos)
			if _, ok := app.Messages[s.GoType]; !ok {
				add(Warning, "CND008", pos, "message type %s has no //ros:type directive; cannot map it to a ROS interface", s.GoType)
			}
		}
		for _, p := range n.Pubs {
			pos := fmt.Sprintf("%s:%d", p.File, p.Line)
			if p.Topic == "" {
				add(Error, "CND001", pos, "node %s: field %s is missing a topic tag", n.Name, p.Field)
			}
			if _, ok := conductor.QoSProfile(p.QoS); !ok {
				add(Error, "CND002", pos, "node %s: unknown qos profile %q on %s", n.Name, p.QoS, p.Field)
			}
			if _, ok := app.Messages[p.GoType]; !ok {
				add(Warning, "CND008", pos, "message type %s has no //ros:type directive; cannot map it to a ROS interface", p.GoType)
			}
		}
		for _, tm := range n.Timers {
			pos := fmt.Sprintf("%s:%d", tm.File, tm.Line)
			if _, err := conductor.ParseRate(tm.Rate); err != nil {
				add(Error, "CND004", pos, "node %s: %v on %s", n.Name, err, tm.Field)
			}
			checkHandler(add, n, tm.Field, scan.MethodSig{}, pos)
		}
		for _, s := range n.Services {
			pos := fmt.Sprintf("%s:%d", s.File, s.Line)
			if s.Service == "" {
				add(Error, "CND001", pos, "node %s: field %s is missing a service tag", n.Name, s.Field)
			}
			checkHandler(add, n, s.Field, scan.MethodSig{Params: 1, Results: 2}, pos)
		}
		for _, c := range n.Clients {
			if c.Service == "" {
				add(Error, "CND001", fmt.Sprintf("%s:%d", c.File, c.Line), "node %s: field %s is missing a service tag", n.Name, c.Field)
			}
		}
		for _, ac := range n.Actions {
			pos := fmt.Sprintf("%s:%d", ac.File, ac.Line)
			if ac.Action == "" {
				add(Error, "CND001", pos, "node %s: field %s is missing an action tag", n.Name, ac.Field)
			}
			checkHandler(add, n, ac.Field, scan.MethodSig{Params: 1, Results: 2}, pos)
		}
	}
	for _, e := range app.Externals {
		switch e.Role {
		case "publisher", "subscriber":
			if _, ok := conductor.QoSProfile(e.QoS); !ok {
				add(Error, "CND002", "", "external for topic %q: unknown qos profile %q", e.Topic, e.QoS)
			}
		case "server", "client", "action_server", "action_client":
			// Services and actions use the default QoS; nothing further to validate.
		default:
			add(Error, "CND005", "", "external for %q: role must be publisher, subscriber, server, client, action_server, or action_client; got %q", e.Topic, e.Role)
		}
	}

	g := Build(app)
	for _, t := range g.Topics {
		if len(t.Pubs) == 0 {
			for _, s := range t.Subs {
				add(Error, "CND010", s.Pos, "topic %q: subscribed by %s but nothing publishes it (declare an external publisher in conductor.json if it comes from outside this app)", t.Name, s.Node)
			}
		}
		if len(t.Subs) == 0 && allInternal(t.Pubs) {
			for _, p := range t.Pubs {
				add(Warning, "CND011", p.Pos, "topic %q: published by %s but nothing subscribes to it", t.Name, p.Node)
			}
		}
		checkTypes(add, t)
		for _, p := range t.Pubs {
			pq, okP := conductor.QoSProfile(p.QoS)
			for _, s := range t.Subs {
				sq, okS := conductor.QoSProfile(s.QoS)
				if !okP || !okS {
					continue
				}
				for _, problem := range qosIncompat(pq, sq) {
					add(Error, "CND013", s.Pos, "topic %q: %s (publisher %s is %q, subscriber %s is %q)",
						t.Name, problem, p.Node, pq.Name, s.Node, sq.Name)
				}
			}
		}
	}

	for _, s := range g.Services {
		if len(s.Servers) == 0 {
			for _, c := range s.Clients {
				add(Error, "CND020", c.Pos, "service %q: called by %s but nothing serves it (declare an external server in conductor.json if it lives outside this app)", s.Name, c.Node)
			}
		}
		if len(s.Servers) > 1 {
			add(Error, "CND022", s.Servers[1].Pos, "service %q has %d servers; a service must have exactly one", s.Name, len(s.Servers))
		}
		if len(s.Clients) == 0 && allInternalSvc(s.Servers) {
			for _, sv := range s.Servers {
				add(Warning, "CND021", sv.Pos, "service %q: served by %s but nothing calls it", s.Name, sv.Node)
			}
		}
		reqTypes, resTypes := map[string]bool{}, map[string]bool{}
		for _, ep := range append(append([]SvcEndpoint{}, s.Servers...), s.Clients...) {
			if !ep.External {
				reqTypes[ep.ReqType] = true
				resTypes[ep.ResType] = true
			}
		}
		if len(reqTypes) > 1 || len(resTypes) > 1 {
			add(Error, "CND023", "", "service %q: endpoints disagree on request/response types: %s -> %s", s.Name, keys(reqTypes), keys(resTypes))
		}
	}

	for _, a := range g.Actions {
		if len(a.Servers) > 1 {
			add(Error, "CND030", a.Servers[1].Pos, "action %q has %d servers; an action must have exactly one", a.Name, len(a.Servers))
		}
		if len(a.Servers) == 0 && len(a.Clients) > 0 {
			add(Error, "CND031", "", "action %q: declared as an external client target but nothing serves it", a.Name)
		}
	}

	sort.SliceStable(issues, func(i, j int) bool {
		return issues[i].Severity == Error && issues[j].Severity == Warning
	})
	return g, issues
}

func allInternalSvc(eps []SvcEndpoint) bool {
	for _, e := range eps {
		if e.External {
			return false
		}
	}
	return true
}

func checkHandler(add func(Severity, string, string, string, ...any), n *scan.Node, field string, want scan.MethodSig, pos string) {
	got, ok := n.Methods["On"+field]
	switch {
	case !ok:
		add(Error, "CND003", pos, "node %s: missing handler method On%s", n.Name, field)
	case got != want:
		add(Error, "CND003", pos, "node %s: handler On%s must take %d parameter(s) and return %d value(s), has %d and %d",
			n.Name, field, want.Params, want.Results, got.Params, got.Results)
	}
}

func checkTypes(add func(Severity, string, string, string, ...any), t *Topic) {
	all := append(append([]Endpoint{}, t.Pubs...), t.Subs...)

	goTypes := map[string]bool{}
	rosTypes := map[string]bool{}
	for _, e := range all {
		if !e.External && e.GoType != "" {
			goTypes[e.GoType] = true
		}
		if e.RosType != "" {
			rosTypes[e.RosType] = true
		}
	}
	if len(goTypes) > 1 {
		add(Error, "CND012", "", "topic %q: endpoints disagree on message type: %s", t.Name, keys(goTypes))
	}
	if len(rosTypes) > 1 {
		add(Error, "CND012", "", "topic %q: endpoints disagree on ROS interface: %s", t.Name, keys(rosTypes))
	}
}

// qosIncompat returns the DDS request-vs-offered violations for a pairing.
func qosIncompat(pub, sub conductor.QoS) []string {
	var probs []string
	if sub.Reliability == conductor.Reliable && pub.Reliability == conductor.BestEffort {
		probs = append(probs, "subscriber requests reliable delivery but publisher offers best-effort")
	}
	if sub.Durability == conductor.TransientLocal && pub.Durability == conductor.Volatile {
		probs = append(probs, "subscriber requests transient-local durability but publisher offers volatile")
	}
	return probs
}

func allInternal(eps []Endpoint) bool {
	for _, e := range eps {
		if e.External {
			return false
		}
	}
	return true
}

func keys(m map[string]bool) string {
	var ks []string
	for k := range m {
		ks = append(ks, k)
	}
	sort.Strings(ks)
	out := ""
	for i, k := range ks {
		if i > 0 {
			out += " vs "
		}
		out += k
	}
	return out
}

// Errors reports whether any issue is an error.
func Errors(issues []Issue) bool {
	for _, i := range issues {
		if i.Severity == Error {
			return true
		}
	}
	return false
}

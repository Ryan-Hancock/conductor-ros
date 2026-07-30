// Package discover reads a live ROS 2 graph and turns it into the
// declarations conductor.json holds.
//
// The externals block is the one place a conductor application takes something
// on trust: it says what exists outside itself — topics, their types, their
// QoS — and the checker believes it completely. That is fine when the outside
// is small and stable, and wrong when it is Nav2: the entries are transcribed
// by hand from someone else's source, and a wrong type or a mismatched QoS
// produces silence, which is exactly the failure the checker exists to
// prevent.
//
// So stop transcribing. rmw_zenoh advertises every node, publisher,
// subscription, service and client as a liveliness token carrying the topic,
// the type, its RIHS01 hash and the QoS profile offered. Ask the network, and
// the externals block is a derivation rather than a claim.
package discover

import (
	"fmt"
	"sort"
	"strings"

	conductor "conductor.dev/conductor"
	"conductor.dev/conductor/transport/rmwzenoh"
)

// Graph is a live ROS graph as discovered, before any interpretation.
type Graph struct {
	Domain     int
	Endpoint   string
	Entities   []rmwzenoh.Entity
	Unreadable []Unreadable
}

// Unreadable is a liveliness token this build could not decode. They are
// reported rather than dropped: the graph belongs to someone else, and a token
// we cannot read is a fact about our parser, not about their system.
type Unreadable struct {
	Token string
	Err   string
}

// Nodes lists the node names on the graph, sorted.
func (g *Graph) Nodes() []string {
	seen := map[string]bool{}
	for _, e := range g.Entities {
		if e.Kind == rmwzenoh.EntityNode {
			seen[e.Node] = true
		}
	}
	return sorted(seen)
}

// Kind is what sort of interface a group of endpoints adds up to.
type Kind string

const (
	KindTopic   Kind = "topic"
	KindService Kind = "service"
	KindAction  Kind = "action"
)

// Interface is one thing on the graph, after the endpoints that make it have
// been rolled up: a topic, a service, or an action (whose five services and
// two topics are one interface to everybody except rmw).
type Interface struct {
	Name string // fully qualified, without the leading slash
	Kind Kind
	Type string // ROS interface name, e.g. nav2_msgs/action/NavigateToPose
	Hash string // RIHS01 hash as advertised; empty for an action

	// Hashes holds every distinct hash seen for this interface. More than one
	// means two peers disagree about the definition of the same type, which
	// is a real fault and not something to average out.
	Hashes []string

	// QoS is the profile to declare: the weakest of the profiles the
	// publishers offer, because that is the one a subscriber has to be
	// compatible with. It is "" when the offer is not one conductor can name.
	QoS    string
	QoSRaw conductor.QoS

	// Offers and Requests are the distinct profiles seen from publishers and
	// from subscribers. Publishers that disagree are a fault worth naming, not
	// something to pick from at random — and picking at random would also make
	// a generated externals block depend on discovery order.
	Offers   []conductor.QoS
	Requests []conductor.QoS

	Publishers  []string // node names
	Subscribers []string
	Servers     []string
	Clients     []string

	// Infrastructure marks the interfaces every ROS node has — parameter and
	// lifecycle services, /rosout, /parameter_events — which are noise in an
	// externals block because conductor provides them itself.
	Infrastructure bool
}

// Providers is who offers this interface: publishers and servers.
func (i *Interface) Providers() []string {
	if i.Kind == KindTopic {
		return i.Publishers
	}
	return i.Servers
}

// Consumers is who uses it: subscribers and clients.
func (i *Interface) Consumers() []string {
	if i.Kind == KindTopic {
		return i.Subscribers
	}
	return i.Clients
}

// actionSuffixes are the endpoints rmw creates for one action, longest first
// so that stripping finds the action's own name.
var actionSuffixes = []string{
	"/_action/send_goal", "/_action/get_result", "/_action/cancel_goal",
	"/_action/feedback", "/_action/status",
}

// infrastructureTypes are the interfaces every ROS node carries. Recognising
// them by type rather than by name is deliberate: names vary with namespaces
// and node names, types do not.
var infrastructureTypes = map[string]bool{
	"rcl_interfaces/msg/ParameterEvent":          true,
	"rcl_interfaces/msg/Log":                     true,
	"rcl_interfaces/srv/DescribeParameters":      true,
	"rcl_interfaces/srv/GetParameterTypes":       true,
	"rcl_interfaces/srv/GetParameters":           true,
	"rcl_interfaces/srv/ListParameters":          true,
	"rcl_interfaces/srv/SetParameters":           true,
	"rcl_interfaces/srv/SetParametersAtomically": true,
	"lifecycle_msgs/msg/TransitionEvent":         true,
	"lifecycle_msgs/srv/ChangeState":             true,
	"lifecycle_msgs/srv/GetState":                true,
	"lifecycle_msgs/srv/GetAvailableStates":      true,
	"lifecycle_msgs/srv/GetAvailableTransitions": true,
	"lifecycle_msgs/srv/GetTransitionGraph":      true,
	"tf2_msgs/msg/TFMessage":                     true,
	// Every node has carried a type-description service since Jazzy.
	"type_description_interfaces/srv/GetTypeDescription": true,
}

// Interfaces rolls the graph's endpoints up into interfaces, sorted by name.
func (g *Graph) Interfaces() []Interface {
	byName := map[string]*Interface{}
	get := func(name string, kind Kind) *Interface {
		key := string(kind) + " " + name
		i, ok := byName[key]
		if !ok {
			i = &Interface{Name: name, Kind: kind}
			byName[key] = i
		}
		return i
	}

	for _, e := range g.Entities {
		if e.Kind == rmwzenoh.EntityNode {
			continue
		}
		name := strings.TrimPrefix(e.Topic, "/")
		if action, ok := splitAction(name); ok {
			// One action, assembled from whichever of its seven endpoints the
			// peer advertised.
			i := get(action, KindAction)
			if i.Type == "" {
				i.Type = actionType(e.Type)
			}
			switch e.Kind {
			case rmwzenoh.EntityService:
				i.Servers = add(i.Servers, e.Node)
			case rmwzenoh.EntityClient:
				i.Clients = add(i.Clients, e.Node)
			case rmwzenoh.EntityPublisher:
				// feedback and status are published by the server.
				i.Servers = add(i.Servers, e.Node)
			case rmwzenoh.EntitySubscription:
				i.Clients = add(i.Clients, e.Node)
			}
			continue
		}

		switch e.Kind {
		case rmwzenoh.EntityPublisher, rmwzenoh.EntitySubscription:
			i := get(name, KindTopic)
			i.record(e)
			if e.Kind == rmwzenoh.EntityPublisher {
				i.Publishers = add(i.Publishers, e.Node)
			} else {
				i.Subscribers = add(i.Subscribers, e.Node)
			}
		case rmwzenoh.EntityService, rmwzenoh.EntityClient:
			i := get(name, KindService)
			i.record(e)
			// A service's wire type is its request message; the interface is
			// the service it belongs to.
			i.Type = serviceType(e.Type)
			if e.Kind == rmwzenoh.EntityService {
				i.Servers = add(i.Servers, e.Node)
			} else {
				i.Clients = add(i.Clients, e.Node)
			}
		}
	}

	out := make([]Interface, 0, len(byName))
	for _, i := range byName {
		i.Infrastructure = infrastructureTypes[i.Type]
		i.settleQoS()
		sort.Strings(i.Publishers)
		sort.Strings(i.Subscribers)
		sort.Strings(i.Servers)
		sort.Strings(i.Clients)
		sort.Strings(i.Hashes)
		out = append(out, *i)
	}
	sort.Slice(out, func(a, b int) bool {
		if out[a].Name != out[b].Name {
			return out[a].Name < out[b].Name
		}
		return out[a].Kind < out[b].Kind
	})
	return out
}

// record takes the type, hash and QoS an endpoint advertises.
func (i *Interface) record(e rmwzenoh.Entity) {
	if i.Type == "" {
		i.Type = e.Type
	}
	if i.Hash == "" {
		i.Hash = e.Hash
	}
	i.Hashes = add(i.Hashes, e.Hash)
	if i.Kind != KindTopic {
		return
	}
	switch e.Kind {
	case rmwzenoh.EntityPublisher:
		i.Offers = addQoS(i.Offers, e.QoS)
	case rmwzenoh.EntitySubscription:
		i.Requests = addQoS(i.Requests, e.QoS)
	}
}

// settleQoS picks the profile to declare once every endpoint has been seen:
// the weakest offer, since a subscriber must be compatible with the least
// generous publisher to hear all of them. With nobody publishing yet, a
// subscriber's own request is the best guess available.
func (i *Interface) settleQoS() {
	pool := i.Offers
	if len(pool) == 0 {
		pool = i.Requests
	}
	if len(pool) == 0 {
		return
	}
	weakest := pool[0]
	for _, q := range pool[1:] {
		if weaker(q, weakest) {
			weakest = q
		}
	}
	i.QoS, i.QoSRaw = weakest.Name, weakest
}

// weaker reports whether a offers less than b: best-effort is weaker than
// reliable, and volatile weaker than transient-local.
func weaker(a, b conductor.QoS) bool {
	if a.Reliability != b.Reliability {
		return a.Reliability == conductor.BestEffort
	}
	if a.Durability != b.Durability {
		return a.Durability == conductor.Volatile
	}
	return a.Depth < b.Depth
}

// Disagreement describes publishers that do not offer the same profile, which
// makes "the" QoS of a topic a question rather than a fact.
func (i *Interface) Disagreement() (profiles []string, ok bool) {
	if len(i.Offers) < 2 {
		return nil, false
	}
	for _, q := range i.Offers {
		profiles = append(profiles, describeQoS(q))
	}
	sort.Strings(profiles)
	return profiles, true
}

func describeQoS(q conductor.QoS) string {
	if q.Name != "" {
		return q.Name
	}
	return fmt.Sprintf("%s/%s depth %d", q.Reliability, q.Durability, q.Depth)
}

// addQoS collects distinct profiles; two publishers with identical settings
// are one offer.
func addQoS(list []conductor.QoS, q conductor.QoS) []conductor.QoS {
	for _, existing := range list {
		if existing.Reliability == q.Reliability && existing.Durability == q.Durability &&
			existing.Depth == q.Depth {
			return list
		}
	}
	return append(list, q)
}

// splitAction reports whether a name is one of an action's endpoints, and if
// so which action it belongs to.
func splitAction(name string) (action string, ok bool) {
	for _, s := range actionSuffixes {
		if strings.HasSuffix(name, s) {
			return strings.TrimSuffix(name, s), true
		}
	}
	return "", false
}

// actionType recovers the action's interface name from one of its derived
// types: nav2_msgs/action/NavigateToPose_SendGoal_Request is
// nav2_msgs/action/NavigateToPose.
func actionType(typeName string) string {
	base := typeName
	for _, s := range []string{
		"_SendGoal_Request", "_SendGoal_Response", "_SendGoal",
		"_GetResult_Request", "_GetResult_Response", "_GetResult",
		"_FeedbackMessage", "_Feedback", "_Goal", "_Result",
	} {
		if strings.HasSuffix(base, s) {
			return strings.TrimSuffix(base, s)
		}
	}
	// status and cancel_goal use action_msgs types, which say nothing about
	// which action they belong to; another endpoint will supply the name.
	if strings.HasPrefix(base, "action_msgs/") {
		return ""
	}
	return base
}

// serviceType recovers a service's interface name from its request or response
// type: std_srvs/srv/SetBool_Request is std_srvs/srv/SetBool.
func serviceType(typeName string) string {
	for _, s := range []string{"_Request", "_Response", "_Event"} {
		if strings.HasSuffix(typeName, s) {
			return strings.TrimSuffix(typeName, s)
		}
	}
	return typeName
}

func add(list []string, v string) []string {
	if v == "" {
		return list
	}
	for _, existing := range list {
		if existing == v {
			return list
		}
	}
	return append(list, v)
}

func sorted(set map[string]bool) []string {
	out := make([]string, 0, len(set))
	for k := range set {
		out = append(out, k)
	}
	sort.Strings(out)
	return out
}

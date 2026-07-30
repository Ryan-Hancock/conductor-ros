package conductor

import (
	"reflect"
	"sort"
)

// The runtime already knows every endpoint it wired — that is what binding
// is — so it can describe itself. This inventory is what the dashboard reads,
// and unlike `conductor check`'s static view it is the live one: what this
// process actually has open, in this environment, right now.

// EndpointKind names a kind of declaration on a node.
type EndpointKind string

const (
	EndpointSub          EndpointKind = "subscription"
	EndpointPub          EndpointKind = "publisher"
	EndpointTimer        EndpointKind = "timer"
	EndpointService      EndpointKind = "service"
	EndpointClient       EndpointKind = "client"
	EndpointAction       EndpointKind = "action"
	EndpointActionClient EndpointKind = "action client"
	EndpointMission      EndpointKind = "mission"
	EndpointTF           EndpointKind = "transforms"
	EndpointLifecycle    EndpointKind = "lifecycle client"
)

// Endpoint describes one wired declaration.
type Endpoint struct {
	Node   string       `json:"node"`
	Kind   EndpointKind `json:"kind"`
	Field  string       `json:"field"`
	Name   string       `json:"name"`           // topic, service or action name
	Type   string       `json:"type,omitempty"` // ROS interface name when known
	QoS    string       `json:"qos,omitempty"`
	Rate   string       `json:"rate,omitempty"`  // timers
	Frame  string       `json:"frame,omitempty"` // frame tag, when declared
	count  *countFunc   // live message/call count, nil if not counted
	Counts uint64       `json:"counts"`
}

type countFunc func() uint64

func (rt *runtimeState) recordEndpoint(e Endpoint) {
	rt.endpoints = append(rt.endpoints, e)
}

// endpointsSnapshot returns the inventory with live counters read.
func (rt *runtimeState) endpointsSnapshot() []Endpoint {
	out := make([]Endpoint, len(rt.endpoints))
	copy(out, rt.endpoints)
	for i := range out {
		if out[i].count != nil {
			out[i].Counts = (*out[i].count)()
		}
	}
	sort.SliceStable(out, func(i, j int) bool {
		if out[i].Node != out[j].Node {
			return out[i].Node < out[j].Node
		}
		return out[i].Name < out[j].Name
	})
	return out
}

// rosTypeName reports the ROS interface name registered for a Go message
// type, falling back to the Go type so the view is never blank.
func rosTypeName(t reflect.Type) string {
	if info, ok := MessageInfoOf(t); ok {
		return info.Name
	}
	return t.String()
}

// rosServiceName reports the ROS interface name for a service pair.
func rosServiceName(req, res reflect.Type) string {
	if info, ok := ServiceInfoOf(req, res); ok {
		return info.Name
	}
	return req.String() + " -> " + res.String()
}

// countOf adapts a counter's Load method to the inventory's reader.
func countOf(f func() uint64) *countFunc {
	c := countFunc(f)
	return &c
}

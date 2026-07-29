package conductor

import (
	"errors"
	"log/slog"
	"reflect"
	"sync/atomic"
)

// Pub declares a publisher. Publish may be called from any of the node's
// handlers (or, with external synchronization, from anywhere).
//
// Tags: topic (required), qos (reliable | sensor | transient; default reliable).
type Pub[T any] struct {
	topic   string
	publish func(any, Metadata) error
	node    *nodeRuntime
	sent    atomic.Uint64
}

// Topic returns the wired topic name (empty before Run).
func (p *Pub[T]) Topic() string { return p.topic }

// Publish sends msg to every subscriber of the topic. Transport errors are
// logged, not returned: publishing is fire-and-forget, like ROS. Messages
// published while the node is not active are dropped, per the managed-node
// design.
//
// When called from inside a callback, the message carries that callback's
// trace context, so the work it triggers downstream joins the same trace.
func (p *Pub[T]) Publish(msg T) {
	if p.publish == nil {
		panic("conductor: Publish called on a publisher that was not wired by Run")
	}
	if p.node != nil && !p.node.active() {
		return
	}
	var md Metadata
	if p.node != nil {
		md.Trace = p.node.currentTrace
	}
	if err := p.publish(msg, md); err != nil {
		slog.Error("conductor: publish failed", "topic", p.topic, "err", err)
		counter("conductor_publish_errors_total", "node", p.node.name, "topic", p.topic).Add(1)
		return
	}
	p.sent.Add(1)
	counter("conductor_messages_published_total", "node", p.node.name, "topic", p.topic).Add(1)
}

func (p *Pub[T]) bind(rt *runtimeState, nr *nodeRuntime, field reflect.StructField, ownerPtr reflect.Value) error {
	topic := field.Tag.Get("topic")
	if topic == "" {
		return errors.New(`missing topic tag (e.g. topic:"cmd_vel")`)
	}
	q, err := qosFromTag(field.Tag.Get("qos"))
	if err != nil {
		return err
	}
	spec := TopicSpec{Topic: topic, QoS: q, Type: reflect.TypeFor[T](), Node: nr.name}
	publish, err := rt.transport.Publisher(spec)
	if err != nil {
		return err
	}
	p.topic = topic
	p.publish = publish
	p.node = nr
	rt.recordProvides(nr.name, topic)
	rt.recordEndpoint(Endpoint{Node: nr.name, Kind: EndpointPub, Field: field.Name, Name: topic,
		Type: rosTypeName(reflect.TypeFor[T]()), QoS: q.Name, count: countOf(p.sent.Load)})
	return nil
}

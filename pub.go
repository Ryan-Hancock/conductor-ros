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
	publish func(any) error
	sent    atomic.Uint64
}

// Topic returns the wired topic name (empty before Run).
func (p *Pub[T]) Topic() string { return p.topic }

// Publish sends msg to every subscriber of the topic. Transport errors are
// logged, not returned: publishing is fire-and-forget, like ROS.
func (p *Pub[T]) Publish(msg T) {
	if p.publish == nil {
		panic("conductor: Publish called on a publisher that was not wired by Run")
	}
	if err := p.publish(msg); err != nil {
		slog.Error("conductor: publish failed", "topic", p.topic, "err", err)
		return
	}
	p.sent.Add(1)
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
	return nil
}

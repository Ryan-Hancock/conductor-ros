package conductor

import (
	"errors"
	"fmt"
	"reflect"
)

// Sub declares a subscription. The owning node must define a handler method
// named On<FieldName> with signature func(T); it is invoked for every message
// on the node's own executor goroutine.
//
// Tags: topic (required), qos (reliable | sensor | transient; default reliable).
type Sub[T any] struct {
	topic string
}

// Topic returns the wired topic name (empty before Run).
func (s *Sub[T]) Topic() string { return s.topic }

func (s *Sub[T]) bind(rt *runtimeState, nr *nodeRuntime, field reflect.StructField, ownerPtr reflect.Value) error {
	topic := field.Tag.Get("topic")
	if topic == "" {
		return errors.New(`missing topic tag (e.g. topic:"cmd_vel")`)
	}
	q, err := qosFromTag(field.Tag.Get("qos"))
	if err != nil {
		return err
	}
	m := ownerPtr.MethodByName("On" + field.Name)
	if !m.IsValid() {
		return fmt.Errorf("missing handler method On%s", field.Name)
	}
	h, ok := m.Interface().(func(T))
	if !ok {
		var zero T
		return fmt.Errorf("On%s must have signature func(%T)", field.Name, zero)
	}
	s.topic = topic
	spec := TopicSpec{Topic: topic, QoS: q, Type: reflect.TypeFor[T](), Node: nr.name}
	return rt.transport.Subscribe(spec, func(msg any) {
		nr.enqueue(func() { h(msg.(T)) })
	})
}

package conductor

import (
	"errors"
	"fmt"
	"reflect"
	"sync/atomic"
)

// Svc declares a service server. The owning node must define a handler
// method named On<FieldName> with signature func(Req) (Res, error); it runs
// on the node's executor goroutine like every other callback, so node state
// is safe to touch. Returning an error rejects the request (networked
// transports report it to the caller as a service error).
//
// Tags: service (required) — the service name.
type Svc[Req, Res any] struct {
	service string
	served  atomic.Uint64
}

// Service returns the wired service name (empty before Run).
func (s *Svc[Req, Res]) Service() string { return s.service }

func (s *Svc[Req, Res]) bind(rt *runtimeState, nr *nodeRuntime, field reflect.StructField, ownerPtr reflect.Value) error {
	service := field.Tag.Get("service")
	if service == "" {
		return errors.New(`missing service tag (e.g. service:"engage_estop")`)
	}
	m := ownerPtr.MethodByName("On" + field.Name)
	if !m.IsValid() {
		return fmt.Errorf("missing handler method On%s", field.Name)
	}
	h, ok := m.Interface().(func(Req) (Res, error))
	if !ok {
		var req Req
		var res Res
		return fmt.Errorf("On%s must have signature func(%T) (%T, error)", field.Name, req, res)
	}
	s.service = service
	spec := ServiceSpec{
		Service: service,
		ReqType: reflect.TypeFor[Req](),
		ResType: reflect.TypeFor[Res](),
		Node:    nr.name,
	}
	rt.recordProvides(nr.name, service)
	rt.recordEndpoint(Endpoint{Node: nr.name, Kind: EndpointService, Field: field.Name, Name: service,
		Type: rosServiceName(reflect.TypeFor[Req](), reflect.TypeFor[Res]()), count: countOf(s.served.Load)})
	handle := func(req any) (any, error) {
		type result struct {
			res Res
			err error
		}
		ch := make(chan result, 1)
		if !nr.enqueue(func() {
			nr.runInstrumented(SpanService, service, TraceContext{}, func() {
				res, err := h(req.(Req))
				ch <- result{res, err}
			})
		}) {
			return nil, fmt.Errorf("service %q: node %s is not accepting work (shutting down or mailbox full)", service, nr.name)
		}
		select {
		case r := <-ch:
			outcome := "ok"
			if r.err != nil {
				outcome = "error"
			}
			counter("conductor_service_requests_total", "node", nr.name, "service", service, "outcome", outcome).Add(1)
			s.served.Add(1)
			if r.err != nil {
				return nil, r.err
			}
			return r.res, nil
		case <-nr.quit:
			return nil, fmt.Errorf("service %q: node %s shut down", service, nr.name)
		}
	}
	return rt.transport.Serve(spec, handle)
}

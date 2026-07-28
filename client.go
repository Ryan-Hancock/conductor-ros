package conductor

import (
	"errors"
	"fmt"
	"reflect"
	"time"
)

// Client declares a service client. Call blocks until the response arrives
// or the timeout elapses.
//
// Do not call a service served by the same node from one of that node's own
// handlers: the executor is busy running the handler, so the call can only
// time out.
//
// Tags: service (required), timeout (time.ParseDuration; default 5s — must
// stay below the transport's own query timeout, 10s on zenoh).
type Client[Req, Res any] struct {
	service string
	timeout time.Duration
	call    func(any, time.Duration) (any, error)
}

// Service returns the wired service name (empty before Run).
func (c *Client[Req, Res]) Service() string { return c.service }

// Call sends req and waits for the response.
func (c *Client[Req, Res]) Call(req Req) (Res, error) {
	var zero Res
	if c.call == nil {
		panic("conductor: Call on a client that was not wired by Run")
	}
	res, err := c.call(req, c.timeout)
	if err != nil {
		return zero, err
	}
	return res.(Res), nil
}

func (c *Client[Req, Res]) bind(rt *runtimeState, nr *nodeRuntime, field reflect.StructField, ownerPtr reflect.Value) error {
	service := field.Tag.Get("service")
	if service == "" {
		return errors.New(`missing service tag (e.g. service:"engage_estop")`)
	}
	timeout := 5 * time.Second
	if tag := field.Tag.Get("timeout"); tag != "" {
		d, err := time.ParseDuration(tag)
		if err != nil || d <= 0 {
			return fmt.Errorf("invalid timeout %q", tag)
		}
		timeout = d
	}
	spec := ServiceSpec{
		Service: service,
		ReqType: reflect.TypeFor[Req](),
		ResType: reflect.TypeFor[Res](),
		Node:    nr.name,
	}
	call, err := rt.transport.ServiceClient(spec)
	if err != nil {
		return err
	}
	c.service = service
	c.timeout = timeout
	c.call = call
	return nil
}

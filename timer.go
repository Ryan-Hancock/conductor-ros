package conductor

import (
	"errors"
	"fmt"
	"reflect"
	"strconv"
	"strings"
	"time"
)

// Timer declares a periodic callback. The owning node must define a handler
// method named On<FieldName> with signature func(); it runs on the node's
// executor goroutine like any other callback.
//
// Tags: rate (required), either a frequency ("10hz", "2.5hz") or a
// time.ParseDuration period ("250ms").
type Timer struct {
	period time.Duration
}

// Period returns the wired period (zero before Run).
func (t *Timer) Period() time.Duration { return t.period }

func (t *Timer) bind(rt *runtimeState, nr *nodeRuntime, field reflect.StructField, ownerPtr reflect.Value) error {
	rate := field.Tag.Get("rate")
	if rate == "" {
		return errors.New(`missing rate tag (e.g. rate:"10hz")`)
	}
	period, err := ParseRate(rate)
	if err != nil {
		return err
	}
	m := ownerPtr.MethodByName("On" + field.Name)
	if !m.IsValid() {
		return fmt.Errorf("missing handler method On%s", field.Name)
	}
	h, ok := m.Interface().(func())
	if !ok {
		return fmt.Errorf("On%s must have signature func()", field.Name)
	}
	t.period = period
	rt.timers = append(rt.timers, &timerHandle{period: period, node: nr, fire: h})
	return nil
}

// ParseRate parses a rate tag value: a frequency with an "hz" suffix or any
// time.ParseDuration string. The result is the tick period.
func ParseRate(s string) (time.Duration, error) {
	if strings.HasSuffix(s, "hz") {
		f, err := strconv.ParseFloat(strings.TrimSuffix(s, "hz"), 64)
		if err != nil || f <= 0 {
			return 0, fmt.Errorf("invalid rate %q", s)
		}
		return time.Duration(float64(time.Second) / f), nil
	}
	d, err := time.ParseDuration(s)
	if err != nil || d <= 0 {
		return 0, fmt.Errorf("invalid rate %q", s)
	}
	return d, nil
}

type timerHandle struct {
	period time.Duration
	node   *nodeRuntime
	fire   func()
	stop   chan struct{}
	done   chan struct{}
}

func (th *timerHandle) start() {
	th.stop = make(chan struct{})
	th.done = make(chan struct{})
	go func() {
		defer close(th.done)
		t := time.NewTicker(th.period)
		defer t.Stop()
		for {
			select {
			case <-th.stop:
				return
			case <-t.C:
				th.node.enqueue(th.fire)
			}
		}
	}()
}

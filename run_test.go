package conductor

import (
	"errors"
	"strings"
	"sync/atomic"
	"testing"
	"time"
)

type ping struct {
	V int
}

type Pinger struct {
	Tick Timer     `rate:"200hz"`
	Out  Pub[ping] `topic:"pings"`
}

func (p *Pinger) OnTick() { p.Out.Publish(ping{V: 1}) }

type Counter struct {
	In Sub[ping] `topic:"pings"`

	n atomic.Int64
}

func (c *Counter) OnIn(m ping) { c.n.Add(int64(m.V)) }

func TestPubSubDelivery(t *testing.T) {
	c := &Counter{}
	a, err := newApp("inproc", TransportOptions{}, "", &Pinger{}, c)
	if err != nil {
		t.Fatal(err)
	}
	time.Sleep(100 * time.Millisecond)
	a.stop()
	if c.n.Load() == 0 {
		t.Fatal("no messages delivered")
	}
}

func TestNodeFilter(t *testing.T) {
	a, err := newApp("inproc", TransportOptions{}, "counter", &Pinger{}, &Counter{})
	if err != nil {
		t.Fatal(err)
	}
	defer a.stop()
	if len(a.rt.nodes) != 1 || a.rt.nodes[0].name != "counter" {
		t.Fatalf("expected only counter to run, got %d node(s)", len(a.rt.nodes))
	}
}

type Tuned struct {
	MaxSpeed Param[float64] `name:"max_speed" default:"1.5"`
	Label    Param[string]  `default:"idle"`
	Period   Param[time.Duration] `default:"250ms"`
}

func TestParamDefaults(t *testing.T) {
	n := &Tuned{}
	a, err := newApp("inproc", TransportOptions{}, "", n)
	if err != nil {
		t.Fatal(err)
	}
	defer a.stop()
	if got := n.MaxSpeed.Get(); got != 1.5 {
		t.Errorf("MaxSpeed = %v, want 1.5", got)
	}
	if got := n.Label.Get(); got != "idle" {
		t.Errorf("Label = %q, want idle", got)
	}
	if got := n.Label.Name(); got != "label" {
		t.Errorf("Label name = %q, want label", got)
	}
	if got := n.Period.Get(); got != 250*time.Millisecond {
		t.Errorf("Period = %v, want 250ms", got)
	}
}

type NoHandler struct {
	In Sub[ping] `topic:"pings"`
}

func TestMissingHandlerFailsWiring(t *testing.T) {
	_, err := newApp("inproc", TransportOptions{}, "", &NoHandler{})
	if err == nil || !strings.Contains(err.Error(), "missing handler method OnIn") {
		t.Fatalf("expected missing-handler error, got %v", err)
	}
}

type BadQoS struct {
	Out Pub[ping] `topic:"pings" qos:"bogus"`
}

func TestUnknownQoSFailsWiring(t *testing.T) {
	_, err := newApp("inproc", TransportOptions{}, "", &BadQoS{})
	if err == nil || !strings.Contains(err.Error(), "unknown qos profile") {
		t.Fatalf("expected qos error, got %v", err)
	}
}

type EchoServer struct {
	Echo Svc[ping, ping] `service:"echo"`
}

func (e *EchoServer) OnEcho(p ping) (ping, error) {
	if p.V < 0 {
		return ping{}, errors.New("negative ping")
	}
	return ping{V: p.V + 1}, nil
}

type EchoCaller struct {
	Echo Client[ping, ping] `service:"echo" timeout:"1s"`
}

func TestServiceRoundTrip(t *testing.T) {
	caller := &EchoCaller{}
	a, err := newApp("inproc", TransportOptions{}, "", &EchoServer{}, caller)
	if err != nil {
		t.Fatal(err)
	}
	defer a.stop()

	res, err := caller.Echo.Call(ping{V: 41})
	if err != nil {
		t.Fatal(err)
	}
	if res.V != 42 {
		t.Fatalf("got %d, want 42", res.V)
	}

	if _, err := caller.Echo.Call(ping{V: -1}); err == nil || !strings.Contains(err.Error(), "negative ping") {
		t.Fatalf("expected handler error, got %v", err)
	}
}

func TestServiceNoServer(t *testing.T) {
	caller := &EchoCaller{}
	a, err := newApp("inproc", TransportOptions{}, "", caller)
	if err != nil {
		t.Fatal(err)
	}
	defer a.stop()
	if _, err := caller.Echo.Call(ping{}); err == nil || !strings.Contains(err.Error(), "no in-process server") {
		t.Fatalf("expected no-server error, got %v", err)
	}
}

type BadEcho struct {
	Echo Svc[ping, ping] `service:"echo"`
}

func (b *BadEcho) OnEcho(p ping) ping { return p } // wrong signature: no error result

func TestServiceBadHandlerSignature(t *testing.T) {
	_, err := newApp("inproc", TransportOptions{}, "", &BadEcho{})
	if err == nil || !strings.Contains(err.Error(), "must have signature") {
		t.Fatalf("expected signature error, got %v", err)
	}
}

func TestParseRate(t *testing.T) {
	cases := map[string]time.Duration{
		"10hz":  100 * time.Millisecond,
		"2.5hz": 400 * time.Millisecond,
		"250ms": 250 * time.Millisecond,
	}
	for in, want := range cases {
		got, err := ParseRate(in)
		if err != nil || got != want {
			t.Errorf("ParseRate(%q) = %v, %v; want %v", in, got, err, want)
		}
	}
	for _, bad := range []string{"", "0hz", "-5hz", "fast"} {
		if _, err := ParseRate(bad); err == nil {
			t.Errorf("ParseRate(%q) should fail", bad)
		}
	}
}

func TestSnakeCase(t *testing.T) {
	cases := map[string]string{
		"Navigator":     "navigator",
		"SafetyMonitor": "safety_monitor",
		"Localizer":     "localizer",
	}
	for in, want := range cases {
		if got := snakeCase(in); got != want {
			t.Errorf("snakeCase(%q) = %q, want %q", in, got, want)
		}
	}
}

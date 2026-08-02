package conductor

import (
	"reflect"
	"strings"
	"sync/atomic"
	"testing"
	"time"
)

// Simulated time is time the simulator publishes. Nothing here advances on its
// own, which is the property that makes a simulation reproducible: a world
// paused at a breakpoint pauses the robot with it.

var simEpoch = time.Unix(1_000, 0).UTC()

func TestSimClockFiresWhenTimePasses(t *testing.T) {
	c := newSimClock()
	if c.Started() {
		t.Error("a simulated clock has a time before the simulator gave it one")
	}

	after := c.After(2 * time.Second)
	select {
	case <-after:
		t.Fatal("a waiter fired without the clock moving")
	default:
	}

	c.Set(simEpoch)
	select {
	case <-after:
		t.Fatal("a waiter fired before its deadline")
	default:
	}
	if !c.Started() || !c.Now().Equal(simEpoch) {
		t.Fatalf("now = %v, started = %v", c.Now(), c.Started())
	}

	c.Set(simEpoch.Add(2 * time.Second))
	select {
	case at := <-after:
		if !at.Equal(simEpoch.Add(2 * time.Second)) {
			t.Errorf("fired at %v, want the simulated deadline", at)
		}
	default:
		t.Fatal("the waiter did not fire when its deadline passed")
	}
}

// A ticker keeps its period in simulated seconds, and a jump in simulated time
// is a jump rather than a burst of missed ticks.
func TestSimClockTickerFollowsSimulatedTime(t *testing.T) {
	c := newSimClock()
	c.Set(simEpoch)
	ticks, stop := c.Ticker(time.Second)
	defer stop()

	c.Set(simEpoch.Add(999 * time.Millisecond))
	select {
	case <-ticks:
		t.Fatal("ticked early")
	default:
	}

	c.Set(simEpoch.Add(time.Second))
	select {
	case <-ticks:
	default:
		t.Fatal("did not tick when the period elapsed in simulated time")
	}

	// Ten seconds pass at once: one tick, not ten.
	c.Set(simEpoch.Add(11 * time.Second))
	got := 0
	for {
		select {
		case <-ticks:
			got++
			continue
		default:
		}
		break
	}
	if got != 1 {
		t.Errorf("a 10s jump produced %d ticks, want one", got)
	}

	stop()
	c.Set(simEpoch.Add(30 * time.Second))
	select {
	case <-ticks:
		t.Error("a stopped ticker fired")
	default:
	}
	if c.pending() != 0 {
		t.Errorf("%d waiters left after stopping", c.pending())
	}
}

// A simulation reset moves time backwards. Waiters must not be left holding
// deadlines an hour in a future that will not arrive.
func TestSimClockHandlesAReset(t *testing.T) {
	c := newSimClock()
	c.Set(simEpoch.Add(time.Hour))
	after := c.After(time.Second)

	c.Set(simEpoch) // the world restarted
	select {
	case <-after:
	default:
		t.Fatal("a waiter survived a reset with an unreachable deadline")
	}
}

// clockedNode counts ticks and records the stamp its publisher put on the wire.
type clockedNode struct {
	Beat  Timer                `rate:"1hz"`
	Pulse Pub[stampedPulseMsg] `topic:"pulse" qos:"reliable" frame:"base_link"`

	ticks atomic.Int64
}

type stampedPulseMsg struct {
	Header pulseHeader
}

type pulseHeader struct {
	Stamp   time.Time
	FrameID string
}

func (c *clockedNode) OnBeat() {
	c.ticks.Add(1)
	c.Pulse.Publish(stampedPulseMsg{})
}

// The whole point, end to end: under simulated time a node's timers follow the
// simulator, and what it publishes is stamped with the simulator's clock rather
// than the machine's.
func TestNodeRunsOnSimulatedTime(t *testing.T) {
	sim := newSimClock()
	node := &clockedNode{}
	ta, err := NewTestApp(TestOptions{Clock: sim, RealTimers: true}, node)
	if err != nil {
		t.Fatal(err)
	}
	defer ta.Close()

	var stamps []time.Time
	q, _ := QoSProfile("reliable")
	err = ta.a.rt.transport.Subscribe(TopicSpec{
		Topic: "pulse", QoS: q, Type: reflect.TypeFor[stampedPulseMsg](), Node: "watcher",
	}, func(m any, _ Metadata) { stamps = append(stamps, m.(stampedPulseMsg).Header.Stamp) })
	if err != nil {
		t.Fatal(err)
	}

	// Real time passing does nothing at all: the simulator has not spoken.
	time.Sleep(30 * time.Millisecond)
	ta.Settle()
	if got := node.ticks.Load(); got != 0 {
		t.Fatalf("%d ticks before the simulator published a time", got)
	}

	sim.Set(simEpoch)
	sim.Set(simEpoch.Add(time.Second))
	waitFor(t, func() bool { return node.ticks.Load() >= 1 }, "the first simulated tick")
	ta.Settle()

	if len(stamps) == 0 {
		t.Fatal("nothing was published")
	}
	if !stamps[0].Equal(simEpoch.Add(time.Second)) {
		t.Errorf("stamp = %v, want the simulated time %v", stamps[0], simEpoch.Add(time.Second))
	}
}

// use_sim_time is what a tool asks to find out which clock a node is on, and
// the answer has to be true. Changing it on a running process is refused rather
// than ignored.
func TestUseSimTimeParameterIsTruthful(t *testing.T) {
	for _, sim := range []bool{false, true} {
		opts := TestOptions{}
		if sim {
			opts.Clock, opts.SimTime = newSimClock(), true
		}
		ta, err := NewTestApp(opts, &clockedNode{})
		if err != nil {
			t.Fatal(err)
		}
		got, err := ta.Param("clocked_node", useSimTimeParam)
		if err != nil {
			t.Fatal(err)
		}
		if got != sim {
			t.Errorf("use_sim_time = %v, want %v", got, sim)
		}
		err = ta.SetParam("clocked_node", useSimTimeParam, map[bool]string{true: "false", false: "true"}[sim])
		if err == nil {
			t.Error("use_sim_time was changed on a running process")
		} else if !strings.Contains(err.Error(), "-use-sim-time") {
			t.Errorf("refusal does not say how to change it: %v", err)
		}
		ta.Close()
	}
}

func waitFor(t *testing.T, done func() bool, what string) {
	t.Helper()
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		if done() {
			return
		}
		time.Sleep(time.Millisecond)
	}
	t.Fatalf("timed out waiting for %s", what)
}

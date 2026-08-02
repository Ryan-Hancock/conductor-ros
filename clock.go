package conductor

import (
	"fmt"
	"reflect"
	"sync"
	"time"
)

// A robot in simulation lives in a different time from the machine running it.
// Gazebo publishes the world's time on /clock and expects every node to use it;
// a node that reads the system clock instead computes velocities over the wrong
// interval, times out early or late, and holds a watchdog against a clock the
// world is not using. ROS 2 answers this with a `use_sim_time` parameter that
// every node is expected to honour, and honouring it is not optional for
// interop: other tools set it on our nodes and assume it took effect.
//
// The runtime therefore reads time from a Clock rather than from the package
// time. There are two implementations — the wall clock, and one driven by
// /clock — and the choice is made once, at startup.
//
// What stays on the wall clock, deliberately: spans, metrics and the dashboard.
// Those measure how long something really took, and an operator asking why the
// robot was slow does not want the answer in simulated seconds.

// Clock is where the runtime reads time. Implementations must be safe for
// concurrent use.
type Clock interface {
	// Now is the current time, in whatever time base this clock keeps.
	Now() time.Time
	// After fires once, after d has elapsed on this clock.
	After(d time.Duration) <-chan time.Time
	// Ticker fires every period on this clock until stopped.
	Ticker(period time.Duration) (<-chan time.Time, func())
	// Started reports whether the clock has a time yet. A simulated clock has
	// none until the simulator publishes one, and a node must not act on a
	// time it has not been given.
	Started() bool
}

// wallClock is the system clock: what a robot runs on.
type wallClock struct{}

func (wallClock) Now() time.Time                         { return time.Now() }
func (wallClock) After(d time.Duration) <-chan time.Time { return time.After(d) }
func (wallClock) Started() bool                          { return true }

func (wallClock) Ticker(period time.Duration) (<-chan time.Time, func()) {
	t := time.NewTicker(period)
	return t.C, t.Stop
}

// simClock is time as the simulator publishes it on /clock.
//
// Nothing here advances on its own: the clock moves when a message arrives, and
// everything waiting on it fires as that message passes their deadline. A
// simulation paused for a debugger's breakpoint therefore pauses the robot's
// timers too, which is the behaviour that makes simulated time worth having.
type simClock struct {
	mu      sync.Mutex
	now     time.Time
	started bool
	waiters []*simWaiter
}

// simWaiter is one After or Ticker waiting on the simulated clock.
//
// A waiter created before the simulator has published anything has no deadline
// yet — only a duration. Anchoring it to the zero time instead would make every
// such waiter fire on the first /clock message, which for a mission's 30-second
// timeout means it expires the instant the simulation starts. So the deadline
// is set when time is: waiting for two seconds means two seconds of the
// simulator's time, counted from when the simulator had a time to count from.
type simWaiter struct {
	at       time.Time
	after    time.Duration // the wait, until the clock starts and it becomes a deadline
	anchored bool
	period   time.Duration // zero for a one-shot After
	c        chan time.Time
	stopped  bool
}

func newSimClock() *simClock { return &simClock{} }

func (c *simClock) Now() time.Time {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.now
}

func (c *simClock) Started() bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.started
}

// Set moves the clock to t, firing whatever was waiting for it. Time going
// backwards — a simulation reset — drops the waiters' deadlines to the new now,
// so a reset does not leave a timer waiting for an hour that will not come.
func (c *simClock) Set(t time.Time) {
	c.mu.Lock()
	first := !c.started
	reset := c.started && t.Before(c.now)
	c.now, c.started = t, true
	if first {
		// Time has begun: what was a duration becomes a deadline.
		for _, w := range c.waiters {
			if !w.anchored {
				w.at, w.anchored = t.Add(w.after), true
			}
		}
	}
	if reset {
		for _, w := range c.waiters {
			if w.at.After(t) {
				w.at = t
			}
		}
	}

	var fired []*simWaiter
	kept := c.waiters[:0]
	for _, w := range c.waiters {
		if w.stopped {
			continue
		}
		if !w.anchored {
			kept = append(kept, w)
			continue
		}
		if !w.at.After(t) {
			fired = append(fired, w)
			if w.period > 0 {
				// Catch up without firing once per missed period: a jump in
				// simulated time is a jump, not a burst of ticks.
				w.at = t.Add(w.period)
				kept = append(kept, w)
			}
			continue
		}
		kept = append(kept, w)
	}
	c.waiters = kept
	c.mu.Unlock()

	// Sends happen outside the lock, and never block: a receiver that has not
	// drained its channel misses this tick, exactly as time.Ticker behaves.
	for _, w := range fired {
		select {
		case w.c <- t:
		default:
		}
	}
}

func (c *simClock) After(d time.Duration) <-chan time.Time {
	c.mu.Lock()
	defer c.mu.Unlock()
	w := &simWaiter{at: c.now.Add(d), after: d, anchored: c.started, c: make(chan time.Time, 1)}
	c.waiters = append(c.waiters, w)
	return w.c
}

func (c *simClock) Ticker(period time.Duration) (<-chan time.Time, func()) {
	c.mu.Lock()
	defer c.mu.Unlock()
	w := &simWaiter{
		at: c.now.Add(period), after: period, anchored: c.started,
		period: period, c: make(chan time.Time, 1),
	}
	c.waiters = append(c.waiters, w)
	return w.c, func() {
		c.mu.Lock()
		defer c.mu.Unlock()
		w.stopped = true
	}
}

// pending is how many waiters the clock is holding, for tests and diagnostics.
func (c *simClock) pending() int {
	c.mu.Lock()
	defer c.mu.Unlock()
	n := 0
	for _, w := range c.waiters {
		if !w.stopped {
			n++
		}
	}
	return n
}

// clockMsg is rosgraph_msgs/msg/Clock, the message a simulator publishes.
type clockMsg struct {
	Clock time.Time
}

// clockTopic is where a simulator publishes the world's time.
const clockTopic = "clock"

// The hash is rosgraph_msgs/msg/Clock's, computed from the installed
// definition by the same code that computes every other one.
const clockHash = "RIHS01_692f7a66e93a3c83e71765d033b60349ba68023a8c689a79e48078bcb5c58564"

func init() {
	RegisterMessage[clockMsg]("rosgraph_msgs/msg/Clock", clockHash)
}

// useSimTimeParam is the parameter every ROS 2 node carries to say which clock
// it is on. Conductor's answer is decided at startup rather than at runtime —
// switching a running robot's time base is not something to do halfway through
// a mission — but the parameter is exposed and truthful, because tools ask.
const useSimTimeParam = "use_sim_time"

// declareSimTimeParam gives a node the use_sim_time parameter, reporting what
// this process is actually doing. Setting it is refused rather than ignored: a
// tool that sets it believes it took effect, and a robot that quietly kept the
// wrong clock is worse than one that says no.
func declareSimTimeParam(rt *runtimeState, nr *nodeRuntime) {
	simulated := rt.simTime
	nr.params = append(nr.params, &paramHandle{
		name:   useSimTimeParam,
		get:    func() any { return simulated },
		typeOf: reflect.TypeFor[bool](),
		set: func(raw string) error {
			var want bool
			if err := parseYAMLScalar(&want, raw); err != nil {
				return err
			}
			if want == simulated {
				return nil
			}
			return fmt.Errorf("use_sim_time is %v for this process and cannot be changed while it runs; "+
				"start it with -use-sim-time%s instead", simulated, map[bool]string{true: "=false"}[simulated])
		},
	})
}

// subscribeClock wires the simulated clock to /clock. The subscription is
// declared on the runtime rather than on any node, because the clock is the
// process's, not one node's — and it is read even while nodes are inactive,
// since a node that has not been activated still has to know what time it is
// when it is.
func subscribeClock(rt *runtimeState, sim *simClock) error {
	q, _ := QoSProfile("sensor") // what a simulator publishes: fast, lossy, current
	return rt.transport.Subscribe(TopicSpec{
		Topic: clockTopic, QoS: q, Type: reflect.TypeFor[clockMsg](), Node: "clock",
	}, func(msg any, _ Metadata) {
		sim.Set(msg.(clockMsg).Clock)
	})
}

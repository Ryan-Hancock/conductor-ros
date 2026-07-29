package conductor

import (
	"context"
	"errors"
	"strings"
	"sync"
	"testing"
	"time"
)

// courier exercises the whole machine: a straight run, a dynamic branch, a
// retried step and a fail branch.
type courier struct {
	Trip Mission `start:"pickup"`

	Pickup   Step `next:"transit"`
	Transit  Step `next:"dropoff" fail:"recharge" retry:"1"`
	Dropoff  Step `next:"done"`
	Recharge Step `next:"transit"`

	mu       sync.Mutex
	visited  []string
	failures int // fail Transit this many times
	divert   bool
	blocking chan struct{} // when non-nil, Transit blocks on it
	stopped  chan struct{} // closed when a canceled step returns
}

func (c *courier) log(step string) {
	c.mu.Lock()
	c.visited = append(c.visited, step)
	c.mu.Unlock()
}

func (c *courier) steps() []string {
	c.mu.Lock()
	defer c.mu.Unlock()
	return append([]string{}, c.visited...)
}

func (c *courier) OnPickup(t *Task) error {
	c.log("pickup")
	return nil
}

func (c *courier) OnTransit(t *Task) error {
	c.log("transit")
	if c.blocking != nil {
		select {
		case <-c.blocking:
		case <-t.Context().Done():
			close(c.stopped)
			return t.Context().Err()
		}
	}
	c.mu.Lock()
	fail := c.failures > 0
	if fail {
		c.failures--
	}
	divert := c.divert
	c.divert = false
	c.mu.Unlock()
	switch {
	case fail:
		return errors.New("wheel slip")
	case divert:
		return t.Goto("recharge") // not where next: would have gone
	}
	return nil
}

func (c *courier) OnDropoff(t *Task) error {
	c.log("dropoff")
	return nil
}

func (c *courier) OnRecharge(t *Task) error {
	c.log("recharge")
	return nil
}

// awaitMission polls until the mission reaches status, or fails the test.
func awaitMission(t *testing.T, m *Mission, status MissionStatus) {
	t.Helper()
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		if m.Status() == status {
			return
		}
		time.Sleep(time.Millisecond)
	}
	t.Fatalf("mission is %s after 2s, want %s (step %s, err %v)", m.Status(), status, m.Step(), m.Err())
}

func TestMissionRunsDeclaredSteps(t *testing.T) {
	c := &courier{}
	app, err := NewTestApp(TestOptions{}, c)
	if err != nil {
		t.Fatal(err)
	}
	defer app.Close()

	awaitMission(t, &c.Trip, MissionDone)
	if got, want := strings.Join(c.steps(), " -> "), "pickup -> transit -> dropoff"; got != want {
		t.Fatalf("steps: %s, want %s", got, want)
	}
	if c.Trip.Step() != StepDone {
		t.Fatalf("final step %q, want %q", c.Trip.Step(), StepDone)
	}
}

func TestMissionGotoOverridesTheDeclaredNext(t *testing.T) {
	c := &courier{divert: true}
	app, err := NewTestApp(TestOptions{}, c)
	if err != nil {
		t.Fatal(err)
	}
	defer app.Close()

	awaitMission(t, &c.Trip, MissionDone)
	got, want := strings.Join(c.steps(), " -> "), "pickup -> transit -> recharge -> transit -> dropoff"
	if got != want {
		t.Fatalf("steps: %s, want %s", got, want)
	}
}

func TestMissionRetriesThenTakesTheFailBranch(t *testing.T) {
	// retry:"1" gives two attempts; three failures exhausts them and sends
	// the mission down fail:"recharge", which loops back into transit.
	c := &courier{failures: 3}
	app, err := NewTestApp(TestOptions{}, c)
	if err != nil {
		t.Fatal(err)
	}
	defer app.Close()

	awaitMission(t, &c.Trip, MissionDone)
	got := strings.Join(c.steps(), " -> ")
	want := "pickup -> transit -> transit -> recharge -> transit -> transit -> dropoff"
	if got != want {
		t.Fatalf("steps:\n got %s\nwant %s", got, want)
	}
}

func TestMissionFailsWhenAStepHasNowhereToGo(t *testing.T) {
	// A step with no fail branch and no retries fails the whole mission.
	d := &missionDeadEnd{}
	app, err := NewTestApp(TestOptions{}, d)
	if err != nil {
		t.Fatal(err)
	}
	defer app.Close()

	awaitMission(t, &d.Run, MissionFailed)
	if err := d.Run.Err(); err == nil || !strings.Contains(err.Error(), "no fuel") {
		t.Fatalf("mission error %v, want it to name the step failure", err)
	}
}

type missionDeadEnd struct {
	Run  Mission `start:"only"`
	Only Step    `next:"done"`
}

func (m *missionDeadEnd) OnOnly(t *Task) error { return errors.New("no fuel") }

func TestMissionStopsWhenTheNodeIsDeactivated(t *testing.T) {
	c := &courier{blocking: make(chan struct{}), stopped: make(chan struct{})}
	app, err := NewTestApp(TestOptions{}, c)
	if err != nil {
		t.Fatal(err)
	}
	defer app.Close()

	// Wait for the mission to be parked in the blocking step.
	deadline := time.Now().Add(2 * time.Second)
	for c.Trip.Step() != "transit" && time.Now().Before(deadline) {
		time.Sleep(time.Millisecond)
	}
	if err := app.Transition("courier", TransitionDeactivate); err != nil {
		t.Fatal(err)
	}
	select {
	case <-c.stopped:
	case <-time.After(time.Second):
		t.Fatal("the step's context was not canceled by deactivating the node")
	}
	if got := c.Trip.Status(); got != MissionCanceled {
		t.Fatalf("status %s after deactivate, want %s", got, MissionCanceled)
	}

	// Re-activating restarts the mission from its start step.
	close(c.blocking)
	c.blocking = nil
	if err := app.Transition("courier", TransitionActivate); err != nil {
		t.Fatal(err)
	}
	awaitMission(t, &c.Trip, MissionDone)
	if got := strings.Join(c.steps(), " -> "); !strings.HasSuffix(got, "pickup -> transit -> dropoff") {
		t.Fatalf("steps after restart: %s", got)
	}
}

func TestMissionWiringRejectsUndeclaredTargets(t *testing.T) {
	cases := []struct {
		name string
		node any
		want string
	}{
		{"unknown next", &missionBadNext{}, `names "nowhere"`},
		{"unknown start", &missionBadStart{}, `starts at "elsewhere"`},
		{"step without a mission", &missionNoMission{}, "no conductor.Mission field"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			app, err := NewTestApp(TestOptions{}, tc.node)
			if err == nil {
				app.Close()
				t.Fatal("wiring succeeded, want an error")
			}
			if !strings.Contains(err.Error(), tc.want) {
				t.Fatalf("error %q, want it to mention %q", err, tc.want)
			}
		})
	}
}

type missionBadNext struct {
	Run  Mission `start:"only"`
	Only Step    `next:"nowhere"`
}

func (m *missionBadNext) OnOnly(t *Task) error { return nil }

type missionBadStart struct {
	Run  Mission `start:"elsewhere"`
	Only Step    `next:"done"`
}

func (m *missionBadStart) OnOnly(t *Task) error { return nil }

type missionNoMission struct {
	Only Step `next:"done"`
}

func (m *missionNoMission) OnOnly(t *Task) error { return nil }

// A step may touch node state that callbacks own by going through the
// executor, which is what Task.Do is for.
type surveyor struct {
	Tick Timer   `rate:"100hz"`
	Run  Mission `start:"count"`

	Count Step `next:"done"`

	ticks int // executor-owned
	seen  int
}

func (s *surveyor) OnTick() { s.ticks++ }

func (s *surveyor) OnCount(t *Task) error {
	if err := t.Sleep(50 * time.Millisecond); err != nil {
		return err
	}
	// Reading s.ticks directly here would race with the executor; going
	// through Do makes the read part of the executor's own single-threaded
	// sequence.
	return t.Do(func() { s.seen = s.ticks })
}

func TestTaskDoRunsOnTheExecutor(t *testing.T) {
	s := &surveyor{}
	app, err := NewTestApp(TestOptions{RealTimers: true}, s)
	if err != nil {
		t.Fatal(err)
	}
	awaitMission(t, &s.Run, MissionDone)
	app.Close() // joins the executor, so the fields are safe to read here

	if s.seen == 0 {
		t.Fatal("the step saw no ticks; Task.Do did not observe executor state")
	}
	if s.seen > s.ticks {
		t.Fatalf("step read %d ticks, executor only ran %d", s.seen, s.ticks)
	}
}

// A step that overruns its timeout has its context canceled and takes the
// fail branch — the tag replaces the timer-and-flag this is usually written
// with.
type slowMission struct {
	Run     Mission `start:"crawl"`
	Crawl   Step    `next:"done" timeout:"30ms" fail:"recover"`
	Recover Step    `next:"done"`

	recovered chan error
}

func (s *slowMission) OnCrawl(t *Task) error {
	<-t.Context().Done()
	return t.Context().Err()
}

func (s *slowMission) OnRecover(t *Task) error {
	s.recovered <- t.Err()
	return nil
}

func TestMissionStepTimeoutTakesTheFailBranch(t *testing.T) {
	m := &slowMission{recovered: make(chan error, 1)}
	app, err := NewTestApp(TestOptions{}, m)
	if err != nil {
		t.Fatal(err)
	}
	defer app.Close()

	awaitMission(t, &m.Run, MissionDone)
	select {
	case cause := <-m.recovered:
		if !errors.Is(cause, context.DeadlineExceeded) {
			t.Fatalf("recovery step saw %v, want the deadline that sent it there", cause)
		}
	default:
		t.Fatal("the fail branch did not run")
	}
}

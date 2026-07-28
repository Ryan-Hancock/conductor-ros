package conductortest_test

import (
	"errors"
	"testing"
	"time"

	"conductor.dev/conductor"
	"conductor.dev/conductor/conductortest"
)

type reading struct{ Value float64 }
type command struct{ Speed float64 }
type limitRequest struct{ Limit float64 }
type limitResponse struct{ Applied float64 }

// controller is a miniature but complete node: a subscription that reacts, a
// timer that publishes, a parameter that bounds it, a service that changes
// the parameter, and lifecycle hooks.
type controller struct {
	In    conductor.Sub[reading]                     `topic:"sensor"`
	Out   conductor.Pub[command]                     `topic:"cmd"`
	Beat  conductor.Timer                            `rate:"10hz"`
	Limit conductor.Param[float64]                   `name:"limit" default:"1.0"`
	SetLm conductor.Svc[limitRequest, limitResponse] `service:"set_limit"`
	Deny  conductor.Svc[limitRequest, limitResponse] `service:"deny"`

	last       float64
	configured int
	activated  int
}

func (c *controller) OnIn(r reading) {
	c.last = r.Value
	c.Out.Publish(command{Speed: min(r.Value, c.Limit.Get())})
}

func (c *controller) OnBeat() { c.Out.Publish(command{Speed: min(c.last, c.Limit.Get())}) }

func (c *controller) OnSetLm(req limitRequest) (limitResponse, error) {
	return limitResponse{Applied: req.Limit}, nil
}

func (c *controller) OnDeny(limitRequest) (limitResponse, error) {
	return limitResponse{}, errors.New("nope")
}

func (c *controller) OnConfigure() error { c.configured++; return nil }
func (c *controller) OnActivate() error  { c.activated++; return nil }

func TestPublishReachesSubscriber(t *testing.T) {
	app := conductortest.Run(t, &controller{})
	cmd := conductortest.Watch[command](app, "cmd")

	conductortest.Publish(app, "sensor", reading{Value: 0.4})

	// Publish waits for the callbacks it causes, so no sleeping here.
	if got := cmd.Len(); got != 1 {
		t.Fatalf("recorded %d commands, want 1", got)
	}
	last, _ := cmd.Last()
	if last.Speed != 0.4 {
		t.Fatalf("speed = %v, want 0.4", last.Speed)
	}
}

func TestParameterBoundsBehaviour(t *testing.T) {
	app := conductortest.RunWith(t, conductortest.Options{
		Params: map[string]map[string]string{"controller": {"limit": "0.25"}},
	}, &controller{})
	cmd := conductortest.Watch[command](app, "cmd")

	conductortest.Publish(app, "sensor", reading{Value: 5})
	if last, _ := cmd.Last(); last.Speed != 0.25 {
		t.Fatalf("speed = %v, want the parameter file's 0.25", last.Speed)
	}

	app.SetParam("controller", "limit", "2.0")
	conductortest.Publish(app, "sensor", reading{Value: 5})
	if last, _ := cmd.Last(); last.Speed != 2.0 {
		t.Fatalf("speed = %v, want 2.0 after the update", last.Speed)
	}
	if got := app.Param("controller", "limit"); got != 2.0 {
		t.Fatalf("Param = %v, want 2.0", got)
	}
}

func TestTimersFireOnlyOnTick(t *testing.T) {
	app := conductortest.Run(t, &controller{})
	cmd := conductortest.Watch[command](app, "cmd")

	// A 10hz timer would have fired several times by now if the harness let
	// the clock drive it.
	time.Sleep(150 * time.Millisecond)
	if got := cmd.Len(); got != 0 {
		t.Fatalf("timer fired %d times without a Tick", got)
	}

	if n := app.Tick("controller"); n != 1 {
		t.Fatalf("fired %d timers, want 1", n)
	}
	app.TickN("controller", 2)
	if got := cmd.Len(); got != 3 {
		t.Fatalf("recorded %d commands, want 3", got)
	}
}

func TestServiceCall(t *testing.T) {
	app := conductortest.Run(t, &controller{})

	res, err := conductortest.Call[limitRequest, limitResponse](app, "set_limit", limitRequest{Limit: 3})
	if err != nil {
		t.Fatal(err)
	}
	if res.Applied != 3 {
		t.Fatalf("applied = %v, want 3", res.Applied)
	}

	if _, err := conductortest.Call[limitRequest, limitResponse](app, "deny", limitRequest{}); err == nil {
		t.Fatal("expected the handler's error to reach the caller")
	}
}

func TestLifecycleGating(t *testing.T) {
	app := conductortest.Run(t, &controller{})
	cmd := conductortest.Watch[command](app, "cmd")

	if s := app.State("controller"); s != conductor.StateActive {
		t.Fatalf("state = %v, want active", s)
	}

	app.Transition("controller", conductor.TransitionDeactivate)
	app.Tick("controller")
	conductortest.Publish(app, "sensor", reading{Value: 1})
	if got := cmd.Len(); got != 0 {
		t.Fatalf("inactive node published %d messages", got)
	}

	app.Transition("controller", conductor.TransitionActivate)
	conductortest.Publish(app, "sensor", reading{Value: 1})
	if got := cmd.Len(); got != 1 {
		t.Fatalf("reactivated node published %d messages, want 1", got)
	}
}

func TestManualLifecycleRunsHooks(t *testing.T) {
	c := &controller{}
	app := conductortest.RunWith(t, conductortest.Options{ManualLifecycle: true}, c)

	if s := app.State("controller"); s != conductor.StateUnconfigured {
		t.Fatalf("state = %v, want unconfigured", s)
	}
	app.Transition("controller", conductor.TransitionConfigure)
	app.Transition("controller", conductor.TransitionActivate)
	if c.configured != 1 || c.activated != 1 {
		t.Fatalf("hooks ran %d/%d times, want 1/1", c.configured, c.activated)
	}
}

// A probe is an extra node the test owns: here it drives the same topics
// from outside, which is also how an action client would be attached.
func TestProbe(t *testing.T) {
	app := conductortest.Run(t, &controller{})
	var probe struct {
		Sensor conductor.Pub[reading] `topic:"sensor"`
	}
	app.Probe("driver", &probe)
	cmd := conductortest.Watch[command](app, "cmd")

	probe.Sensor.Publish(reading{Value: 0.75})
	app.Settle()

	if last, ok := cmd.Last(); !ok || last.Speed != 0.75 {
		t.Fatalf("last command = %v (recorded %d), want 0.75", last, cmd.Len())
	}
}

type stepGoal struct{ Steps int32 }
type stepFeedback struct{ At int32 }
type stepResult struct{ Done int32 }

func init() {
	conductor.RegisterAction[stepGoal, stepFeedback, stepResult](conductor.ActionInfo{
		Name:            "test_pkg/action/Step",
		SendGoal:        conductor.MessageInfo{Name: "test_pkg/action/Step_SendGoal", Hash: "RIHS01_test"},
		GetResult:       conductor.MessageInfo{Name: "test_pkg/action/Step_GetResult", Hash: "RIHS01_test"},
		FeedbackMessage: conductor.MessageInfo{Name: "test_pkg/action/Step_FeedbackMessage", Hash: "RIHS01_test"},
	})
}

type stepper struct {
	Walk conductor.Action[stepGoal, stepFeedback, stepResult] `action:"walk"`
}

func (s *stepper) OnWalk(g *conductor.Goal[stepGoal, stepFeedback]) (stepResult, error) {
	var i int32
	for i = 0; i < g.Value().Steps; i++ {
		g.Feedback(stepFeedback{At: i})
	}
	return stepResult{Done: i}, nil
}

// An action client is a probe's job: the goal, feedback and result protocol
// runs over the same transport it would in production.
func TestActionServerThroughProbe(t *testing.T) {
	app := conductortest.Run(t, &stepper{})
	var driver struct {
		Walk conductor.ActionClient[stepGoal, stepFeedback, stepResult] `action:"walk" timeout:"10s"`
	}
	app.Probe("driver", &driver)

	h, err := driver.Walk.SendGoal(stepGoal{Steps: 3})
	if err != nil {
		t.Fatal(err)
	}
	seen := make(chan int, 1)
	go func() {
		n := 0
		for range h.Feedback() {
			n++
		}
		seen <- n
	}()

	res, status, err := h.Result()
	if err != nil {
		t.Fatal(err)
	}
	if !status.Succeeded() || res.Done != 3 {
		t.Fatalf("status %v, done %d; want SUCCEEDED and 3", status, res.Done)
	}
	if n := <-seen; n == 0 {
		t.Fatal("no feedback reached the client")
	}
}

func TestAwait(t *testing.T) {
	app := conductortest.Run(t, &controller{})
	cmd := conductortest.Watch[command](app, "cmd")

	go func() {
		time.Sleep(20 * time.Millisecond)
		conductortest.Publish(app, "sensor", reading{Value: 0.9})
	}()

	got := cmd.Await(t, 2*time.Second)
	if got.Speed != 0.9 {
		t.Fatalf("speed = %v, want 0.9", got.Speed)
	}
}

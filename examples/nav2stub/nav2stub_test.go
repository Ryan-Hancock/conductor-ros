package main

import (
	"math"
	"testing"
	"time"

	"conductor.dev/conductor"
	"conductor.dev/conductor/conductortest"
)

// A stand-in that nobody tests is a stand-in that quietly stops standing in.
// These are the behaviours examples/nav2 depends on: managed nodes that start
// Unconfigured and refuse work until activated, goals that complete, goals that
// fail on schedule, and a stack that publishes what a stack publishes.

// driver is the client half, wired in as a probe — the same action clients the
// commander declares.
type driver struct {
	NavTo conductor.ActionClient[NavigateToPoseGoal, NavigateToPoseFeedback, NavigateToPoseResult] `action:"navigate_to_pose" timeout:"30s"`
	Back  conductor.ActionClient[BackUpGoal, BackUpFeedback, BackUpResult]                         `action:"backup" timeout:"30s"`
}

// runStub wires the whole stand-in with its lifecycle manual, as
// `-lifecycle manual` does in the environment: nothing is up until a test (or a
// commander) brings it up.
func runStub(t *testing.T, params map[string]string) (*conductortest.App, *driver) {
	t.Helper()
	tree, err := conductor.LoadFrames("frames.json")
	if err != nil || tree == nil {
		t.Fatalf("loading frames.json: %v", err)
	}
	r := &robot{}
	app := conductortest.RunWith(t, conductortest.Options{
		ManualLifecycle: true,
		Frames:          tree,
		Params: map[string]map[string]string{
			"bt_navigator":      params,
			"controller_server": params,
		},
	},
		&MapServer{}, &Amcl{robot: r}, &ControllerServer{robot: r},
		&PlannerServer{}, &BehaviorServer{robot: r}, &BtNavigator{robot: r},
	)
	d := &driver{}
	app.Probe("driver", d)
	return app, d
}

// bringUp is what the commander's Lifecycle field does, in the order it
// declares.
func bringUp(t *testing.T, app *conductortest.App) {
	t.Helper()
	nodes := []string{"map_server", "amcl", "controller_server", "planner_server",
		"behavior_server", "bt_navigator"}
	for _, tr := range []conductor.Transition{conductor.TransitionConfigure, conductor.TransitionActivate} {
		for _, node := range nodes {
			app.Transition(node, tr)
		}
	}
	for _, node := range nodes {
		if got := app.State(node); got != conductor.StateActive {
			t.Fatalf("%s is %s after bringup, want active", node, got)
		}
	}
}

func goalAt(x, y float64) NavigateToPoseGoal {
	return NavigateToPoseGoal{Pose: PoseStamped{Pose: Pose{Position: Point{X: x, Y: y}}}}
}

// Nothing navigates before the stack has been activated. This is the behaviour
// that makes the commander's first mission step meaningful rather than
// decorative — and the reason the stand-in is six managed nodes.
func TestGoalsFailUntilTheStackIsActive(t *testing.T) {
	app, d := runStub(t, map[string]string{"speed": "4.0", "fail_every": "0"})

	if got := app.State("bt_navigator"); got != conductor.StateUnconfigured {
		t.Fatalf("bt_navigator starts %s, want unconfigured", got)
	}
	h, err := d.NavTo.SendGoal(goalAt(1, 0))
	if err != nil {
		t.Fatalf("sending a goal: %v", err)
	}
	res, status, err := h.Result()
	if err != nil {
		t.Fatalf("waiting for the result: %v", err)
	}
	if status.Succeeded() {
		t.Fatal("a goal succeeded while the navigation stack was inactive")
	}
	if res.ErrorMsg == "" {
		t.Error("an aborted goal came back with no error message")
	}

	// Brought up, the same goal completes.
	bringUp(t, app)
	h, err = d.NavTo.SendGoal(goalAt(1, 0))
	if err != nil {
		t.Fatalf("sending a goal: %v", err)
	}
	if _, status, err := h.Result(); err != nil {
		t.Fatalf("waiting for the result: %v", err)
	} else if !status.Succeeded() {
		t.Fatalf("goal %s with the stack active", status)
	}
}

// Deactivating stops it working again, which is what makes a lifecycle worth
// having: the commander's failure exit deactivates the stack rather than
// leaving servers active with nobody driving them.
func TestGoalsFailAgainAfterDeactivation(t *testing.T) {
	app, d := runStub(t, map[string]string{"speed": "4.0", "fail_every": "0"})
	bringUp(t, app)
	app.Transition("bt_navigator", conductor.TransitionDeactivate)

	h, err := d.NavTo.SendGoal(goalAt(1, 0))
	if err != nil {
		t.Fatal(err)
	}
	if _, status, err := h.Result(); err != nil {
		t.Fatal(err)
	} else if status.Succeeded() {
		t.Fatal("a goal succeeded against a deactivated bt_navigator")
	}
}

// fail_every aborts one goal in n, which is what keeps the commander's recovery
// branch exercised by `make nav2` rather than only by `go test`.
func TestFailEveryAbortsOneGoalInN(t *testing.T) {
	app, d := runStub(t, map[string]string{"speed": "4.0", "fail_every": "2"})
	bringUp(t, app)

	var aborted int
	for i := 0; i < 4; i++ {
		h, err := d.NavTo.SendGoal(goalAt(float64(i%2)*1.5, 0))
		if err != nil {
			t.Fatalf("sending goal %d: %v", i, err)
		}
		_, status, err := h.Result()
		if err != nil {
			t.Fatalf("goal %d: %v", i, err)
		}
		if !status.Succeeded() {
			aborted++
		}
	}
	if aborted != 2 {
		t.Fatalf("%d of 4 goals aborted, want 2 with fail_every 2", aborted)
	}
}

// The stack publishes what a stack publishes: a localized pose from amcl and a
// velocity command from the controller — the zero command included, because a
// controller holding still is what a stall looks like from outside.
func TestTheStackPublishesItsState(t *testing.T) {
	app, _ := runStub(t, map[string]string{"fail_every": "0"})
	poses := conductortest.Watch[PoseWithCovarianceStamped](app, "amcl_pose")
	cmds := conductortest.Watch[TwistStamped](app, "cmd_vel")

	// Inactive, its publishers are silent: conductor gates those on Active
	// without the node having to think about it.
	app.Tick("amcl")
	app.Tick("controller_server")
	if poses.Len() != 0 || cmds.Len() != 0 {
		t.Fatalf("an unconfigured stack published %d pose(s) and %d command(s)", poses.Len(), cmds.Len())
	}

	bringUp(t, app)
	app.Tick("amcl")
	app.Tick("controller_server")

	pose, ok := poses.Last()
	if !ok {
		t.Fatal("no pose published")
	}
	if pose.Header.FrameId != "map" {
		t.Fatalf("pose stamped %q, want map", pose.Header.FrameId)
	}
	cmd, ok := cmds.Last()
	if !ok {
		t.Fatal("no velocity command published")
	}
	if cmd.Header.FrameId != "base_link" {
		t.Fatalf("command stamped %q, want base_link", cmd.Header.FrameId)
	}
	if speed := math.Hypot(cmd.Twist.Linear.X, cmd.Twist.Linear.Y); speed != 0 {
		t.Fatalf("standing still but commanding %v", speed)
	}
}

// Driving is visible on cmd_vel, which is how the watchdog — and the battery
// driver in .tools/fake_battery.py — know the robot is moving.
func TestDrivingShowsOnCmdVel(t *testing.T) {
	app, d := runStub(t, map[string]string{"speed": "0.5", "fail_every": "0"})
	bringUp(t, app)
	cmds := conductortest.Watch[TwistStamped](app, "cmd_vel")

	h, err := d.NavTo.SendGoal(goalAt(3, 0))
	if err != nil {
		t.Fatal(err)
	}
	defer func() { h.Cancel(); h.Result() }()

	deadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(deadline) {
		app.Tick("controller_server")
		if cmd, ok := cmds.Last(); ok && cmd.Twist.Linear.X > 0 {
			return
		}
		time.Sleep(10 * time.Millisecond)
	}
	t.Fatal("the controller never commanded any motion while a goal was under way")
}

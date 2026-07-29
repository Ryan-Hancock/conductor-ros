package main

import (
	"math"
	"testing"
	"time"

	"conductor.dev/conductor"
	"conductor.dev/conductor/conductortest"
)

// A stand-in that nobody tests is a stand-in that quietly stops standing in.
// These are the two behaviours examples/nav2 depends on: the lifecycle gate,
// and goals that actually complete.

// driver is the client half, wired in as a probe — the same three action
// clients the commander declares.
type driver struct {
	NavTo conductor.ActionClient[NavigateToPoseGoal, NavigateToPoseFeedback, NavigateToPoseResult] `action:"navigate_to_pose" timeout:"30s"`
	Back  conductor.ActionClient[BackUpGoal, BackUpFeedback, BackUpResult]                         `action:"backup" timeout:"30s"`
}

func runStub(t *testing.T, params map[string]string) (*conductortest.App, *driver) {
	t.Helper()
	tree, err := conductor.LoadFrames("frames.json")
	if err != nil || tree == nil {
		t.Fatalf("loading frames.json: %v", err)
	}
	app := conductortest.RunWith(t, conductortest.Options{
		Frames: tree,
		Params: map[string]map[string]string{"nav2": params},
	}, &Nav2{})
	d := &driver{}
	app.Probe("driver", d)
	return app, d
}

func goalAt(x, y float64) NavigateToPoseGoal {
	return NavigateToPoseGoal{Pose: PoseStamped{Pose: Pose{Position: Point{X: x, Y: y}}}}
}

// Nothing navigates before the lifecycle manager has started the stack. This
// is the behaviour that makes the commander's first mission step meaningful
// rather than decorative.
func TestGoalsFailUntilTheStackIsStarted(t *testing.T) {
	app, d := runStub(t, map[string]string{"speed": "4.0", "fail_every": "0"})

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

	// Started, the same goal completes and the robot ends up where it was sent.
	if _, err := conductortest.Call[ManageLifecycleNodesRequest, ManageLifecycleNodesResponse](
		app, "lifecycle_manager_navigation/manage_nodes",
		ManageLifecycleNodesRequest{Command: ManageLifecycleNodes_Request_STARTUP}); err != nil {
		t.Fatal(err)
	}

	h, err = d.NavTo.SendGoal(goalAt(1, 0))
	if err != nil {
		t.Fatalf("sending a goal: %v", err)
	}
	if _, status, err := h.Result(); err != nil {
		t.Fatalf("waiting for the result: %v", err)
	} else if !status.Succeeded() {
		t.Fatalf("goal %s with the stack started", status)
	}
}

// fail_every aborts one goal in n, which is what keeps the commander's
// recovery branch exercised by `make nav2` rather than only by `go test`.
func TestFailEveryAbortsOneGoalInN(t *testing.T) {
	app, d := runStub(t, map[string]string{"speed": "4.0", "fail_every": "2"})
	if _, err := conductortest.Call[ManageLifecycleNodesRequest, ManageLifecycleNodesResponse](
		app, "lifecycle_manager_navigation/manage_nodes",
		ManageLifecycleNodesRequest{Command: ManageLifecycleNodes_Request_STARTUP}); err != nil {
		t.Fatal(err)
	}

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

// The stack publishes what a stack publishes: a pose, a velocity command and a
// battery, every cycle — the zero command included, because a controller
// holding still is what a stall looks like from outside.
func TestTheStackPublishesItsState(t *testing.T) {
	app, _ := runStub(t, map[string]string{"fail_every": "0"})
	poses := conductortest.Watch[PoseWithCovarianceStamped](app, "amcl_pose")
	cmds := conductortest.Watch[TwistStamped](app, "cmd_vel")
	batteries := conductortest.Watch[BatteryState](app, "battery_state")

	app.Tick("nav2")

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
	if battery, _ := batteries.Last(); battery.Percentage != 1.0 {
		t.Fatalf("battery %v at rest on the dock, want a full 1.0", battery.Percentage)
	}
}

// Driving drains the battery and parking on the dock charges it again; the
// commander's docking and charging steps are waiting for exactly this.
func TestDrivingDrainsAndTheDockCharges(t *testing.T) {
	app, d := runStub(t, map[string]string{
		// Draining fast and charging slowly keeps both measurements clear of
		// the full/empty clamps, whatever order the ticks land in.
		"speed": "0.5", "drain_per_second": "5.0", "charge_per_second": "0.5", "fail_every": "0",
	})
	batteries := conductortest.Watch[BatteryState](app, "battery_state")
	if _, err := conductortest.Call[ManageLifecycleNodesRequest, ManageLifecycleNodesResponse](
		app, "lifecycle_manager_navigation/manage_nodes",
		ManageLifecycleNodesRequest{Command: ManageLifecycleNodes_Request_STARTUP}); err != nil {
		t.Fatal(err)
	}

	// Away from the dock, under way: the timer's ticks drain it. A goal is
	// accepted before its handler has started driving, so this waits for the
	// state rather than assuming it.
	h, err := d.NavTo.SendGoal(goalAt(3, 0))
	if err != nil {
		t.Fatal(err)
	}
	drained := awaitBatteryStatus(t, app, batteries, BatteryState_POWER_SUPPLY_STATUS_DISCHARGING)
	if drained.Percentage >= 1.0 {
		t.Fatalf("battery %v while driving, want it draining", drained.Percentage)
	}
	h.Cancel()
	h.Result()

	// Home again, stopped: it recovers. Charging is measured between two
	// samples taken while docked — the driving in between spent charge, so
	// comparing against the earlier reading would prove nothing.
	home, err := d.NavTo.SendGoal(goalAt(0, 0))
	if err != nil {
		t.Fatal(err)
	}
	if _, status, err := home.Result(); err != nil || !status.Succeeded() {
		t.Fatalf("driving home: %s, %v", status, err)
	}
	first := awaitBatteryStatus(t, app, batteries, BatteryState_POWER_SUPPLY_STATUS_CHARGING)
	app.TickN("nav2", 2)
	later, _ := batteries.Last()
	if later.PowerSupplyStatus != BatteryState_POWER_SUPPLY_STATUS_CHARGING {
		t.Fatalf("power supply status %d while parked on the dock, want CHARGING", later.PowerSupplyStatus)
	}
	if later.Percentage <= first.Percentage {
		t.Fatalf("battery went %v -> %v on the dock, want it rising", first.Percentage, later.Percentage)
	}
}

// awaitBatteryStatus ticks the node until the battery reports the status
// wanted, and fails the test if it never does.
func awaitBatteryStatus(t *testing.T, app *conductortest.App, batteries *conductortest.Recorder[BatteryState], want uint8) BatteryState {
	t.Helper()
	deadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(deadline) {
		app.Tick("nav2")
		if got, ok := batteries.Last(); ok && got.PowerSupplyStatus == want {
			return got
		}
		time.Sleep(10 * time.Millisecond)
	}
	t.Fatalf("the battery never reported power supply status %d", want)
	return BatteryState{}
}

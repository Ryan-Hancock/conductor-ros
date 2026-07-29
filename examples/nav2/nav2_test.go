package main

import (
	"errors"
	"fmt"
	"math"
	"testing"
	"time"

	"conductor.dev/conductor"
	"conductor.dev/conductor/conductortest"
)

// These tests run the commander's mission against a scripted Nav2: a probe
// wired into the app under test that serves the same three action servers and
// the lifecycle manager's service, and answers however the test needs it to.
//
// That is the point of driving a stack through declared interfaces rather than
// a behaviour tree XML. "What does this robot do when navigate_to_pose
// aborts?" is a question `go test` can answer in ten milliseconds, with no
// ROS install, no simulator, and no waiting for the fourth waypoint.

// fakeNav2 is Nav2's half of the conversation, under the test's control.
type fakeNav2 struct {
	Manage conductor.Svc[ManageLifecycleNodesRequest, ManageLifecycleNodesResponse] `service:"lifecycle_manager_navigation/manage_nodes"`

	NavTo conductor.Action[NavigateToPoseGoal, NavigateToPoseFeedback, NavigateToPoseResult] `action:"navigate_to_pose"`
	Back  conductor.Action[BackUpGoal, BackUpFeedback, BackUpResult]                         `action:"backup"`
	Spin  conductor.Action[SpinGoal, SpinFeedback, SpinResult]                               `action:"spin"`

	// active is what the lifecycle manager reports; a false one is a stack
	// that will not come up.
	active bool

	commands   chan uint8              // manage_nodes commands received
	goals      chan NavigateToPoseGoal // navigate_to_pose goals received
	recoveries chan string             // recovery behaviours run, in order
	outcomes   chan error              // scripted failures, one per goal
}

func newFakeNav2(active bool) *fakeNav2 {
	return &fakeNav2{
		active:     active,
		commands:   make(chan uint8, 16),
		goals:      make(chan NavigateToPoseGoal, 16),
		recoveries: make(chan string, 16),
		outcomes:   make(chan error, 16),
	}
}

func (f *fakeNav2) OnManage(req ManageLifecycleNodesRequest) (ManageLifecycleNodesResponse, error) {
	f.commands <- req.Command
	return ManageLifecycleNodesResponse{Success: f.active}, nil
}

// OnNavTo accepts a goal and completes it, unless the test has queued a
// failure for it — which Nav2 reports as an aborted goal carrying an error
// code, not as a transport error.
func (f *fakeNav2) OnNavTo(g *conductor.Goal[NavigateToPoseGoal, NavigateToPoseFeedback]) (NavigateToPoseResult, error) {
	f.goals <- g.Value()
	select {
	case err := <-f.outcomes:
		if err != nil {
			return NavigateToPoseResult{
				ErrorCode: NavigateToPose_Result_UNKNOWN,
				ErrorMsg:  err.Error(),
			}, err
		}
	default:
	}
	return NavigateToPoseResult{}, nil
}

func (f *fakeNav2) OnBack(g *conductor.Goal[BackUpGoal, BackUpFeedback]) (BackUpResult, error) {
	f.recoveries <- "backup"
	return BackUpResult{}, nil
}

func (f *fakeNav2) OnSpin(g *conductor.Goal[SpinGoal, SpinFeedback]) (SpinResult, error) {
	f.recoveries <- "spin"
	return SpinResult{}, nil
}

// runCommander starts the commander with a scripted Nav2 behind it. The
// lifecycle is manual because the mission starts on Activate, and the stack
// has to be answering by then — the same ordering problem the mission's first
// step exists to solve on a real robot.
func runCommander(t *testing.T, nav *fakeNav2, params map[string]string) (*conductortest.App, *Commander) {
	t.Helper()
	tree, err := conductor.LoadFrames("frames.json")
	if err != nil || tree == nil {
		t.Fatalf("loading frames.json: %v", err)
	}
	if params == nil {
		params = map[string]string{}
	}
	// A short dwell keeps the test quick; the route's timing is a parameter
	// like any other.
	if _, ok := params["dwell"]; !ok {
		params["dwell"] = "10ms"
	}

	commander := &Commander{}
	app := conductortest.RunWith(t, conductortest.Options{
		ManualLifecycle: true,
		Frames:          tree,
		Params:          map[string]map[string]string{"commander": params},
	}, commander)
	app.Probe("nav2", nav)
	return app, commander
}

// activate brings the commander up and answers the localize step with a pose,
// as amcl would.
func activate(t *testing.T, app *conductortest.App) {
	t.Helper()
	app.Transition("commander", conductor.TransitionConfigure)
	app.Transition("commander", conductor.TransitionActivate)
	publishPose(app, 0, 0)
}

func publishPose(app *conductortest.App, x, y float64) {
	conductortest.Publish(app, "amcl_pose", PoseWithCovarianceStamped{
		Pose: PoseWithCovariance{Pose: Pose{Position: Point{X: x, Y: y}}},
	})
}

func publishBattery(app *conductortest.App, charge float32) {
	conductortest.Publish(app, "battery_state", BatteryState{Percentage: charge, Present: true})
}

func await[T any](t *testing.T, ch <-chan T, what string) T {
	t.Helper()
	select {
	case v := <-ch:
		return v
	case <-time.After(5 * time.Second):
		var zero T
		t.Fatalf("timed out waiting for %s", what)
		return zero
	}
}

// The stack is brought up before anything is commanded, AMCL is seeded, and
// the first waypoint of the route goes to navigate_to_pose.
func TestBringUpThenNavigate(t *testing.T) {
	nav := newFakeNav2(true)
	app, commander := runCommander(t, nav, nil)
	estimates := conductortest.Watch[PoseWithCovarianceStamped](app, "initialpose")
	waypoints := conductortest.Watch[PoseStamped](app, "patrol/waypoint")

	activate(t, app)

	if cmd := await(t, nav.commands, "a manage_nodes command"); cmd != ManageLifecycleNodes_Request_STARTUP {
		t.Fatalf("lifecycle command %d, want STARTUP", cmd)
	}
	// The localize step seeds amcl, and it does so before anything is
	// commanded: navigating without a pose is how a robot drives into a wall.
	estimates.Await(t, 2*time.Second)

	goal := await(t, nav.goals, "a navigate_to_pose goal")
	want := commander.route[0]
	if goal.Pose.Pose.Position != want {
		t.Fatalf("first goal %v, want the first waypoint %v", goal.Pose.Pose.Position, want)
	}

	// The waypoint is published for the rest of the application too, stamped
	// in the frame the tag declares.
	w := waypoints.Await(t, time.Second)
	if w.Header.FrameId != "map" {
		t.Fatalf("waypoint stamped %q, want the declared map frame", w.Header.FrameId)
	}
}

// Nothing is commanded while the stack is refusing to start, and the retry tag
// keeps asking rather than giving up on the first no.
func TestNothingIsCommandedUntilTheStackIsActive(t *testing.T) {
	nav := newFakeNav2(false)
	app, _ := runCommander(t, nav, nil)

	activate(t, app)

	// bring_up declares retry:"3" backoff:"2s", so a second attempt is proof
	// the tag is doing the work a retry loop would otherwise do by hand.
	await(t, nav.commands, "the first startup attempt")
	await(t, nav.commands, "a retried startup attempt")

	select {
	case goal := <-nav.goals:
		t.Fatalf("commanded %v before the navigation stack was active", goal.Pose.Pose.Position)
	default:
	}
	if status, step := app.Mission("commander"); status != conductor.MissionRunning || step != "bring_up" {
		t.Fatalf("mission %s at %q, want it still running bring_up", status, step)
	}
}

// An aborted goal takes the fail: branch, which runs Nav2's own recovery
// behaviours and then tries the same waypoint again.
func TestAbortedGoalRunsNav2sRecoveryBehaviours(t *testing.T) {
	nav := newFakeNav2(true)
	nav.outcomes <- errors.New("controller could not make progress")
	app, commander := runCommander(t, nav, nil)

	activate(t, app)

	first := await(t, nav.goals, "the first goal")
	if got := await(t, nav.recoveries, "the backup recovery"); got != "backup" {
		t.Fatalf("first recovery %q, want backup", got)
	}
	if got := await(t, nav.recoveries, "the spin recovery"); got != "spin" {
		t.Fatalf("second recovery %q, want spin", got)
	}

	retried := await(t, nav.goals, "the retried goal")
	if retried.Pose.Pose.Position != first.Pose.Pose.Position {
		t.Fatalf("after recovering, navigated to %v; want the same waypoint %v",
			retried.Pose.Pose.Position, first.Pose.Pose.Position)
	}
	if commander.at != 0 {
		t.Fatalf("route index advanced to %d over a failed waypoint", commander.at)
	}
}

// A low battery on arrival diverts to the dock, waits for the charge, and
// rejoins the route where it left off. The Goto that does it is checked
// statically: `conductor check` resolves "docking" against the declared steps.
func TestLowBatteryDivertsToTheDock(t *testing.T) {
	nav := newFakeNav2(true)
	app, commander := runCommander(t, nav, map[string]string{
		"low_battery":    "0.3",
		"resume_battery": "0.9",
	})

	activate(t, app)
	publishBattery(app, 0.12)

	first := await(t, nav.goals, "the first waypoint")
	if first.Pose.Pose.Position != commander.route[0] {
		t.Fatalf("first goal %v, want the first waypoint", first.Pose.Pose.Position)
	}

	dock := await(t, nav.goals, "the dock")
	if dock.Pose.Pose.Position != commander.dock {
		t.Fatalf("diverted to %v, want the dock %v", dock.Pose.Pose.Position, commander.dock)
	}

	// Charging holds the mission until the battery comes back.
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		if _, step := app.Mission("commander"); step == "charging" {
			break
		}
		time.Sleep(time.Millisecond)
	}
	if _, step := app.Mission("commander"); step != "charging" {
		t.Fatalf("mission at %q after reaching the dock, want charging", step)
	}

	publishBattery(app, 0.95)
	resumed := await(t, nav.goals, "the route to resume")
	if resumed.Pose.Pose.Position != commander.route[1] {
		t.Fatalf("resumed at %v, want the next waypoint %v", resumed.Pose.Pose.Position, commander.route[1])
	}
}

// The watchdog reports a stack that has stopped moving with a waypoint still
// outstanding — the failure that otherwise shows up as a robot sitting still
// while every node reports itself healthy.
func TestWatchdogReportsAStall(t *testing.T) {
	app := conductortest.RunWith(t, conductortest.Options{
		Frames: mustFrames(t),
		Params: map[string]map[string]string{"watchdog": {"stall_after": "1ms"}},
	}, &Watchdog{})
	diagnostics := conductortest.Watch[DiagnosticArray](app, "diagnostics")

	// Localized, navigating, and the controller is asking for motion.
	publishPose(app, 0, 0)
	conductortest.Publish(app, "patrol/waypoint", PoseStamped{Pose: Pose{Position: Point{X: 2}}})
	conductortest.Publish(app, "cmd_vel", TwistStamped{Twist: Twist{Linear: Vector3{X: 0.4}}})
	app.Tick("watchdog")

	got, ok := diagnostics.Last()
	if !ok || len(got.Status) == 0 {
		t.Fatal("the watchdog published no diagnostics")
	}
	if got.Status[0].Level != DiagnosticStatus_OK {
		t.Fatalf("level %d while navigating (%q), want OK", got.Status[0].Level, got.Status[0].Message)
	}
	if d := value(got.Status[0], "distance_to_waypoint"); math.Abs(d-2) > 1e-9 {
		t.Fatalf("distance_to_waypoint %v, want 2", d)
	}

	// The controller is still publishing, but zero: Nav2 holding still with a
	// goal outstanding is a stall, not silence.
	conductortest.Publish(app, "cmd_vel", TwistStamped{})
	time.Sleep(2 * time.Millisecond)
	app.Tick("watchdog")

	if got, _ := diagnostics.Last(); got.Status[0].Level != DiagnosticStatus_WARN {
		t.Fatalf("level %d for a stalled stack (%q), want WARN", got.Status[0].Level, got.Status[0].Message)
	}
}

// A robot parked on its waypoint is not stalled, however long it sits there —
// which is what the whole charging step looks like from here.
func TestWatchdogDoesNotCallArrivalAStall(t *testing.T) {
	app := conductortest.RunWith(t, conductortest.Options{
		Frames: mustFrames(t),
		Params: map[string]map[string]string{"watchdog": {"stall_after": "1ms"}},
	}, &Watchdog{})
	diagnostics := conductortest.Watch[DiagnosticArray](app, "diagnostics")

	conductortest.Publish(app, "patrol/waypoint", PoseStamped{Pose: Pose{Position: Point{X: 0.1}}})
	publishPose(app, 0, 0)
	conductortest.Publish(app, "cmd_vel", TwistStamped{})
	time.Sleep(2 * time.Millisecond)
	app.Tick("watchdog")

	got, _ := diagnostics.Last()
	if got.Status[0].Level != DiagnosticStatus_OK {
		t.Fatalf("level %d parked on the waypoint (%q), want OK", got.Status[0].Level, got.Status[0].Message)
	}
}

// A watchdog with no pose is stale rather than optimistic: it cannot tell
// whether the robot is moving, and says so.
func TestWatchdogWithoutLocalizationIsStale(t *testing.T) {
	app := conductortest.RunWith(t, conductortest.Options{Frames: mustFrames(t)}, &Watchdog{})
	diagnostics := conductortest.Watch[DiagnosticArray](app, "diagnostics")

	app.Tick("watchdog")
	got, _ := diagnostics.Last()
	if got.Status[0].Level != DiagnosticStatus_STALE {
		t.Fatalf("level %d without a pose (%q), want STALE", got.Status[0].Level, got.Status[0].Message)
	}
}

func mustFrames(t *testing.T) *conductor.FrameTree {
	t.Helper()
	tree, err := conductor.LoadFrames("frames.json")
	if err != nil || tree == nil {
		t.Fatalf("loading frames.json: %v", err)
	}
	return tree
}

func value(s DiagnosticStatus, key string) float64 {
	for _, kv := range s.Values {
		if kv.Key == key {
			var f float64
			if _, err := fmt.Sscanf(kv.Value, "%g", &f); err == nil {
				return f
			}
		}
	}
	return math.NaN()
}

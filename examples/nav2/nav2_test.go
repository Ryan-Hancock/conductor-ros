package main

import (
	"errors"
	"fmt"
	"math"
	"testing"
	"time"

	"conductor.dev/conductor"
	"conductor.dev/conductor/conductortest"
	"conductor.dev/conductor/internal/urdf"
)

// These tests run the commander's mission against a scripted Nav2 — the six
// managed nodes it declares, wired into the app under test, with the action
// servers under the test's control.
//
// The managed nodes are real conductor nodes, so the lifecycle the commander
// drives is the real protocol: they start Unconfigured and only the commander's
// first mission step brings them up. What is faked is navigation, not ROS.
//
// That is the point of driving a stack through declared interfaces rather than a
// behaviour tree XML. "What does this robot do when navigate_to_pose aborts?" is
// a question `go test` answers in milliseconds, with no ROS install, no
// simulator, and no waiting for the fourth waypoint.

// stackNodes are the two Nav2 servers this example actually talks to, plus the
// four it only manages. Their names are what the commander's nodes tag lists.
type MapServer struct{}
type Amcl struct{}
type ControllerServer struct{}
type PlannerServer struct{}

// script is the test's control over what the stack does.
type script struct {
	goals      chan NavigateToPoseGoal // navigate_to_pose goals received
	recoveries chan string             // recovery behaviours run, in order
	outcomes   chan error              // scripted failures, one per goal
}

func newScript() *script {
	return &script{
		goals:      make(chan NavigateToPoseGoal, 16),
		recoveries: make(chan string, 16),
		outcomes:   make(chan error, 16),
	}
}

// BtNavigator serves navigate_to_pose, as Nav2's does.
type BtNavigator struct {
	NavTo  conductor.Action[NavigateToPoseGoal, NavigateToPoseFeedback, NavigateToPoseResult] `action:"navigate_to_pose"`
	script *script
}

// OnNavTo accepts a goal and completes it, unless the test has queued a failure
// for it — which Nav2 reports as an aborted goal carrying an error code, not as
// a transport error.
func (b *BtNavigator) OnNavTo(g *conductor.Goal[NavigateToPoseGoal, NavigateToPoseFeedback]) (NavigateToPoseResult, error) {
	b.script.goals <- g.Value()
	select {
	case err := <-b.script.outcomes:
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

// BehaviorServer serves the two recovery behaviours.
type BehaviorServer struct {
	Back   conductor.Action[BackUpGoal, BackUpFeedback, BackUpResult] `action:"backup"`
	Spin   conductor.Action[SpinGoal, SpinFeedback, SpinResult]       `action:"spin"`
	script *script
}

func (b *BehaviorServer) OnBack(g *conductor.Goal[BackUpGoal, BackUpFeedback]) (BackUpResult, error) {
	b.script.recoveries <- "backup"
	return BackUpResult{}, nil
}

func (b *BehaviorServer) OnSpin(g *conductor.Goal[SpinGoal, SpinFeedback]) (SpinResult, error) {
	b.script.recoveries <- "spin"
	return SpinResult{}, nil
}

// stack is every managed node the commander declares, in any order — bringing
// them up in the declared one is the commander's job.
func stack(s *script) []any {
	return []any{
		&MapServer{}, &Amcl{}, &ControllerServer{}, &PlannerServer{},
		&BehaviorServer{script: s}, &BtNavigator{script: s},
	}
}

// runCommander starts the commander with the stack behind it, nothing brought
// up. The lifecycle is manual for the same reason `autostart:=False` exists:
// bringing the stack up is the application's job, and the test wants to watch
// it happen.
func runCommander(t *testing.T, s *script, params map[string]string) (*conductortest.App, *Commander) {
	t.Helper()
	return runCommanderWith(t, params, stack(s)...)
}

// runCommanderWith is runCommander over an explicit set of managed nodes, so a
// test can leave one out and see what the commander does about it.
func runCommanderWith(t *testing.T, params map[string]string, nodes ...any) (*conductortest.App, *Commander) {
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
	}, append([]any{commander}, nodes...)...)
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
	s := newScript()
	app, commander := runCommander(t, s, nil)
	estimates := conductortest.Watch[PoseWithCovarianceStamped](app, "initialpose")
	waypoints := conductortest.Watch[PoseStamped](app, "patrol/waypoint")

	// Every managed node starts Unconfigured: nothing has brought them up.
	for _, node := range commander.Stack.Nodes() {
		if got := app.State(node); got != conductor.StateUnconfigured {
			t.Fatalf("%s starts %s, want unconfigured", node, got)
		}
	}

	activate(t, app)

	// The mission's first step is the lifecycle_manager this replaces.
	deadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(deadline) {
		if len(commander.Stack.NotActive()) == 0 {
			break
		}
		time.Sleep(2 * time.Millisecond)
	}
	if left := commander.Stack.NotActive(); len(left) != 0 {
		t.Fatalf("%v never reached active", left)
	}

	// The localize step seeds amcl, and it does so before anything is
	// commanded: navigating without a pose is how a robot drives into a wall.
	estimates.Await(t, 2*time.Second)

	goal := await(t, s.goals, "a navigate_to_pose goal")
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

// A managed node that is not there stops the whole bringup, and nothing is
// commanded until it comes back. This is the failure the declared list exists to
// make legible: one name, reported, instead of a stack that is half up.
func TestNothingIsCommandedUntilTheStackIsUp(t *testing.T) {
	s := newScript()
	// Everything except planner_server, which the commander still manages.
	app, commander := runCommanderWith(t, nil,
		&MapServer{}, &Amcl{}, &ControllerServer{},
		&BehaviorServer{script: s}, &BtNavigator{script: s})

	activate(t, app)

	// bring_up declares retry:"3" backoff:"2s", so waiting past the first
	// backoff proves the tag is doing the work a retry loop would do by hand.
	time.Sleep(2500 * time.Millisecond)

	select {
	case goal := <-s.goals:
		t.Fatalf("commanded %v with the stack half up", goal.Pose.Pose.Position)
	default:
	}
	if status, step := app.Mission("commander"); status != conductor.MissionRunning || step != "bring_up" {
		t.Fatalf("mission %s at %q, want it still running bring_up", status, step)
	}
	notActive := commander.Stack.NotActive()
	var named bool
	for _, node := range notActive {
		if node == "planner_server" {
			named = true
		}
	}
	if !named {
		t.Errorf("NotActive = %v, want it to name the missing planner_server", notActive)
	}
}

// An aborted goal takes the fail: branch, which runs Nav2's own recovery
// behaviours and then tries the same waypoint again.
func TestAbortedGoalRunsNav2sRecoveryBehaviours(t *testing.T) {
	s := newScript()
	s.outcomes <- errors.New("controller could not make progress")
	app, commander := runCommander(t, s, nil)

	activate(t, app)

	first := await(t, s.goals, "the first goal")
	if got := await(t, s.recoveries, "the backup recovery"); got != "backup" {
		t.Fatalf("first recovery %q, want backup", got)
	}
	if got := await(t, s.recoveries, "the spin recovery"); got != "spin" {
		t.Fatalf("second recovery %q, want spin", got)
	}

	retried := await(t, s.goals, "the retried goal")
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
	s := newScript()
	app, commander := runCommander(t, s, map[string]string{
		"low_battery":    "0.3",
		"resume_battery": "0.9",
	})

	activate(t, app)
	publishBattery(app, 0.12)

	first := await(t, s.goals, "the first waypoint")
	if first.Pose.Pose.Position != commander.route[0] {
		t.Fatalf("first goal %v, want the first waypoint", first.Pose.Pose.Position)
	}

	dock := await(t, s.goals, "the dock")
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
	resumed := await(t, s.goals, "the route to resume")
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

// The lidar's mounting comes from the robot's own description: frames.json is
// derived from turtlebot3_waffle.urdf, so the number the watchdog reports is the
// one in the URDF — and it resolves even though robot_state_publisher, not this
// application, publishes that transform.
func TestGeometryComesFromTheRobotDescription(t *testing.T) {
	watchdog := &Watchdog{}
	conductortest.RunWith(t, conductortest.Options{Frames: mustFrames(t)}, watchdog)

	// turtlebot3_waffle.urdf: the scan joint is 64mm behind base_link, 122mm up.
	if math.Abs(watchdog.lidarAhead-(-0.064)) > 1e-9 {
		t.Errorf("lidar %v m ahead of base_link, want the URDF's -0.064", watchdog.lidarAhead)
	}
	if math.Abs(watchdog.lidarAbove-0.122) > 1e-9 {
		t.Errorf("lidar %v m above base_link, want the URDF's 0.122", watchdog.lidarAbove)
	}

	// And nothing here publishes it: that is robot_state_publisher's job, and
	// publishing it too would put two static transforms on one child.
	tree := mustFrames(t)
	if got := len(tree.Published()); got != 0 {
		t.Errorf("%d transform(s) would be published by this application, want none", got)
	}
	if got := len(tree.Fixed()); got != 10 {
		t.Errorf("%d fixed transforms, want the URDF's 10", got)
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

// frames.json is derived from turtlebot3_waffle.urdf — the description the
// simulation and the real robot are both launched with — so it can drift from
// it. This is the drift check, and it is also where the two halves of the tree
// show: the robot's geometry comes from the URDF, and the world links into it
// (map -> odom, odom -> base_footprint) come from amcl and the odometry, which
// no description mentions.
func TestCommittedFramesMatchTheDescription(t *testing.T) {
	robot, err := urdf.Load("turtlebot3_waffle.urdf")
	if err != nil {
		t.Fatal(err)
	}
	derived, _ := urdf.Frames(robot, urdf.Options{})

	committed := mustFrames(t)
	byChild := map[string]conductor.Transform{}
	for _, tf := range committed.Transforms {
		byChild[tf.Child] = tf
	}
	for _, want := range derived.Transforms {
		got, ok := byChild[want.Child]
		if !ok {
			t.Errorf("frames.json is missing %s; re-derive it with conductor frames", want)
			continue
		}
		if got != want {
			t.Errorf("%s: frames.json says %+v, the description says %+v; re-derive it",
				want.Child, got, want)
		}
	}

	// Nothing the robot_state_publisher owns is claimed by this application.
	for _, tf := range committed.Transforms {
		if tf.Ours() {
			t.Errorf("%s is claimed as ours, but this robot has a robot_state_publisher", tf)
		}
	}
	// The world links are not in the URDF and must survive a re-derivation.
	for _, child := range []string{"odom", "base_footprint"} {
		if tf, ok := byChild[child]; !ok || !tf.Dynamic {
			t.Errorf("frames.json lost the dynamic transform into %q", child)
		}
	}
}

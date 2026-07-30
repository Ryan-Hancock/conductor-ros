package main

import (
	"math"
	"testing"
	"time"

	"conductor.dev/conductor"
	"conductor.dev/conductor/conductortest"
	"conductor.dev/conductor/internal/urdf"
	"conductor.dev/conductor/msgs"
	"conductor.dev/conductor/srvs"
)

// runPatrol wires nodes with the application's real transform tree, the way
// the runtime loads it from frames.json at startup. The safety monitor reads
// the lidar's mounting out of it while configuring, so a test without it
// would not be running the same application.
func runPatrol(t *testing.T, nodes ...any) *conductortest.App {
	t.Helper()
	tree, err := conductor.LoadFrames("frames.json")
	if err != nil || tree == nil {
		t.Fatalf("loading frames.json: %v", err)
	}
	return conductortest.RunWith(t, conductortest.Options{Frames: tree}, nodes...)
}

func poseAt(x, y float64) msgs.PoseStamped {
	return msgs.PoseStamped{
		Header: msgs.Header{Stamp: time.Now(), FrameID: "map"},
		Pose:   msgs.Pose{Position: msgs.Point{X: x, Y: y}},
	}
}

func speed(c msgs.Twist) float64 { return math.Hypot(c.Linear.X, c.Linear.Y) }

// The navigator steers toward its goal and never exceeds max_speed.
func TestNavigatorSteersTowardGoal(t *testing.T) {
	app := runPatrol(t, &Navigator{goal: msgs.Point{X: 5, Y: 4}})
	cmd := conductortest.Watch[msgs.Twist](app, "cmd_vel")

	conductortest.Publish(app, "amcl_pose", poseAt(0, 0))

	got, ok := cmd.Last()
	if !ok {
		t.Fatal("navigator published no command")
	}
	if got.Linear.X <= 0 || got.Linear.Y <= 0 {
		t.Fatalf("command %v does not point toward the goal", got.Linear)
	}
	if s := speed(got); math.Abs(s-1.5) > 1e-9 {
		t.Fatalf("speed %v, want the default max_speed 1.5", s)
	}
}

// Arriving at the goal stops the robot rather than overshooting it.
func TestNavigatorStopsAtGoal(t *testing.T) {
	app := runPatrol(t, &Navigator{goal: msgs.Point{X: 5, Y: 4}})
	cmd := conductortest.Watch[msgs.Twist](app, "cmd_vel")

	conductortest.Publish(app, "amcl_pose", poseAt(5, 4))

	if got, _ := cmd.Last(); speed(got) != 0 {
		t.Fatalf("speed %v at the goal, want 0", speed(got))
	}
}

// max_speed is the same knob whether it arrives from a parameter file at
// startup or from `ros2 param set` at runtime.
func TestMaxSpeedParameter(t *testing.T) {
	tree, err := conductor.LoadFrames("frames.json")
	if err != nil {
		t.Fatal(err)
	}
	app := conductortest.RunWith(t, conductortest.Options{
		Params: map[string]map[string]string{"navigator": {"max_speed": "0.25"}},
		Frames: tree,
	}, &Navigator{goal: msgs.Point{X: 50, Y: 40}})
	cmd := conductortest.Watch[msgs.Twist](app, "cmd_vel")

	conductortest.Publish(app, "amcl_pose", poseAt(0, 0))
	if got, _ := cmd.Last(); math.Abs(speed(got)-0.25) > 1e-9 {
		t.Fatalf("speed %v, want the file's 0.25", speed(got))
	}

	app.SetParam("navigator", "max_speed", "0.75")
	conductortest.Publish(app, "amcl_pose", poseAt(0, 0))
	if got, _ := cmd.Last(); math.Abs(speed(got)-0.75) > 1e-9 {
		t.Fatalf("speed %v, want 0.75 after the update", speed(got))
	}
}

// The e-stop service changes what the monitor reports on its next watchdog.
func TestEstopServiceChangesStatus(t *testing.T) {
	app := runPatrol(t, &SafetyMonitor{})
	status := conductortest.Watch[PatrolStatus](app, "patrol_status")

	app.Tick("safety_monitor")
	if got, _ := status.Last(); got.Mode != PatrolStatus_MODE_PATROLLING {
		t.Fatalf("mode %d, want patrolling", got.Mode)
	}

	res, err := conductortest.Call[srvs.SetBoolRequest, srvs.SetBoolResponse](
		app, "engage_estop", srvs.SetBoolRequest{Data: true})
	if err != nil {
		t.Fatal(err)
	}
	if !res.Success {
		t.Fatalf("service reported failure: %q", res.Message)
	}

	app.Tick("safety_monitor")
	if got, _ := status.Last(); got.Mode != PatrolStatus_MODE_ESTOPPED {
		t.Fatalf("mode %d after e-stop, want estopped", got.Mode)
	}
}

// The e-stop topic and the e-stop service must agree.
func TestEstopTopic(t *testing.T) {
	app := runPatrol(t, &SafetyMonitor{})
	status := conductortest.Watch[PatrolStatus](app, "patrol_status")

	conductortest.Publish(app, "estop", msgs.Bool{Data: true})
	app.Tick("safety_monitor")

	if got, _ := status.Last(); got.Mode != PatrolStatus_MODE_ESTOPPED {
		t.Fatalf("mode %d, want estopped", got.Mode)
	}
}

// The whole app wired together: the localizer's pose drives the navigator,
// whose command reaches the safety monitor — one tick, three nodes.
func TestFullGraph(t *testing.T) {
	app := runPatrol(t,
		&Localizer{},
		&Navigator{goal: msgs.Point{X: 5, Y: 4}},
		&SafetyMonitor{},
	)
	cmd := conductortest.Watch[msgs.Twist](app, "cmd_vel")
	status := conductortest.Watch[PatrolStatus](app, "patrol_status")

	app.Tick("localizer") // one pose, through the navigator, to the monitor
	if cmd.Len() != 1 {
		t.Fatalf("recorded %d commands, want 1", cmd.Len())
	}

	app.Tick("safety_monitor")
	got, ok := status.Last()
	if !ok {
		t.Fatal("no status published")
	}
	if got.CmdStale {
		t.Fatal("cmd_vel reported stale immediately after a command")
	}
}

// A deactivated node is silent, which is the whole point of the lifecycle.
func TestDeactivatedNavigatorIsSilent(t *testing.T) {
	app := runPatrol(t, &Navigator{goal: msgs.Point{X: 5, Y: 4}})
	cmd := conductortest.Watch[msgs.Twist](app, "cmd_vel")

	app.Transition("navigator", conductor.TransitionDeactivate)
	conductortest.Publish(app, "amcl_pose", poseAt(0, 0))
	if cmd.Len() != 0 {
		t.Fatalf("deactivated navigator published %d commands", cmd.Len())
	}

	app.Transition("navigator", conductor.TransitionActivate)
	conductortest.Publish(app, "amcl_pose", poseAt(0, 0))
	if cmd.Len() != 1 {
		t.Fatalf("reactivated navigator published %d commands, want 1", cmd.Len())
	}
}

// The frame tag is the single place the frame is written: the publisher
// stamps it, and the transform tree is what the monitor measures the robot
// with.
func TestFramesAreDeclaredOnce(t *testing.T) {
	app := runPatrol(t, &Localizer{}, &SafetyMonitor{})
	pose := conductortest.Watch[msgs.PoseStamped](app, "amcl_pose")
	status := conductortest.Watch[PatrolStatus](app, "patrol_status")

	app.Tick("localizer")
	got, ok := pose.Last()
	if !ok {
		t.Fatal("no pose published")
	}
	if got.Header.FrameID != "map" {
		t.Fatalf("pose stamped %q, want the declared frame \"map\"", got.Header.FrameID)
	}
	if got.Header.Stamp.IsZero() {
		t.Fatal("pose has no timestamp; the frame tag should stamp one")
	}

	app.Tick("safety_monitor")
	if s, _ := status.Last(); s.Header.FrameId != "base_link" {
		t.Fatalf("status stamped %q, want \"base_link\"", s.Header.FrameId)
	}
}

// The lidar's pose comes from frames.json, composed by the runtime — the same
// lookup conductor check resolves at build time.
func TestLidarOffsetComesFromTheTransformTree(t *testing.T) {
	monitor := &SafetyMonitor{}
	runPatrol(t, monitor)
	if math.Abs(monitor.laserAhead-0.12) > 1e-9 {
		t.Fatalf("lidar %v m ahead of base_link, want 0.12 from frames.json", monitor.laserAhead)
	}
}

// The route is a mission: activating the patroller drives it to the first
// waypoint, dwelling moves it to the next, and an e-stop parks it in holding
// until the stop clears.
func TestPatrolRouteIsAMission(t *testing.T) {
	tree, err := conductor.LoadFrames("frames.json")
	if err != nil {
		t.Fatal(err)
	}
	app := conductortest.RunWith(t, conductortest.Options{
		ManualLifecycle: true,
		Frames:          tree,
		// A short dwell keeps the test quick; the route's timing is a
		// parameter like any other.
		Params: map[string]map[string]string{"patroller": {"dwell": "10ms"}},
	}, &Patroller{})
	goals := conductortest.Watch[msgs.PoseStamped](app, "goal_pose")

	app.Transition("patroller", conductor.TransitionConfigure)
	app.Transition("patroller", conductor.TransitionActivate)

	deadline := time.Now().Add(2 * time.Second)
	for goals.Len() < 3 && time.Now().Before(deadline) {
		time.Sleep(time.Millisecond)
	}
	if goals.Len() < 3 {
		t.Fatalf("the route published %d waypoints in 2s", goals.Len())
	}
	if got, _ := goals.Last(); got.Header.FrameID != "map" {
		t.Fatalf("waypoint stamped %q, want the declared map frame", got.Header.FrameID)
	}

	// An e-stop sends the next transition into holding rather than onward.
	conductortest.Publish(app, "estop", msgs.Bool{Data: true})
	deadline = time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		if _, step := app.Mission("patroller"); step == "holding" {
			break
		}
		time.Sleep(time.Millisecond)
	}
	if _, step := app.Mission("patroller"); step != "holding" {
		t.Fatalf("mission at %q after an e-stop, want holding", step)
	}

	// Deactivating stops the mission, which is what a lifecycle is for.
	app.Transition("patroller", conductor.TransitionDeactivate)
	if status, _ := app.Mission("patroller"); status != conductor.MissionCanceled {
		t.Fatalf("mission %s after deactivate, want canceled", status)
	}
}

// frames.json is derived from patrol.urdf, so it can drift from the robot it
// describes — the same way conductor.json could drift from a live graph before
// `conductor externals -check`. This is that check, for the description:
//
//	conductor frames -from examples/patrol/patrol.urdf \
//	    -o examples/patrol/frames.json -publish -fixed-only
//
// The world links (map -> odom, odom -> base_link) are not in any URDF and are
// preserved by the derivation, so they are compared separately.
func TestCommittedFramesMatchTheDescription(t *testing.T) {
	robot, err := urdf.Load("patrol.urdf")
	if err != nil {
		t.Fatal(err)
	}
	derived, _ := urdf.Frames(robot, urdf.Options{Ours: true, FixedOnly: true})

	committed, err := conductor.LoadFrames("frames.json")
	if err != nil || committed == nil {
		t.Fatalf("loading frames.json: %v", err)
	}

	// Everything the description produces must match exactly: this robot has no
	// robot_state_publisher, so conductor publishes its geometry, and these are
	// the numbers that go on tf_static.
	want := map[string]conductor.Transform{}
	for _, tf := range derived.Transforms {
		want[tf.Child] = tf
	}
	got := map[string]conductor.Transform{}
	for _, tf := range committed.Transforms {
		if _, fromURDF := want[tf.Child]; fromURDF {
			got[tf.Child] = tf
		}
	}
	if len(got) != len(want) {
		t.Fatalf("frames.json has %d of the description's %d transforms; re-derive it",
			len(got), len(want))
	}
	for child, w := range want {
		if got[child] != w {
			t.Errorf("%s: frames.json says %+v, patrol.urdf says %+v; re-derive it", child, got[child], w)
		}
	}

	// And the world links survive, because nothing in a URDF describes them.
	for _, child := range []string{"odom", "base_link"} {
		var found bool
		for _, tf := range committed.Transforms {
			if tf.Child == child && tf.Dynamic {
				found = true
			}
		}
		if !found {
			t.Errorf("frames.json lost the dynamic transform into %q", child)
		}
	}
}

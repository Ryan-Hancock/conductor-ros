package main

import (
	"math"
	"testing"
	"time"

	"conductor.dev/conductor"
	"conductor.dev/conductor/conductortest"
	"conductor.dev/conductor/msgs"
	"conductor.dev/conductor/srvs"
)

func poseAt(x, y float64) msgs.PoseStamped {
	return msgs.PoseStamped{
		Header: msgs.Header{Stamp: time.Now(), FrameID: "map"},
		Pose:   msgs.Pose{Position: msgs.Point{X: x, Y: y}},
	}
}

func speed(c msgs.Twist) float64 { return math.Hypot(c.Linear.X, c.Linear.Y) }

// The navigator steers toward its goal and never exceeds max_speed.
func TestNavigatorSteersTowardGoal(t *testing.T) {
	app := conductortest.Run(t, &Navigator{goal: msgs.Point{X: 5, Y: 4}})
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
	app := conductortest.Run(t, &Navigator{goal: msgs.Point{X: 5, Y: 4}})
	cmd := conductortest.Watch[msgs.Twist](app, "cmd_vel")

	conductortest.Publish(app, "amcl_pose", poseAt(5, 4))

	if got, _ := cmd.Last(); speed(got) != 0 {
		t.Fatalf("speed %v at the goal, want 0", speed(got))
	}
}

// max_speed is the same knob whether it arrives from a parameter file at
// startup or from `ros2 param set` at runtime.
func TestMaxSpeedParameter(t *testing.T) {
	app := conductortest.RunWith(t, conductortest.Options{
		Params: map[string]map[string]string{"navigator": {"max_speed": "0.25"}},
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
	app := conductortest.Run(t, &SafetyMonitor{})
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
	app := conductortest.Run(t, &SafetyMonitor{})
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
	app := conductortest.Run(t,
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
	app := conductortest.Run(t, &Navigator{goal: msgs.Point{X: 5, Y: 4}})
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

// Nav2stub stands in for a Nav2 bringup so that examples/nav2 can be run and
// tested without a 2 GB navigation stack installed. It is deliberately not a
// navigation stack: there is no planner, no costmap and no controller here.
// What it has is Nav2's *interfaces* — navigate_to_pose, backup and spin, the
// lifecycle manager's manage_nodes service, amcl_pose, cmd_vel and a battery
// — by the same names, types and RIHS01 type hashes as the real thing.
//
// That is the point: examples/nav2 cannot tell the difference, and neither
// can `ros2 action list`. Swap this for a real nav2_bringup (`conductor run
// examples/nav2 -env sim`) and the application is unchanged.
//
// It answers honestly rather than always succeeding — a navigation stack that
// never fails would leave the commander's recovery branch untested, so every
// fail_every'th goal is aborted with a Nav2 error code.
//
//	go run -tags zenoh ./examples/nav2stub -transport zenoh
package main

import (
	"context"
	"errors"
	"log/slog"
	"math"
	"sync"
	"time"

	"conductor.dev/conductor"
	_ "conductor.dev/conductor/transport/zenoh"
)

// The two ways a goal fails here. Nav2 reports failure twice over — an
// aborted goal status and an error_code in the result — and so does this.
var (
	errNotActive  = errors.New("navigation servers are not active")
	errNoProgress = errors.New("controller could not make progress")
)

// Nav2 is one node pretending to be several: bt_navigator, the behaviour
// server, amcl, the controller and a battery driver.
//
//conductor:node
type Nav2 struct {
	Manage conductor.Svc[ManageLifecycleNodesRequest, ManageLifecycleNodesResponse] `service:"lifecycle_manager_navigation/manage_nodes"`

	NavTo conductor.Action[NavigateToPoseGoal, NavigateToPoseFeedback, NavigateToPoseResult] `action:"navigate_to_pose"`
	Back  conductor.Action[BackUpGoal, BackUpFeedback, BackUpResult]                         `action:"backup"`
	Spin  conductor.Action[SpinGoal, SpinFeedback, SpinResult]                               `action:"spin"`

	Initial conductor.Sub[PoseWithCovarianceStamped] `topic:"initialpose" qos:"reliable" frame:"map"`
	Pose    conductor.Pub[PoseWithCovarianceStamped] `topic:"amcl_pose" qos:"transient" frame:"map"`
	Cmd     conductor.Pub[TwistStamped]              `topic:"cmd_vel" qos:"reliable" frame:"base_link"`
	Battery conductor.Pub[BatteryState]              `topic:"battery_state" qos:"sensor"`

	Beat conductor.Timer `rate:"10hz"`

	Speed     conductor.Param[float64] `name:"speed" default:"0.45"`
	Drain     conductor.Param[float64] `name:"drain_per_second" default:"0.03"`
	Charging  conductor.Param[float64] `name:"charge_per_second" default:"0.12"`
	DockRange conductor.Param[float64] `name:"charge_radius" default:"0.8"`
	FailEvery conductor.Param[int]     `name:"fail_every" default:"4"`

	// Action handlers run one goroutine per goal, off the executor, so the
	// simulated robot they steer is shared state and says so. A real driver
	// has the same problem and answers it the same way.
	mu      sync.Mutex
	started bool
	at      Point
	heading float64
	charge  float64
	driving bool
	goals   int
}

func (n *Nav2) OnConfigure() error {
	n.charge = 1.0
	return nil
}

// OnManage is the lifecycle manager's service. Before STARTUP the navigation
// servers refuse to work, which is the behaviour that makes the commander's
// first mission step worth having.
func (n *Nav2) OnManage(req ManageLifecycleNodesRequest) (ManageLifecycleNodesResponse, error) {
	n.mu.Lock()
	defer n.mu.Unlock()
	switch req.Command {
	case ManageLifecycleNodes_Request_STARTUP, ManageLifecycleNodes_Request_RESUME:
		n.started = true
	case ManageLifecycleNodes_Request_PAUSE, ManageLifecycleNodes_Request_RESET,
		ManageLifecycleNodes_Request_SHUTDOWN:
		n.started = false
	}
	slog.Info("manage_nodes", "command", req.Command, "active", n.started)
	return ManageLifecycleNodesResponse{Success: true}, nil
}

// OnInitial accepts an initial pose estimate, as amcl does.
func (n *Nav2) OnInitial(p PoseWithCovarianceStamped) {
	n.mu.Lock()
	defer n.mu.Unlock()
	n.at = p.Pose.Pose.Position
	slog.Info("initial pose accepted", "x", n.at.X, "y", n.at.Y)
}

// OnBeat publishes what a running stack publishes: the localized pose, the
// controller's velocity command, and the battery.
func (n *Nav2) OnBeat() {
	n.mu.Lock()
	at, heading, driving := n.at, n.heading, n.driving
	docked := math.Hypot(at.X, at.Y) < n.DockRange.Get()
	switch {
	case driving:
		n.charge = math.Max(0, n.charge-n.Drain.Get()/10)
	case docked:
		n.charge = math.Min(1, n.charge+n.Charging.Get()/10)
	}
	charge := n.charge
	n.mu.Unlock()

	n.Pose.Publish(PoseWithCovarianceStamped{
		Pose: PoseWithCovariance{Pose: Pose{Position: at, Orientation: yawQuaternion(heading)}},
	})
	// The controller publishes a command every cycle, zero included: that is
	// what a stalled stack looks like to anything watching, the watchdog
	// included.
	cmd := TwistStamped{}
	if driving {
		cmd.Twist.Linear.X = n.Speed.Get()
	}
	n.Cmd.Publish(cmd)
	n.Battery.Publish(BatteryState{
		Percentage:        float32(charge),
		Voltage:           float32(11.1 + charge),
		Present:           true,
		PowerSupplyStatus: batteryStatus(driving, docked),
	})
}

// OnNavTo walks the robot toward the goal in a straight line, reporting
// distance remaining as feedback. Every fail_every'th goal is aborted, so the
// commander's recovery branch is exercised rather than merely declared.
func (n *Nav2) OnNavTo(g *conductor.Goal[NavigateToPoseGoal, NavigateToPoseFeedback]) (NavigateToPoseResult, error) {
	target := g.Value().Pose.Pose.Position

	n.mu.Lock()
	if !n.started {
		n.mu.Unlock()
		slog.Warn("goal refused: the stack is not active")
		return NavigateToPoseResult{
			ErrorCode: NavigateToPose_Result_UNKNOWN,
			ErrorMsg:  "lifecycle: " + errNotActive.Error(),
		}, errNotActive
	}
	n.goals++
	fail := n.FailEvery.Get() > 0 && n.goals%n.FailEvery.Get() == 0
	n.driving = true
	n.mu.Unlock()
	defer n.park()

	started := time.Now()
	for {
		select {
		case <-g.Context().Done():
			return NavigateToPoseResult{}, g.Context().Err()
		case <-time.After(100 * time.Millisecond):
		}

		at, remaining := n.advance(target)

		// A goal picked to fail gives up part way, the way a controller does
		// when it cannot get around something.
		if fail && remaining < 0.75 {
			slog.Warn("aborting the goal", "goal", n.goals, "remaining", round(remaining))
			return NavigateToPoseResult{
				ErrorCode: NavigateToPose_Result_UNKNOWN,
				ErrorMsg:  "simulated: " + errNoProgress.Error(),
			}, errNoProgress
		}
		if remaining == 0 {
			slog.Info("goal reached", "x", target.X, "y", target.Y, "took", time.Since(started).Round(time.Millisecond))
			return NavigateToPoseResult{}, nil
		}
		g.Feedback(NavigateToPoseFeedback{
			CurrentPose:            PoseStamped{Pose: Pose{Position: at}},
			NavigationTime:         time.Since(started),
			EstimatedTimeRemaining: time.Duration(remaining / n.Speed.Get() * float64(time.Second)),
			DistanceRemaining:      float32(remaining),
		})
	}
}

// OnBack is the behaviour server's backup recovery.
func (n *Nav2) OnBack(g *conductor.Goal[BackUpGoal, BackUpFeedback]) (BackUpResult, error) {
	distance := math.Abs(g.Value().Target.X)
	slog.Info("backing up", "distance", distance)
	if err := n.creep(g.Context(), -distance, func(done float64) {
		g.Feedback(BackUpFeedback{DistanceTraveled: float32(done)})
	}); err != nil {
		return BackUpResult{ErrorCode: BackUp_Result_TIMEOUT, ErrorMsg: err.Error()}, err
	}
	return BackUpResult{}, nil
}

// OnSpin is the behaviour server's spin recovery. Turning in place moves
// nothing, so it costs only time and the heading.
func (n *Nav2) OnSpin(g *conductor.Goal[SpinGoal, SpinFeedback]) (SpinResult, error) {
	yaw := float64(g.Value().TargetYaw)
	slog.Info("spinning", "target_yaw", round(yaw))
	for done := 0.0; done < math.Abs(yaw); done += 0.5 {
		select {
		case <-g.Context().Done():
			return SpinResult{ErrorCode: Spin_Result_TIMEOUT}, g.Context().Err()
		case <-time.After(100 * time.Millisecond):
		}
		g.Feedback(SpinFeedback{AngularDistanceTraveled: float32(done)})
	}
	n.mu.Lock()
	n.heading += yaw
	n.mu.Unlock()
	return SpinResult{}, nil
}

// advance moves the robot one tenth of a second toward target, returning
// where it now is and how far it still has to go.
func (n *Nav2) advance(target Point) (Point, float64) {
	n.mu.Lock()
	defer n.mu.Unlock()

	dx, dy := target.X-n.at.X, target.Y-n.at.Y
	remaining := math.Hypot(dx, dy)
	step := n.Speed.Get() / 10
	if remaining <= step {
		n.at = target
		return n.at, 0
	}
	n.at.X += step * dx / remaining
	n.at.Y += step * dy / remaining
	n.heading = math.Atan2(dy, dx)
	return n.at, remaining - step
}

// creep moves the robot along its current heading by distance, in small
// steps, reporting progress as it goes.
func (n *Nav2) creep(ctx context.Context, distance float64, report func(float64)) error {
	n.mu.Lock()
	heading := n.heading
	n.driving = true
	n.mu.Unlock()
	defer n.park()

	const step = 0.05
	for done := step; done <= math.Abs(distance); done += step {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-time.After(100 * time.Millisecond):
		}
		n.mu.Lock()
		n.at.X += math.Copysign(step, distance) * math.Cos(heading)
		n.at.Y += math.Copysign(step, distance) * math.Sin(heading)
		n.mu.Unlock()
		report(done)
	}
	return nil
}

func (n *Nav2) park() {
	n.mu.Lock()
	n.driving = false
	n.mu.Unlock()
}

func batteryStatus(driving, docked bool) uint8 {
	switch {
	case driving:
		return BatteryState_POWER_SUPPLY_STATUS_DISCHARGING
	case docked:
		return BatteryState_POWER_SUPPLY_STATUS_CHARGING
	default:
		return BatteryState_POWER_SUPPLY_STATUS_NOT_CHARGING
	}
}

func yawQuaternion(yaw float64) Quaternion {
	return Quaternion{Z: math.Sin(yaw / 2), W: math.Cos(yaw / 2)}
}

func round(v float64) float64 { return math.Round(v*100) / 100 }

func main() {
	conductor.Run(&Nav2{})
}

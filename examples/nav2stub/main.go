// Nav2stub stands in for a Nav2 bringup so that examples/nav2 can be run and
// tested without a 2 GB navigation stack installed. It is deliberately not a
// navigation stack: there is no planner, no costmap and no controller here.
// What it has is Nav2's *shape* — the same node names, the same interfaces on
// the same nodes, the same types and RIHS01 type hashes, and the same
// managed-node lifecycle.
//
// That last part is why the node split matters. Nav2 is six managed nodes that
// something has to configure and activate in order, and examples/nav2 does that
// itself with a conductor.Lifecycle field. So the stand-in has to be six nodes
// with those names, not one pretending: the list the commander declares is then
// the same list against a real bringup.
//
//	go run -tags zenoh ./examples/nav2stub -transport zenoh -lifecycle manual
//
// It runs with the lifecycle manual, so its nodes start Unconfigured and stay
// there until the commander brings them up — which is what `autostart:=False`
// does to a real Nav2. And it answers honestly rather than always succeeding:
// every fail_every'th goal is aborted with a Nav2 error code, because a
// stand-in that never fails would leave the recovery branch untested.
package main

import (
	"context"
	"errors"
	"log/slog"
	"math"
	"sync"
	"sync/atomic"
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

// robot is the simulated vehicle the nodes below share. Real Nav2's nodes share
// a robot too; they just do it through hardware and TF. Action handlers run off
// the executor, one goroutine per goal, so this is locked.
type robot struct {
	mu      sync.Mutex
	at      Point
	heading float64
	driving bool
	goals   int
}

// place sets where the robot is, as accepting an initial pose estimate does.
func (r *robot) place(at Point) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.at = at
}

// pose is where the robot is and which way it faces.
func (r *robot) pose() (Point, float64) {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.at, r.heading
}

func (r *robot) moving() bool {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.driving
}

// start claims the robot for a goal, reporting whether this goal is one of the
// ones chosen to fail.
func (r *robot) start(failEvery int) (fail bool) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.goals++
	r.driving = true
	return failEvery > 0 && r.goals%failEvery == 0
}

func (r *robot) park() {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.driving = false
}

// advance moves the robot one step toward target, returning where it now is and
// how far it still has to go.
func (r *robot) advance(target Point, step float64) (Point, float64) {
	r.mu.Lock()
	defer r.mu.Unlock()

	dx, dy := target.X-r.at.X, target.Y-r.at.Y
	remaining := math.Hypot(dx, dy)
	if remaining <= step {
		r.at = target
		return r.at, 0
	}
	r.at.X += step * dx / remaining
	r.at.Y += step * dy / remaining
	r.heading = math.Atan2(dy, dx)
	return r.at, remaining - step
}

// creep moves the robot along its current heading, which is what a recovery
// behaviour does.
func (r *robot) creep(ctx context.Context, distance float64, report func(float64)) error {
	_, heading := r.pose()
	r.mu.Lock()
	r.driving = true
	r.mu.Unlock()
	defer r.park()

	const step = 0.05
	for done := step; done <= math.Abs(distance); done += step {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-time.After(100 * time.Millisecond):
		}
		r.mu.Lock()
		r.at.X += math.Copysign(step, distance) * math.Cos(heading)
		r.at.Y += math.Copysign(step, distance) * math.Sin(heading)
		r.mu.Unlock()
		report(done)
	}
	return nil
}

func (r *robot) turn(yaw float64) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.heading += yaw
}

// gate is the "am I active?" flag every managed node needs. Conductor gates
// publishers, subscriptions and timers on Active by itself; services and action
// servers are the author's to gate, exactly as they are in rclcpp — which is
// why Nav2's servers do this too.
//
// It is atomic because the lifecycle hooks run on the node's executor and the
// action handlers that read it do not.
type gate struct{ live atomic.Bool }

func (g *gate) OnActivate() error   { g.live.Store(true); return nil }
func (g *gate) OnDeactivate() error { g.live.Store(false); return nil }
func (g *gate) active() bool        { return g.live.Load() }

// MapServer stands in for the node that serves the map. This example never
// asks for one, so it has no endpoints at all — it is here because a managed
// stack contains nodes you do not talk to, and the lifecycle list has to be
// able to hold them.
//
//conductor:node
type MapServer struct{ gate }

// Amcl publishes where the robot thinks it is and accepts a pose estimate,
// which is the whole of localization as far as this example is concerned.
//
//conductor:node
type Amcl struct {
	gate
	Initial conductor.Sub[PoseWithCovarianceStamped] `topic:"initialpose" qos:"reliable" frame:"map"`
	Pose    conductor.Pub[PoseWithCovarianceStamped] `topic:"amcl_pose" qos:"transient" frame:"map"`
	Beat    conductor.Timer                          `rate:"10hz"`

	robot *robot
}

func (a *Amcl) OnInitial(p PoseWithCovarianceStamped) {
	a.robot.place(p.Pose.Pose.Position)
	at, _ := a.robot.pose()
	slog.Info("initial pose accepted", "x", at.X, "y", at.Y)
}

func (a *Amcl) OnBeat() {
	at, heading := a.robot.pose()
	a.Pose.Publish(PoseWithCovarianceStamped{
		Pose: PoseWithCovariance{Pose: Pose{Position: at, Orientation: yawQuaternion(heading)}},
	})
}

// ControllerServer publishes the velocity command. Every cycle, zero included:
// a controller holding still is what a stalled stack looks like from outside,
// and the example's watchdog is watching for exactly that.
//
//conductor:node
type ControllerServer struct {
	gate
	Cmd   conductor.Pub[TwistStamped] `topic:"cmd_vel" qos:"reliable" frame:"base_link"`
	Beat  conductor.Timer             `rate:"10hz"`
	Speed conductor.Param[float64]    `name:"speed" default:"0.45"`

	robot *robot
}

func (c *ControllerServer) OnBeat() {
	cmd := TwistStamped{}
	if c.robot.moving() {
		cmd.Twist.Linear.X = c.Speed.Get()
	}
	c.Cmd.Publish(cmd)
}

// PlannerServer stands in for the planner. Like the map server it has no
// endpoints this example uses: the commander asks bt_navigator to navigate, and
// what the planner is asked for happens inside the stack.
//
//conductor:node
type PlannerServer struct{ gate }

// BtNavigator serves navigate_to_pose, as Nav2's does.
//
//conductor:node
type BtNavigator struct {
	gate
	NavTo conductor.Action[NavigateToPoseGoal, NavigateToPoseFeedback, NavigateToPoseResult] `action:"navigate_to_pose"`

	Speed     conductor.Param[float64] `name:"speed" default:"0.45"`
	FailEvery conductor.Param[int]     `name:"fail_every" default:"4"`

	robot *robot
}

// OnNavTo walks the robot toward the goal in a straight line, reporting
// distance remaining as feedback.
func (b *BtNavigator) OnNavTo(g *conductor.Goal[NavigateToPoseGoal, NavigateToPoseFeedback]) (NavigateToPoseResult, error) {
	if !b.active() {
		slog.Warn("goal refused: bt_navigator is not active")
		return NavigateToPoseResult{
			ErrorCode: NavigateToPose_Result_UNKNOWN,
			ErrorMsg:  "lifecycle: " + errNotActive.Error(),
		}, errNotActive
	}

	target := g.Value().Pose.Pose.Position
	fail := b.robot.start(b.FailEvery.Get())
	defer b.robot.park()

	started := time.Now()
	for {
		select {
		case <-g.Context().Done():
			return NavigateToPoseResult{}, g.Context().Err()
		case <-time.After(100 * time.Millisecond):
		}

		at, remaining := b.robot.advance(target, b.Speed.Get()/10)

		// A goal picked to fail gives up part way, the way a controller does
		// when it cannot get around something.
		if fail && remaining < 0.75 {
			slog.Warn("aborting the goal", "remaining", round(remaining))
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
			EstimatedTimeRemaining: time.Duration(remaining / b.Speed.Get() * float64(time.Second)),
			DistanceRemaining:      float32(remaining),
		})
	}
}

// BehaviorServer serves the two recovery behaviours the commander falls back
// on, as Nav2's behavior_server does.
//
//conductor:node
type BehaviorServer struct {
	gate
	Back conductor.Action[BackUpGoal, BackUpFeedback, BackUpResult] `action:"backup"`
	Spin conductor.Action[SpinGoal, SpinFeedback, SpinResult]       `action:"spin"`

	robot *robot
}

func (b *BehaviorServer) OnBack(g *conductor.Goal[BackUpGoal, BackUpFeedback]) (BackUpResult, error) {
	if !b.active() {
		return BackUpResult{ErrorCode: BackUp_Result_UNKNOWN, ErrorMsg: errNotActive.Error()}, errNotActive
	}
	distance := math.Abs(g.Value().Target.X)
	slog.Info("backing up", "distance", distance)
	if err := b.robot.creep(g.Context(), -distance, func(done float64) {
		g.Feedback(BackUpFeedback{DistanceTraveled: float32(done)})
	}); err != nil {
		return BackUpResult{ErrorCode: BackUp_Result_TIMEOUT, ErrorMsg: err.Error()}, err
	}
	return BackUpResult{}, nil
}

// OnSpin turns in place, so it costs only time and the heading.
func (b *BehaviorServer) OnSpin(g *conductor.Goal[SpinGoal, SpinFeedback]) (SpinResult, error) {
	if !b.active() {
		return SpinResult{ErrorCode: Spin_Result_UNKNOWN, ErrorMsg: errNotActive.Error()}, errNotActive
	}
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
	b.robot.turn(yaw)
	return SpinResult{}, nil
}

func yawQuaternion(yaw float64) Quaternion {
	return Quaternion{Z: math.Sin(yaw / 2), W: math.Cos(yaw / 2)}
}

func round(v float64) float64 { return math.Round(v*100) / 100 }

func main() {
	// One simulated robot, six nodes sharing it — the same division of labour
	// as the stack this stands in for.
	r := &robot{}
	conductor.Run(
		&MapServer{},
		&Amcl{robot: r},
		&ControllerServer{robot: r},
		&PlannerServer{},
		&BehaviorServer{robot: r},
		&BtNavigator{robot: r},
	)
}

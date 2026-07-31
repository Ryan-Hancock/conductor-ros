// Moveitstub stands in for a MoveIt move_group so that examples/moveit can be
// run and tested without a MoveIt install. It plans nothing: there is no
// kinematics, no collision checking and no controller here.
//
// What it has is move_group's interface — the move_action server carrying
// moveit_msgs/action/MoveGroup, the largest nested message in common ROS use —
// answered with the same error codes and a trajectory of the right shape. The
// application cannot tell the difference, and neither can `ros2 action list`.
//
//	go run -tags zenoh ./examples/moveitstub -transport zenoh
//
// Like the Nav2 stand-in it fails on purpose: every fail_every'th request comes
// back PLANNING_FAILED, because a planner that always succeeds would leave the
// commander's recovery branch untested.
package main

import (
	"errors"
	"log/slog"
	"math"
	"sync"
	"time"

	"conductor.dev/conductor"
	_ "conductor.dev/conductor/transport/zenoh"
)

// errPlanning is what a refused or failed request returns alongside its error
// code: MoveIt reports failure twice over, in the goal status and in the
// result, and so does this.
var errPlanning = errors.New("no plan found")

// MoveGroup is the node move_group is: one action server, and a robot that is
// wherever the last accepted plan left it.
//
//conductor:node
type MoveGroup struct {
	Move conductor.Action[MoveGroupGoal, MoveGroupFeedback, MoveGroupResult] `action:"move_action"`

	PlanTime  conductor.Param[time.Duration] `name:"planning_time" default:"300ms"`
	MoveTime  conductor.Param[time.Duration] `name:"execution_time" default:"600ms"`
	FailEvery conductor.Param[int]           `name:"fail_every" default:"13"`

	// Action handlers run one goroutine per goal, off the executor, so the
	// arm they move is locked. A real move_group has the same problem.
	mu       sync.Mutex
	requests int
	joints   map[string]float64
}

func (m *MoveGroup) OnConfigure() error {
	m.joints = map[string]float64{}
	return nil
}

// OnMove answers a planning request: report PLANNING while it thinks, MONITOR
// while it moves, then a trajectory and an error code.
func (m *MoveGroup) OnMove(g *conductor.Goal[MoveGroupGoal, MoveGroupFeedback]) (MoveGroupResult, error) {
	req := g.Value().Request
	m.mu.Lock()
	m.requests++
	fail := m.FailEvery.Get() > 0 && m.requests%m.FailEvery.Get() == 0
	m.mu.Unlock()

	if req.GroupName == "" {
		return m.failed(MoveItErrorCodes_INVALID_GROUP_NAME, "no planning group in the request"), errPlanning
	}
	slog.Info("planning", "group", req.GroupName, "constraints", len(req.GoalConstraints),
		"attempts", req.NumPlanningAttempts, "allowed", req.AllowedPlanningTime)

	g.Feedback(MoveGroupFeedback{State: "PLANNING"})
	if err := sleep(g, m.PlanTime.Get()); err != nil {
		return MoveGroupResult{}, err
	}
	if fail {
		slog.Warn("planning failed", "group", req.GroupName, "request", m.requests)
		return m.failed(MoveItErrorCodes_PLANNING_FAILED, "simulated: no plan found"), errPlanning
	}

	// A joint goal moves the arm to those values; a pose goal is accepted as
	// reached, since there is no kinematics here to disagree with.
	target := jointTarget(req)
	g.Feedback(MoveGroupFeedback{State: "MONITOR"})
	if err := sleep(g, m.MoveTime.Get()); err != nil {
		return MoveGroupResult{}, err
	}

	m.mu.Lock()
	for name, position := range target {
		m.joints[name] = position
	}
	state := m.snapshot()
	m.mu.Unlock()

	g.Feedback(MoveGroupFeedback{State: "IDLE"})
	slog.Info("executed", "group", req.GroupName, "joints", len(state.Name))
	return MoveGroupResult{
		ErrorCode:          MoveItErrorCodes{Val: MoveItErrorCodes_SUCCESS},
		TrajectoryStart:    RobotState{JointState: state},
		PlannedTrajectory:  trajectory(state, m.MoveTime.Get()),
		ExecutedTrajectory: trajectory(state, m.MoveTime.Get()),
		PlanningTime:       m.PlanTime.Get().Seconds(),
	}, nil
}

// failed is what move_group returns when it cannot plan: a result carrying the
// reason, alongside an aborted goal.
func (m *MoveGroup) failed(code int32, why string) MoveGroupResult {
	return MoveGroupResult{ErrorCode: MoveItErrorCodes{Val: code, Message: why}}
}

// snapshot is the arm's current joint state. Callers hold m.mu.
func (m *MoveGroup) snapshot() JointState {
	state := JointState{Header: Header{Stamp: time.Now(), FrameId: "panda_link0"}}
	for name, position := range m.joints {
		state.Name = append(state.Name, name)
		state.Position = append(state.Position, position)
	}
	return state
}

// jointTarget reads the joint values out of a request's constraints. Anything
// else — a position constraint, say — moves the arm somewhere this stand-in
// does not model, so it reports success without changing the joints.
func jointTarget(req MotionPlanRequest) map[string]float64 {
	out := map[string]float64{}
	for _, c := range req.GoalConstraints {
		for _, j := range c.JointConstraints {
			out[j.JointName] = j.Position
		}
	}
	return out
}

// trajectory is a two-point trajectory ending at the state reached: the right
// shape for a client to read, without pretending to be a real plan.
func trajectory(state JointState, over time.Duration) RobotTrajectory {
	start := make([]float64, len(state.Position))
	for i, p := range state.Position {
		start[i] = p - math.Copysign(0.01, p)
	}
	return RobotTrajectory{JointTrajectory: JointTrajectory{
		Header:     state.Header,
		JointNames: state.Name,
		Points: []JointTrajectoryPoint{
			{Positions: start, TimeFromStart: 0},
			{Positions: state.Position, TimeFromStart: over},
		},
	}}
}

// sleep waits, or gives up early if the goal is cancelled.
func sleep(g *conductor.Goal[MoveGroupGoal, MoveGroupFeedback], d time.Duration) error {
	select {
	case <-g.Context().Done():
		return g.Context().Err()
	case <-time.After(d):
		return nil
	}
}

func main() {
	conductor.Run(&MoveGroup{})
}

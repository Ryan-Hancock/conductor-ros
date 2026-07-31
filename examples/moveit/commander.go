package main

import (
	"fmt"
	"log/slog"
	"time"

	"conductor.dev/conductor"
)

// Commander is a pick and place, declared.
//
// What MoveIt asks of an application is a sequence of planning requests, each
// naming a planning group and a goal, with something sensible to do when one
// fails. Written by hand that is a goroutine, a switch, and two magic strings
// per request: the group's name and the named pose, both copied out of an SRDF
// that already holds them.
//
// Here the groups are declared fields resolved against the robot's semantics,
// the sequence is a mission, and the failure path is a tag.
//
//conductor:node
type Commander struct {
	// move_action is MoveIt's own interface: one action carrying the largest
	// nested message in common ROS use.
	Move conductor.ActionClient[MoveGroupGoal, MoveGroupFeedback, MoveGroupResult] `action:"move_action" timeout:"120s"`

	// The planning groups, from panda.srdf by way of groups.json. `conductor
	// check` resolves both the group names and the named configurations these
	// steps ask for, so a pose that does not exist is a build error rather than
	// a request move_group rejects.
	Arm  conductor.Group `group:"panda_arm"`
	Hand conductor.Group `group:"hand"`

	Objects  conductor.Param[int]           `name:"objects" default:"3"`
	Attempts conductor.Param[int]           `name:"planning_attempts" default:"5"`
	Allowed  conductor.Param[time.Duration] `name:"allowed_planning_time" default:"5s"`
	Settle   conductor.Param[time.Duration] `name:"settle" default:"500ms"`

	// The manipulation itself: reach, grasp, lift, place, release, home. A
	// plan that fails goes back to a known configuration and tries again,
	// which is what a person does with an arm that has stopped somewhere odd.
	Job     conductor.Mission `start:"ready"`
	Ready   conductor.Step    `next:"reach" retry:"2" backoff:"1s" fail:"give_up"`
	Reach   conductor.Step    `next:"grasp" fail:"recover"`
	Grasp   conductor.Step    `next:"lift" fail:"recover"`
	Lift    conductor.Step    `next:"place" fail:"recover"`
	Place   conductor.Step    `next:"release" fail:"recover"`
	Release conductor.Step    `next:"home" fail:"recover"`
	Home    conductor.Step    `next:"done" fail:"recover"`
	Recover conductor.Step    `next:"ready" retry:"1" fail:"give_up"`
	GiveUp  conductor.Step    `next:"failed"`

	picked int
}

// OnReady takes the arm to the SRDF's "ready" configuration. The joint values
// are not in this file: they are in the robot's semantics, which is the only
// place they are ever right.
func (c *Commander) OnReady(t *conductor.Task) error {
	// The literal is at the State call, which is where `conductor check`
	// resolves it: the scanner is syntactic, so it can hold a string to the
	// declarations only where it can see it.
	ready, err := c.Arm.State("ready")
	if err != nil {
		return err
	}
	slog.Info("moving to a known configuration", "group", c.Arm.Name(), "attempt", t.Attempt())
	return c.planToState(t, c.Arm, ready)
}

// OnReach plans the arm to the object's pose. This is the one goal that is not
// a named configuration: a pick happens where the object is.
func (c *Commander) OnReach(t *conductor.Task) error {
	target := Point{X: 0.4, Y: 0.1, Z: 0.35}
	slog.Info("reaching for the object", "x", target.X, "y", target.Y, "z", target.Z)
	return c.planToPose(t, c.Arm, "panda_link8", target)
}

// OnGrasp closes the hand, which is a planning request like any other — the
// gripper is a planning group with two named configurations.
func (c *Commander) OnGrasp(t *conductor.Task) error {
	closed, err := c.Hand.State("close")
	if err != nil {
		return err
	}
	slog.Info("closing the hand", "group", c.Hand.Name())
	if err := c.planToState(t, c.Hand, closed); err != nil {
		return err
	}
	return t.Sleep(c.Settle.Get())
}

// OnLift raises the object clear of the table before moving across it.
func (c *Commander) OnLift(t *conductor.Task) error {
	slog.Info("lifting")
	return c.planToPose(t, c.Arm, "panda_link8", Point{X: 0.4, Y: 0.1, Z: 0.5})
}

// OnPlace moves to where the object is going.
func (c *Commander) OnPlace(t *conductor.Task) error {
	slog.Info("placing")
	return c.planToPose(t, c.Arm, "panda_link8", Point{X: 0.4, Y: -0.2, Z: 0.4})
}

// OnRelease opens the hand and counts the object as placed.
func (c *Commander) OnRelease(t *conductor.Task) error {
	open, err := c.Hand.State("open")
	if err != nil {
		return err
	}
	slog.Info("opening the hand")
	if err := c.planToState(t, c.Hand, open); err != nil {
		return err
	}
	c.picked++
	return t.Sleep(c.Settle.Get())
}

// OnHome parks the arm between objects, and ends the job when the last one is
// placed: an application that has finished should stop, not idle. The Goto is
// resolved by `conductor check` like every other transition.
func (c *Commander) OnHome(t *conductor.Task) error {
	ready, err := c.Arm.State("ready")
	if err != nil {
		return err
	}
	if err := c.planToState(t, c.Arm, ready); err != nil {
		return err
	}
	if c.picked < c.Objects.Get() {
		slog.Info("object placed, fetching the next", "placed", c.picked, "of", c.Objects.Get())
		return t.Goto("reach")
	}
	slog.Info("job complete", "objects_placed", c.picked)
	conductor.Shutdown()
	return nil
}

// OnRecover is where a failed plan goes. The arm is somewhere unknown, so the
// only safe move is back to a configuration the SRDF names — and then the
// mission tries the sequence again from the top.
func (c *Commander) OnRecover(t *conductor.Task) error {
	transport, err := c.Arm.State("transport")
	if err != nil {
		return err
	}
	slog.Warn("planning failed, returning to a known configuration", "because", t.Err())
	return c.planToState(t, c.Arm, transport)
}

// OnGiveUp is the failure exit: an arm that cannot reach a known configuration
// is not something to keep sending goals to.
func (c *Commander) OnGiveUp(t *conductor.Task) error {
	conductor.Abort(fmt.Errorf("manipulation failed: %w", t.Err()))
	return nil
}

// planToState asks move_group to reach a named configuration. The joint names
// and values are the SRDF's, so this is the whole of "move the arm to ready" —
// no angles in this file at all.
func (c *Commander) planToState(t *conductor.Task, group conductor.Group, target conductor.NamedState) error {
	constraints := Constraints{Name: target.Name}
	for i, joint := range target.JointNames {
		constraints.JointConstraints = append(constraints.JointConstraints, JointConstraint{
			JointName:      joint,
			Position:       target.Positions[i],
			ToleranceAbove: 0.01,
			ToleranceBelow: 0.01,
			Weight:         1,
		})
	}
	return c.plan(t, group.Name(), constraints)
}

// planToPose asks for a position goal instead: where the object is, rather
// than a configuration somebody named in advance.
func (c *Commander) planToPose(t *conductor.Task, group conductor.Group, link string, at Point) error {
	region := BoundingVolume{
		Primitives:     []SolidPrimitive{{Type: SolidPrimitive_SPHERE, Dimensions: []float64{0.01}}},
		PrimitivePoses: []Pose{{Position: at, Orientation: Quaternion{W: 1}}},
	}
	return c.plan(t, group.Name(), Constraints{
		Name: "pose",
		PositionConstraints: []PositionConstraint{{
			Header:           Header{FrameId: "panda_link0"},
			LinkName:         link,
			ConstraintRegion: region,
			Weight:           1,
		}},
	})
}

// plan sends one motion planning request and waits for it. MoveIt reports
// failure in the result's error code rather than as an aborted goal, so both
// are checked — a plan that "succeeded" with error code -1 has not moved the
// arm anywhere.
func (c *Commander) plan(t *conductor.Task, group string, goal Constraints) error {
	handle, err := c.Move.SendGoal(MoveGroupGoal{
		Request: MotionPlanRequest{
			GroupName:                    group,
			GoalConstraints:              []Constraints{goal},
			NumPlanningAttempts:          int32(c.Attempts.Get()),
			AllowedPlanningTime:          c.Allowed.Get().Seconds(),
			MaxVelocityScalingFactor:     0.3,
			MaxAccelerationScalingFactor: 0.3,
		},
		PlanningOptions: PlanningOptions{PlanOnly: false, ReplanAttempts: 2},
	})
	if err != nil {
		return fmt.Errorf("move_action: %w", err)
	}
	go c.logProgress(handle)

	result, status, err := handle.Result()
	if err != nil {
		return fmt.Errorf("waiting for the plan: %w", err)
	}
	if !status.Succeeded() {
		return fmt.Errorf("planning %s: %s (moveit error %s)", group, status, moveItError(result.ErrorCode))
	}
	if result.ErrorCode.Val != MoveItErrorCodes_SUCCESS {
		return fmt.Errorf("planning %s: %s", group, moveItError(result.ErrorCode))
	}
	slog.Info("plan executed", "group", group,
		"planning_time", time.Duration(result.PlanningTime*float64(time.Second)).Round(time.Millisecond),
		"points", len(result.PlannedTrajectory.JointTrajectory.Points))
	return nil
}

// logProgress reports the state move_group publishes as it works: PLANNING,
// MONITOR, IDLE.
func (c *Commander) logProgress(h *conductor.GoalHandle[MoveGroupFeedback, MoveGroupResult]) {
	for fb := range h.Feedback() {
		slog.Info("move_group", "state", fb.State)
	}
}

// moveItError names the error code, because "-1" is not a diagnosis.
func moveItError(code MoveItErrorCodes) string {
	name := map[int32]string{
		MoveItErrorCodes_SUCCESS:                  "success",
		MoveItErrorCodes_FAILURE:                  "failure",
		MoveItErrorCodes_PLANNING_FAILED:          "planning failed",
		MoveItErrorCodes_INVALID_MOTION_PLAN:      "invalid motion plan",
		MoveItErrorCodes_CONTROL_FAILED:           "control failed",
		MoveItErrorCodes_TIMED_OUT:                "timed out",
		MoveItErrorCodes_START_STATE_IN_COLLISION: "start state in collision",
		MoveItErrorCodes_GOAL_IN_COLLISION:        "goal in collision",
		MoveItErrorCodes_INVALID_GROUP_NAME:       "invalid group name",
		MoveItErrorCodes_NO_IK_SOLUTION:           "no inverse kinematics solution",
	}[code.Val]
	if name == "" {
		name = fmt.Sprintf("code %d", code.Val)
	}
	if code.Message != "" {
		return name + ": " + code.Message
	}
	return name
}

package main

import (
	"log/slog"
	"math"
	"time"

	"conductor.dev/conductor"
	"conductor.dev/conductor/msgs"
	"conductor.dev/conductor/srvs"
)

//conductor:node
type Localizer struct {
	Clock conductor.Timer                 `rate:"5hz"`
	Pose  conductor.Pub[msgs.PoseStamped] `topic:"amcl_pose" qos:"reliable"`

	x, y float64
}

// OnClock publishes a simulated pose estimate drifting across the map.
func (l *Localizer) OnClock() {
	l.x += 0.1
	l.y += 0.08
	l.Pose.Publish(msgs.PoseStamped{
		Header: msgs.Header{Stamp: time.Now(), FrameID: "map"},
		Pose:   msgs.Pose{Position: msgs.Point{X: l.x, Y: l.y}},
	})
}

//conductor:node
type Navigator struct {
	Pose     conductor.Sub[msgs.PoseStamped] `topic:"amcl_pose" qos:"reliable"`
	Cmd      conductor.Pub[msgs.Twist]       `topic:"cmd_vel" qos:"reliable"`
	MaxSpeed conductor.Param[float64]        `name:"max_speed" default:"1.5"`

	goal msgs.Point
}

// OnPose steers toward the goal, clamping speed to the max_speed parameter.
func (n *Navigator) OnPose(p msgs.PoseStamped) {
	dx := n.goal.X - p.Pose.Position.X
	dy := n.goal.Y - p.Pose.Position.Y
	dist := math.Hypot(dx, dy)
	if dist < 0.1 {
		n.Cmd.Publish(msgs.Twist{})
		return
	}
	speed := math.Min(dist, n.MaxSpeed.Get())
	n.Cmd.Publish(msgs.Twist{
		Linear:  msgs.Vector3{X: speed * dx / dist, Y: speed * dy / dist},
		Angular: msgs.Vector3{Z: math.Atan2(dy, dx)},
	})
}

//conductor:node
type SafetyMonitor struct {
	Cmd      conductor.Sub[msgs.Twist]                              `topic:"cmd_vel" qos:"reliable"`
	Estop    conductor.Sub[msgs.Bool]                               `topic:"estop" qos:"transient"`
	Status   conductor.Pub[PatrolStatus]                            `topic:"patrol_status" qos:"reliable"`
	Engage   conductor.Svc[srvs.SetBoolRequest, srvs.SetBoolResponse] `service:"engage_estop"`
	Watchdog conductor.Timer                                        `rate:"1hz"`

	lastCmd time.Time
	stopped bool
}

// OnEngage sets or clears the e-stop; like all callbacks it runs on the
// node's executor, so touching stopped is safe.
func (s *SafetyMonitor) OnEngage(req srvs.SetBoolRequest) (srvs.SetBoolResponse, error) {
	s.stopped = req.Data
	slog.Warn("estop set via service", "engaged", s.stopped)
	return srvs.SetBoolResponse{Success: true, Message: "estop updated"}, nil
}

func (s *SafetyMonitor) OnCmd(msgs.Twist) {
	s.lastCmd = time.Now()
}

func (s *SafetyMonitor) OnEstop(b msgs.Bool) {
	s.stopped = b.Data
	slog.Warn("estop state changed", "engaged", s.stopped)
}

func (s *SafetyMonitor) OnWatchdog() {
	stale := !s.lastCmd.IsZero() && time.Since(s.lastCmd) > 2*time.Second
	if stale {
		slog.Warn("cmd_vel stale", "since", time.Since(s.lastCmd).Round(time.Millisecond))
	}
	mode := uint8(PatrolStatus_MODE_PATROLLING)
	if s.stopped {
		mode = PatrolStatus_MODE_ESTOPPED
	}
	s.Status.Publish(PatrolStatus{
		Header:   Header{Stamp: time.Now(), FrameId: "base_link"},
		Mode:     mode,
		CmdStale: stale,
	})
}

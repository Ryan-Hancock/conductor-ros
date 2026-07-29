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
	Pose  conductor.Pub[msgs.PoseStamped] `topic:"amcl_pose" qos:"reliable" frame:"map"`

	x, y float64
}

// OnClock publishes a simulated pose estimate drifting across the map.
func (l *Localizer) OnClock() {
	l.x += 0.1
	l.y += 0.08
	// The frame tag stamps the header: the frame a topic is in is a
	// declaration, not something every publish site repeats.
	l.Pose.Publish(msgs.PoseStamped{Pose: msgs.Pose{Position: msgs.Point{X: l.x, Y: l.y}}})
}

// Patroller is the task layer: the route is a mission, and the mission is a
// declaration. `conductor check` prints the machine, gen/mission.dot draws
// it, and the dashboard shows which step the robot is on.
//
//conductor:node
type Patroller struct {
	Goal  conductor.Pub[msgs.PoseStamped] `topic:"goal_pose" qos:"reliable" frame:"map"`
	Estop conductor.Sub[msgs.Bool]        `topic:"estop" qos:"transient"`
	Dwell conductor.Param[time.Duration]  `name:"dwell" default:"3s"`

	Route    conductor.Mission `start:"drive_to"`
	DriveTo  conductor.Step    `next:"dwelling"`
	Dwelling conductor.Step    `next:"drive_to"`
	Holding  conductor.Step    `next:"drive_to"`

	waypoints []msgs.Point
	at        int
	stopped   bool // written by OnEstop on the executor
}

func (p *Patroller) OnConfigure() error {
	p.waypoints = []msgs.Point{{X: 5, Y: 4}, {X: 5, Y: -4}, {X: -5, Y: -4}, {X: -5, Y: 4}}
	return nil
}

func (p *Patroller) OnEstop(b msgs.Bool) { p.stopped = b.Data }

// OnDriveTo sends the next waypoint — unless the robot is stopped, which is a
// branch the tags do not cover, so the step takes it itself.
func (p *Patroller) OnDriveTo(t *conductor.Task) error {
	var stopped bool
	// stopped belongs to the executor (OnEstop writes it), so a step reads it
	// through the executor rather than racing with it.
	if err := t.Do(func() { stopped = p.stopped }); err != nil {
		return err
	}
	if stopped {
		return t.Goto("holding")
	}
	next := p.waypoints[p.at%len(p.waypoints)]
	slog.Info("driving to waypoint", "index", p.at%len(p.waypoints), "x", next.X, "y", next.Y)
	p.Goal.Publish(msgs.PoseStamped{Pose: msgs.Pose{Position: next}})
	return nil
}

// OnDwelling waits at the waypoint, then moves the route on. Sleeping through
// the Task means a deactivation interrupts it.
func (p *Patroller) OnDwelling(t *conductor.Task) error {
	if err := t.Sleep(p.Dwell.Get()); err != nil {
		return err
	}
	p.at++
	return nil
}

// OnHolding waits out an e-stop, then hands back to the route. Waiting is a
// loop around an interruptible sleep, so deactivating the node ends it.
func (p *Patroller) OnHolding(t *conductor.Task) error {
	for {
		var stopped bool
		if err := t.Do(func() { stopped = p.stopped }); err != nil {
			return err
		}
		if !stopped {
			slog.Info("e-stop cleared, resuming the route")
			return nil
		}
		if err := t.Sleep(time.Second); err != nil {
			return err
		}
	}
}

//conductor:node
type Navigator struct {
	Pose     conductor.Sub[msgs.PoseStamped] `topic:"amcl_pose" qos:"reliable" frame:"map"`
	Target   conductor.Sub[msgs.PoseStamped] `topic:"goal_pose" qos:"reliable" frame:"map"`
	Cmd      conductor.Pub[msgs.Twist]       `topic:"cmd_vel" qos:"reliable"`
	MaxSpeed conductor.Param[float64]        `name:"max_speed" default:"1.5"`

	goal msgs.Point
}

// OnTarget accepts the waypoint the patrol mission is driving to.
func (n *Navigator) OnTarget(p msgs.PoseStamped) { n.goal = p.Pose.Position }

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
	Cmd      conductor.Sub[msgs.Twist]                                `topic:"cmd_vel" qos:"reliable"`
	Estop    conductor.Sub[msgs.Bool]                                 `topic:"estop" qos:"transient"`
	Status   conductor.Pub[PatrolStatus]                              `topic:"patrol_status" qos:"reliable" frame:"base_link"`
	Engage   conductor.Svc[srvs.SetBoolRequest, srvs.SetBoolResponse] `service:"engage_estop"`
	Watchdog conductor.Timer                                          `rate:"1hz"`
	TF       conductor.TF

	lastCmd    time.Time
	stopped    bool
	laserAhead float64 // how far in front of base_link the lidar sits
}

// OnConfigure reads the robot's geometry out of the declared transform tree.
// `conductor check` resolves this same lookup statically, so a frames.json
// that cannot answer it is a build error rather than a surprise here.
func (s *SafetyMonitor) OnConfigure() error {
	at, err := s.TF.Lookup("base_link", "laser")
	if err != nil {
		return err
	}
	s.laserAhead = at.Translation[0]
	slog.Info("lidar mounted", "ahead_of_base_link_m", s.laserAhead)
	return nil
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
	s.Status.Publish(PatrolStatus{Mode: mode, CmdStale: stale})
}

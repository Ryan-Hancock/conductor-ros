package main

import (
	"fmt"
	"log/slog"
	"math"
	"time"

	"conductor.dev/conductor"
)

// Watchdog is the second half of the reason this example has two nodes: it
// watches what Nav2 is actually doing, which is a different job from deciding
// what it should do, and it wants to keep running when the commander is
// blocked inside a five-minute goal.
//
// It also gives the split run something to see: `conductor run -split` starts
// these two nodes as two processes over zenoh, and patrol/waypoint is the one
// topic that crosses between them, so the fleet view has a real graph to draw
// rather than two unrelated halves.
//
//conductor:node
type Watchdog struct {
	// Nav2's controller publishes TwistStamped here by default
	// (enable_stamped_cmd_vel), stamped in the costmap's robot_base_frame.
	// The frame tag verifies that: an incoming header in some other frame is
	// reported rather than quietly averaged into a distance.
	Cmd conductor.Sub[TwistStamped] `topic:"cmd_vel" qos:"reliable" frame:"base_link"`

	Waypoint conductor.Sub[PoseStamped]               `topic:"patrol/waypoint" qos:"reliable" frame:"map"`
	Pose     conductor.Sub[PoseWithCovarianceStamped] `topic:"amcl_pose" qos:"transient" frame:"map"`

	// /diagnostics is where ROS expects this to go, so it goes there — a
	// diagnostic aggregator or rqt_robot_monitor picks it up with no
	// knowledge of conductor at all.
	Report conductor.Pub[DiagnosticArray] `topic:"diagnostics" qos:"reliable"`

	Beat       conductor.Timer                `rate:"1hz"`
	StallAfter conductor.Param[time.Duration] `name:"stall_after" default:"5s"`
	// How close counts as arrived. Without it, a robot parked on its
	// waypoint — charging, say — looks exactly like a stalled one: a goal
	// outstanding and no motion.
	ArrivedWithin conductor.Param[float64] `name:"arrived_within" default:"0.35"`

	target     Point
	haveTarget bool
	pose       Point
	havePose   bool
	poseAt     time.Time
	movingAt   time.Time
	closest    float64
}

// OnWaypoint follows the commander's goal. A new waypoint resets the progress
// measure: the robot is meant to be getting closer to this one now.
func (w *Watchdog) OnWaypoint(p PoseStamped) {
	w.target, w.haveTarget = p.Pose.Position, true
	w.closest = math.Inf(1)
	w.movingAt = time.Now()
}

func (w *Watchdog) OnPose(p PoseWithCovarianceStamped) {
	w.pose, w.havePose, w.poseAt = p.Pose.Pose.Position, true, time.Now()
	if d, ok := w.distance(); ok && d < w.closest {
		w.closest = d
	}
}

// OnCmd notes that the controller is still asking for motion. A zero command
// is Nav2 holding still, which is exactly what a stall looks like.
func (w *Watchdog) OnCmd(c TwistStamped) {
	if math.Hypot(c.Twist.Linear.X, c.Twist.Linear.Y) > 0.01 || math.Abs(c.Twist.Angular.Z) > 0.01 {
		w.movingAt = time.Now()
	}
}

// OnBeat publishes one diagnostic status per second: whether the robot is
// making progress toward the waypoint it was given, and whether localization
// is still arriving.
func (w *Watchdog) OnBeat() {
	status := DiagnosticStatus{Name: "patrol: navigation", HardwareId: "nav2"}
	distance, measured := w.distance()

	switch {
	case !w.havePose || time.Since(w.poseAt) > 5*time.Second:
		status.Level = DiagnosticStatus_STALE
		status.Message = "no pose from amcl"
	case !w.haveTarget:
		status.Level = DiagnosticStatus_OK
		status.Message = "idle: no waypoint"
	case measured && distance <= w.ArrivedWithin.Get():
		status.Level = DiagnosticStatus_OK
		status.Message = "at the waypoint"
	case time.Since(w.movingAt) > w.StallAfter.Get():
		status.Level = DiagnosticStatus_WARN
		status.Message = fmt.Sprintf("stalled: no motion for %s", time.Since(w.movingAt).Round(time.Second))
		slog.Warn("navigation appears stalled",
			"since", time.Since(w.movingAt).Round(time.Second), "closest_approach", round(w.closest))
	default:
		status.Level = DiagnosticStatus_OK
		status.Message = "navigating"
	}

	if measured {
		status.Values = append(status.Values,
			KeyValue{Key: "distance_to_waypoint", Value: fmt.Sprintf("%.2f", distance)},
			KeyValue{Key: "closest_approach", Value: fmt.Sprintf("%.2f", w.closest)})
	}
	w.Report.Publish(DiagnosticArray{Status: []DiagnosticStatus{status}})
}

// distance is how far the robot is from the waypoint it was given.
func (w *Watchdog) distance() (float64, bool) {
	if !w.haveTarget || !w.havePose {
		return 0, false
	}
	return math.Hypot(w.target.X-w.pose.X, w.target.Y-w.pose.Y), true
}

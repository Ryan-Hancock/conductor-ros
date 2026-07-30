package main

import (
	"fmt"
	"log/slog"
	"math"
	"strings"
	"time"

	"conductor.dev/conductor"
)

// Commander decides what Nav2 should be doing. It plans nothing and controls
// nothing: every motion in this example is computed by Nav2's planner and
// controller, and every recovery is Nav2's own behaviour server.
//
// What it replaces is the part of a Nav2 deployment that is normally spread
// across four places — a lifecycle_manager, a launch file's startup ordering,
// a behaviour tree XML, and a Python node with a goroutine and a flag. Here
// the stack's startup is the mission's first step, the route is a state
// machine of fields and tags, and a failed goal has a declared recovery
// branch. `conductor check` prints the machine, gen/mission.dot draws it, and
// the dashboard shows which step the robot is on.
//
//conductor:node
type Commander struct {
	// Nav2's servers are managed nodes: navigate_to_pose does not answer until
	// something has driven them through Configure and Activate. Nav2 ships a
	// lifecycle_manager to do it, configured by a parameter nobody checks and
	// started from a launch file. This is that list, declared — and bringing
	// the stack up is a mission step with a retry tag rather than the
	// `sleep 10` that usually stands in for one.
	Stack conductor.Lifecycle `nodes:"map_server,amcl,controller_server,planner_server,behavior_server,bt_navigator" timeout:"30s"`

	// The action servers this application drives: bt_navigator, and the two
	// recovery behaviours it falls back on.
	NavTo   conductor.ActionClient[NavigateToPoseGoal, NavigateToPoseFeedback, NavigateToPoseResult] `action:"navigate_to_pose" timeout:"5m"`
	Reverse conductor.ActionClient[BackUpGoal, BackUpFeedback, BackUpResult]                         `action:"backup" timeout:"30s"`
	Rotate  conductor.ActionClient[SpinGoal, SpinFeedback, SpinResult]                               `action:"spin" timeout:"30s"`

	// AMCL's two ends of the localization conversation, and the battery the
	// docking branch watches. The frame tags stamp what goes out and verify
	// what comes in: "map" is declared once, here, not at every publish.
	Estimate conductor.Pub[PoseWithCovarianceStamped] `topic:"initialpose" qos:"reliable" frame:"map"`
	Pose     conductor.Sub[PoseWithCovarianceStamped] `topic:"amcl_pose" qos:"transient" frame:"map"`
	Battery  conductor.Sub[BatteryState]              `topic:"battery_state" qos:"sensor"`

	// The waypoint under way, for anything else in the application that
	// wants to know — the watchdog, and the dashboard.
	Waypoint conductor.Pub[PoseStamped] `topic:"patrol/waypoint" qos:"reliable" frame:"map"`

	LowBattery    conductor.Param[float64]       `name:"low_battery" default:"0.25"`
	ResumeBattery conductor.Param[float64]       `name:"resume_battery" default:"0.9"`
	Dwell         conductor.Param[time.Duration] `name:"dwell" default:"2s"`
	BackUpBy      conductor.Param[float64]       `name:"backup_distance" default:"0.3"`

	// The patrol itself. Read the tags as the sentence they are: bring the
	// stack up, then localize, then drive to a waypoint, following it until
	// it is reached; if following fails, recover and try the same waypoint
	// again; if the stack never comes up, give up.
	Patrol    conductor.Mission `start:"bring_up"`
	BringUp   conductor.Step    `next:"localize" retry:"3" backoff:"2s" fail:"give_up"`
	Localize  conductor.Step    `next:"drive_to" timeout:"30s" fail:"give_up"`
	DriveTo   conductor.Step    `next:"following" retry:"2" backoff:"1s" fail:"give_up"`
	Following conductor.Step    `next:"arrived" fail:"recover"`
	Arrived   conductor.Step    `next:"drive_to"`
	Recover   conductor.Step    `next:"drive_to" retry:"1" fail:"give_up"`
	Docking   conductor.Step    `next:"charging" retry:"3" backoff:"2s" fail:"give_up"`
	Charging  conductor.Step    `next:"drive_to"`
	GiveUp    conductor.Step    `next:"failed"`

	// Route data. A real deployment would read this from a parameter file;
	// it is here so the example is one file to read.
	route []Point
	dock  Point
	at    int

	// Held between steps, which run one at a time on the mission's own
	// goroutine — so no lock is needed for these.
	goal *conductor.GoalHandle[NavigateToPoseFeedback, NavigateToPoseResult]

	// Written by callbacks on the executor. Steps read them through
	// Task.Do, which hops onto the executor rather than racing with it.
	pose     Point
	havePose bool
	charge   float64
}

func (c *Commander) OnConfigure() error {
	c.route = []Point{{X: 1.6, Y: 0.4}, {X: 1.6, Y: -1.2}, {X: -1.2, Y: -1.2}, {X: -1.2, Y: 0.4}}
	c.dock = Point{X: 0, Y: 0}
	return nil
}

// OnPose records where AMCL thinks the robot is.
func (c *Commander) OnPose(p PoseWithCovarianceStamped) {
	c.pose = p.Pose.Pose.Position
	c.havePose = true
}

// OnBattery records the charge the docking branch decides on.
func (c *Commander) OnBattery(b BatteryState) { c.charge = float64(b.Percentage) }

// OnBringUp configures and activates the navigation stack, in the order the
// field declares. This is what a lifecycle_manager does; the difference is that
// the order is a declaration `conductor check` prints, and a node that will not
// come up is named here rather than in somebody's launch log.
//
// The retry tag covers the ordinary case of the stack not being discoverable
// yet: this is a distributed system coming up, and "not yet" is not "broken".
// BringUp leaves nodes that are already Active alone, so retrying is safe.
func (c *Commander) OnBringUp(t *conductor.Task) error {
	slog.Info("bringing the navigation stack up",
		"nodes", strings.Join(c.Stack.Nodes(), " -> "), "attempt", t.Attempt())
	if err := c.Stack.BringUp(t.Context()); err != nil {
		return fmt.Errorf("navigation stack: %w", err)
	}
	// A transition reporting success and a stack being ready are different
	// claims, so this asks again. The error names whichever nodes are not up.
	if err := c.Stack.AwaitActive(t.Context(), 10*time.Second); err != nil {
		return err
	}
	slog.Info("navigation stack active", "nodes", len(c.Stack.Nodes()))
	return nil
}

// OnLocalize seeds AMCL and waits for it to answer with a pose. The 30s
// timeout tag is the whole of the "what if it never converges" handling.
func (c *Commander) OnLocalize(t *conductor.Task) error {
	// The covariance RViz sends for a hand-placed estimate: half a metre of
	// position uncertainty and about fifteen degrees of heading.
	var cov [36]float64
	cov[0], cov[7], cov[35] = 0.25, 0.25, 0.0685
	c.Estimate.Publish(PoseWithCovarianceStamped{
		Pose: PoseWithCovariance{Pose: Pose{Orientation: yawQuaternion(0)}, Covariance: cov},
	})

	for {
		var ready bool
		if err := t.Do(func() { ready = c.havePose }); err != nil {
			return err
		}
		if ready {
			slog.Info("localized", "x", c.pose.X, "y", c.pose.Y)
			return nil
		}
		if err := t.Sleep(200 * time.Millisecond); err != nil {
			return err
		}
	}
}

// OnDriveTo sends the next waypoint to bt_navigator. Sending is its own step
// because accepting a goal and achieving it are different failures: a goal
// the server will not take should be retried, one it aborts halfway wants a
// recovery behaviour.
func (c *Commander) OnDriveTo(t *conductor.Task) error {
	next := c.route[c.at%len(c.route)]
	slog.Info("navigating to waypoint", "index", c.at%len(c.route), "x", next.X, "y", next.Y, "attempt", t.Attempt())

	// Anything watching the application — the watchdog, the dashboard —
	// learns the goal from this, not from Nav2's internals.
	c.Waypoint.Publish(PoseStamped{Pose: Pose{Position: next, Orientation: yawQuaternion(0)}})

	h, err := c.NavTo.SendGoal(NavigateToPoseGoal{
		Pose: PoseStamped{Pose: Pose{Position: next, Orientation: yawQuaternion(0)}},
	})
	if err != nil {
		return fmt.Errorf("navigate_to_pose: %w", err)
	}
	c.goal = h
	return nil
}

// OnFollowing follows the goal to its end. Nav2 reports an aborted goal
// through the result status rather than an error, so this is where a failed
// navigation becomes a failed step — and the fail: tag turns that into the
// recovery branch.
func (c *Commander) OnFollowing(t *conductor.Task) error {
	go c.logProgress(c.goal)

	res, status, err := c.goal.Result()
	if err != nil {
		return fmt.Errorf("waiting for navigate_to_pose: %w", err)
	}
	if !status.Succeeded() {
		return fmt.Errorf("navigation %s: %s (nav2 error_code %d)", status, orNoMessage(res.ErrorMsg), res.ErrorCode)
	}
	return nil
}

// OnArrived dwells at the waypoint and decides what comes next: the route, or
// the dock. `conductor check` resolves this Goto statically — a typo here is
// a build error, not a mission that fails at 3am on the fourth waypoint.
func (c *Commander) OnArrived(t *conductor.Task) error {
	slog.Info("arrived", "waypoint", c.at%len(c.route))
	if err := t.Sleep(c.Dwell.Get()); err != nil {
		return err
	}
	c.at++

	var charge float64
	if err := t.Do(func() { charge = c.charge }); err != nil {
		return err
	}
	if charge > 0 && charge < c.LowBattery.Get() {
		slog.Warn("battery low, heading for the dock", "charge", charge, "below", c.LowBattery.Get())
		return t.Goto("docking")
	}
	return nil
}

// OnRecover runs Nav2's recovery behaviours, the same two a behaviour tree
// would run: back away from whatever the controller could not get around,
// then spin to re-acquire the costmap. t.Err() is the failure that routed us
// here, which is the reason a fail: branch is a step and not a callback.
func (c *Commander) OnRecover(t *conductor.Task) error {
	slog.Warn("recovering", "because", t.Err(), "attempt", t.Attempt())

	back, err := c.Reverse.SendGoal(BackUpGoal{
		Target:        Point{X: -c.BackUpBy.Get()},
		Speed:         0.15,
		TimeAllowance: 15 * time.Second,
	})
	if err != nil {
		return fmt.Errorf("backup: %w", err)
	}
	if _, status, err := back.Result(); err != nil {
		return fmt.Errorf("backup result: %w", err)
	} else if !status.Succeeded() {
		return fmt.Errorf("backup %s", status)
	}

	spin, err := c.Rotate.SendGoal(SpinGoal{TargetYaw: math.Pi, TimeAllowance: 20 * time.Second})
	if err != nil {
		return fmt.Errorf("spin: %w", err)
	}
	if _, status, err := spin.Result(); err != nil {
		return fmt.Errorf("spin result: %w", err)
	} else if !status.Succeeded() {
		return fmt.Errorf("spin %s", status)
	}

	// The route index is untouched, so the same waypoint is tried again.
	slog.Info("recovered, retrying the waypoint", "index", c.at%len(c.route))
	return nil
}

// OnDocking drives to the dock with the same action server the patrol uses —
// going home is navigation like any other. It retries rather than recovering:
// a robot low on battery has somewhere it needs to be, and if three attempts
// cannot get it there, a spin will not help.
func (c *Commander) OnDocking(t *conductor.Task) error {
	slog.Info("docking", "x", c.dock.X, "y", c.dock.Y, "attempt", t.Attempt())
	c.Waypoint.Publish(PoseStamped{Pose: Pose{Position: c.dock, Orientation: yawQuaternion(0)}})

	h, err := c.NavTo.SendGoal(NavigateToPoseGoal{
		Pose: PoseStamped{Pose: Pose{Position: c.dock, Orientation: yawQuaternion(0)}},
	})
	if err != nil {
		return fmt.Errorf("navigate_to_pose (dock): %w", err)
	}
	go c.logProgress(h)
	res, status, err := h.Result()
	if err != nil {
		return fmt.Errorf("waiting for the dock: %w", err)
	}
	if !status.Succeeded() {
		return fmt.Errorf("navigation to the dock %s: %s (nav2 error_code %d)", status, orNoMessage(res.ErrorMsg), res.ErrorCode)
	}
	return nil
}

// OnCharging waits for the battery to come back, then rejoins the route.
func (c *Commander) OnCharging(t *conductor.Task) error {
	slog.Info("charging", "until", c.ResumeBattery.Get())
	for {
		var charge float64
		if err := t.Do(func() { charge = c.charge }); err != nil {
			return err
		}
		if charge >= c.ResumeBattery.Get() {
			slog.Info("charged, resuming the patrol", "charge", charge)
			return nil
		}
		if err := t.Sleep(time.Second); err != nil {
			return err
		}
	}
}

// OnGiveUp is the failure exit: park the stack rather than leaving servers
// active with nobody driving them, and fail the process so whatever started
// it knows. Deactivating runs the list in reverse, which is the same rule the
// runtime's own shutdown follows.
func (c *Commander) OnGiveUp(t *conductor.Task) error {
	slog.Error("giving up on the patrol", "err", t.Err(), "not_active", c.Stack.NotActive())
	if err := c.Stack.Deactivate(); err != nil {
		slog.Error("could not deactivate the navigation stack", "err", err)
	}
	conductor.Abort(fmt.Errorf("patrol failed: %w", t.Err()))
	return nil
}

// logProgress reports what Nav2 says about the goal under way. Feedback
// arrives faster than anyone wants to read, so this thins it out.
func (c *Commander) logProgress(h *conductor.GoalHandle[NavigateToPoseFeedback, NavigateToPoseResult]) {
	var last time.Time
	for fb := range h.Feedback() {
		if time.Since(last) < 2*time.Second {
			continue
		}
		last = time.Now()
		slog.Info("navigating",
			"distance_remaining", round(float64(fb.DistanceRemaining)),
			"eta", fb.EstimatedTimeRemaining.Round(100*time.Millisecond),
			"recoveries", fb.NumberOfRecoveries)
	}
}

// yawQuaternion is the orientation a heading of yaw radians about z.
func yawQuaternion(yaw float64) Quaternion {
	return Quaternion{Z: math.Sin(yaw / 2), W: math.Cos(yaw / 2)}
}

func round(v float64) float64 { return math.Round(v*100) / 100 }

func orNoMessage(s string) string {
	if s == "" {
		return "no message"
	}
	return s
}

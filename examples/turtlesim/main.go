// Turtlesim runs the classic ROS 2 turtlesim tutorial from conductor,
// against the real turtlesim_node: it reads the turtle's pose, drives it in
// a square, spawns a second turtle, teleports it, and rotates turtle1 with
// the RotateAbsolute action.
//
//	ros2 run turtlesim turtlesim_node        # in one terminal
//	turtlesim -transport zenoh               # in another
//
// Everything here is ordinary conductor code: nothing knows it is talking to
// a C++ node.
package main

import (
	"log/slog"
	"math"
	"os"
	"time"

	"conductor.dev/conductor"
	_ "conductor.dev/conductor/transport/zenoh"
)

//conductor:node
type TurtleDriver struct {
	Pose    conductor.Sub[Pose]      `topic:"turtle1/pose" qos:"sensor"`
	Cmd     conductor.Pub[Twist]     `topic:"turtle1/cmd_vel" qos:"reliable"`
	Start   conductor.Timer          `rate:"1hz"`
	EdgeLen conductor.Param[float64] `name:"edge_length" default:"2.0"`

	Spawn    conductor.Client[SpawnRequest, SpawnResponse]                                            `service:"spawn" timeout:"5s"`
	SetPen   conductor.Client[SetPenRequest, SetPenResponse]                                          `service:"turtle1/set_pen" timeout:"5s"`
	Teleport conductor.Client[TeleportAbsoluteRequest, TeleportAbsoluteResponse]                      `service:"turtle2/teleport_absolute" timeout:"5s"`
	Rotate   conductor.ActionClient[RotateAbsoluteGoal, RotateAbsoluteFeedback, RotateAbsoluteResult] `action:"turtle1/rotate_absolute" timeout:"30s"`

	latest  Pose
	haveIt  bool
	started bool
}

// OnPose records the turtle's position; turtlesim publishes it at 62 Hz.
func (t *TurtleDriver) OnPose(p Pose) {
	t.latest = p
	if !t.haveIt {
		t.haveIt = true
		slog.Info("first pose from turtlesim", "x", p.X, "y", p.Y, "theta", p.Theta)
	}
}

// OnStart runs the tutorial once, on its own goroutine because the service
// and action calls block.
func (t *TurtleDriver) OnStart() {
	if t.started || !t.haveIt {
		return
	}
	t.started = true
	go t.tutorial()
}

func (t *TurtleDriver) tutorial() {
	fail := func(step string, err error) {
		slog.Error("step failed", "step", step, "err", err)
		os.Exit(1)
	}

	// 1. Service call: draw with a thick red pen.
	if _, err := t.SetPen.Call(SetPenRequest{R: 255, G: 0, B: 0, Width: 5}); err != nil {
		fail("set_pen", err)
	}
	slog.Info("set_pen: turtle1 now draws in red")

	// 2. Drive a square, using the pose feed to know when each edge is done.
	for edge := 1; edge <= 4; edge++ {
		t.driveForward(t.EdgeLen.Get())
		t.turnLeft()
		slog.Info("completed edge", "edge", edge, "x", t.latest.X, "y", t.latest.Y)
	}

	// 3. Service call with a response value: spawn a second turtle.
	spawned, err := t.Spawn.Call(SpawnRequest{X: 2.0, Y: 2.0, Theta: 0.0, Name: "turtle2"})
	if err != nil {
		fail("spawn", err)
	}
	slog.Info("spawned a second turtle", "name", spawned.Name)

	// 4. Service call on the new turtle.
	if _, err := t.Teleport.Call(TeleportAbsoluteRequest{X: 8.0, Y: 8.0, Theta: 1.57}); err != nil {
		fail("teleport_absolute", err)
	}
	slog.Info("teleported turtle2 to (8, 8)")

	// 5. Action with feedback: rotate turtle1 to an absolute heading.
	goal, err := t.Rotate.SendGoal(RotateAbsoluteGoal{Theta: math.Pi})
	if err != nil {
		fail("rotate_absolute send", err)
	}
	go func() {
		for fb := range goal.Feedback() {
			slog.Info("rotate feedback", "remaining", fb.Remaining)
		}
	}()
	result, status, err := goal.Result()
	if err != nil {
		fail("rotate_absolute result", err)
	}
	slog.Info("rotate finished", "status", status.String(), "delta", result.Delta)

	slog.Info("turtlesim tutorial complete", "final_x", t.latest.X, "final_y", t.latest.Y, "final_theta", t.latest.Theta)
	os.Exit(0)
}

// driveForward publishes velocity until the turtle has moved dist, judging
// progress from the pose topic rather than dead reckoning.
func (t *TurtleDriver) driveForward(dist float64) {
	startX, startY := t.latest.X, t.latest.Y
	deadline := time.After(15 * time.Second)
	for {
		moved := math.Hypot(float64(t.latest.X-startX), float64(t.latest.Y-startY))
		if moved >= dist {
			t.Cmd.Publish(Twist{})
			return
		}
		select {
		case <-deadline:
			slog.Warn("drive timed out", "moved", moved)
			t.Cmd.Publish(Twist{})
			return
		case <-time.After(20 * time.Millisecond):
			t.Cmd.Publish(Twist{Linear: Vector3{X: 1.5}})
		}
	}
}

// turnLeft rotates the turtle 90 degrees.
func (t *TurtleDriver) turnLeft() {
	start := t.latest.Theta
	deadline := time.After(15 * time.Second)
	for {
		turned := math.Abs(angleDiff(float64(t.latest.Theta), float64(start)))
		if turned >= math.Pi/2-0.02 {
			t.Cmd.Publish(Twist{})
			return
		}
		select {
		case <-deadline:
			t.Cmd.Publish(Twist{})
			return
		case <-time.After(20 * time.Millisecond):
			t.Cmd.Publish(Twist{Angular: Vector3{Z: 1.0}})
		}
	}
}

func angleDiff(a, b float64) float64 {
	d := math.Mod(a-b+math.Pi, 2*math.Pi)
	if d < 0 {
		d += 2 * math.Pi
	}
	return d - math.Pi
}

func main() {
	conductor.Run(&TurtleDriver{})
}

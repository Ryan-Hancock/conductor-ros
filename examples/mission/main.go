// Mission is the conductor mission-layer example: a task state machine
// declared as fields, driving the standard example_interfaces/action/Fibonacci
// action. It streams feedback, gives up if the server takes too long, and
// cancels the goal on the way out. It works against any ROS 2 action server —
// the Go one in examples/fibonacci, an rclpy server, or a real stack like
// Nav2.
//
//	mission -transport zenoh
//
// The same flow written by hand is a goroutine, a bool, a time.AfterFunc and
// a comment explaining the order things happen in. Here the order is the
// declaration: `conductor check` prints the machine, gen/mission.dot draws
// it, and the dashboard shows which step is running.
package main

import (
	"errors"
	"fmt"
	"log/slog"

	"conductor.dev/conductor"
	_ "conductor.dev/conductor/transport/zenoh"
)

//conductor:node
type Mission struct {
	Fib conductor.ActionClient[FibonacciGoal, FibonacciFeedback, FibonacciResult] `action:"fibonacci" timeout:"60s"`

	Run     conductor.Mission `start:"send"`
	Send    conductor.Step    `next:"follow" retry:"2" backoff:"1s" fail:"abandon"`
	Follow  conductor.Step    `next:"report" timeout:"20s" fail:"give_up"`
	GiveUp  conductor.Step    `next:"report"`
	Report  conductor.Step    `next:"done"`
	Abandon conductor.Step    `next:"failed"`

	// Step handlers run one at a time on the mission's own goroutine, so
	// state passed between steps needs no locking. State shared with the
	// node's callbacks would: that is what Task.Do is for.
	goal   *conductor.GoalHandle[FibonacciFeedback, FibonacciResult]
	result FibonacciResult
	status conductor.GoalStatus
}

// OnSend asks the server to start. Retrying is a tag, not a loop: the server
// may not have finished coming up.
func (m *Mission) OnSend(t *conductor.Task) error {
	goal := FibonacciGoal{Order: 8}
	slog.Info("sending goal", "order", goal.Order, "attempt", t.Attempt())
	h, err := m.Fib.SendGoal(goal)
	if err != nil {
		return fmt.Errorf("sending goal: %w", err)
	}
	m.goal = h
	slog.Info("goal accepted", "id", h.ID())
	return nil
}

// OnFollow streams feedback until the result arrives. The 20s timeout tag
// replaces the timer-and-flag this used to be: when it expires the step's
// context is canceled, Result returns, and the fail branch runs.
func (m *Mission) OnFollow(t *conductor.Task) error {
	go func() {
		for fb := range m.goal.Feedback() {
			slog.Info("feedback", "sequence", fb.Sequence)
		}
	}()
	res, status, err := m.goal.Result()
	if err != nil {
		return fmt.Errorf("waiting for result: %w", err)
	}
	m.result, m.status = res, status
	return nil
}

// OnGiveUp cancels a goal that overran, then reports what there is to report.
func (m *Mission) OnGiveUp(t *conductor.Task) error {
	slog.Warn("goal is taking too long, cancelling", "because", t.Err())
	if err := m.goal.Cancel(); err != nil {
		slog.Error("cancel failed", "err", err)
	}
	return nil
}

// OnReport ends the application: a mission that has finished should stop,
// not idle.
func (m *Mission) OnReport(t *conductor.Task) error {
	slog.Info("goal finished", "status", m.status.String(), "sequence", m.result.Sequence)
	conductor.Shutdown()
	return nil
}

// OnAbandon is the failure exit: no server answered, so there is nothing to
// wait for and the process should say so.
func (m *Mission) OnAbandon(t *conductor.Task) error {
	conductor.Abort(errors.New("could not start the goal: " + t.Err().Error()))
	return nil
}

func main() {
	conductor.Run(&Mission{})
}

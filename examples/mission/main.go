// Mission is the conductor action-client example: it drives the standard
// example_interfaces/action/Fibonacci action, streaming feedback and
// cancelling the goal if it takes too long. It works against any ROS 2
// action server — the Go one in examples/fibonacci, an rclpy server, or a
// real stack like Nav2.
//
//	mission -transport zenoh
package main

import (
	"fmt"
	"log/slog"
	"time"

	"conductor.dev/conductor"
	_ "conductor.dev/conductor/transport/zenoh"
)

//conductor:node
type Mission struct {
	Start conductor.Timer                                                           `rate:"1hz"`
	Fib   conductor.ActionClient[FibonacciGoal, FibonacciFeedback, FibonacciResult] `action:"fibonacci" timeout:"60s"`

	started bool
}

// OnStart kicks the mission off once. Goal calls block, so they run on their
// own goroutine rather than on the node's executor.
func (m *Mission) OnStart() {
	if m.started {
		return
	}
	m.started = true
	go m.run()
}

func (m *Mission) run() {
	goal := FibonacciGoal{Order: 8}
	slog.Info("sending goal", "order", goal.Order)
	h, err := m.Fib.SendGoal(goal)
	if err != nil {
		conductor.Abort(fmt.Errorf("sending goal: %w", err))
		return
	}
	slog.Info("goal accepted", "id", h.ID())

	// Cancel if the server takes an unreasonable amount of time.
	deadline := time.AfterFunc(20*time.Second, func() {
		slog.Warn("goal is taking too long, cancelling")
		if err := h.Cancel(); err != nil {
			slog.Error("cancel failed", "err", err)
		}
	})
	defer deadline.Stop()

	go func() {
		for fb := range h.Feedback() {
			slog.Info("feedback", "sequence", fb.Sequence)
		}
	}()

	res, status, err := h.Result()
	if err != nil {
		conductor.Abort(fmt.Errorf("waiting for result: %w", err))
		return
	}
	slog.Info("goal finished", "status", status.String(), "sequence", res.Sequence)
	conductor.Shutdown()
}

func main() {
	conductor.Run(&Mission{})
}

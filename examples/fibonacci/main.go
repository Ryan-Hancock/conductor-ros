// Fibonacci is the conductor action-server example: it serves the standard
// example_interfaces/action/Fibonacci action, streaming the sequence as
// feedback. Exercise it from ROS with:
//
//	ros2 action send_goal --feedback /fibonacci example_interfaces/action/Fibonacci "{order: 6}"
package main

import (
	"time"

	"conductor.dev/conductor"
	_ "conductor.dev/conductor/transport/zenoh"
)

//conductor:node
type FibServer struct {
	Fib conductor.Action[FibonacciGoal, FibonacciFeedback, FibonacciResult] `action:"fibonacci"`
}

// OnFib computes the sequence one step per half second, publishing feedback
// as it goes. It runs on its own goroutine per goal; cancellation arrives
// via the goal context.
func (f *FibServer) OnFib(g *conductor.Goal[FibonacciGoal, FibonacciFeedback]) (FibonacciResult, error) {
	seq := []int32{0, 1}
	for i := int32(1); i < g.Value().Order; i++ {
		select {
		case <-g.Context().Done():
			return FibonacciResult{Sequence: seq}, g.Context().Err()
		case <-time.After(500 * time.Millisecond):
		}
		seq = append(seq, seq[len(seq)-1]+seq[len(seq)-2])
		g.Feedback(FibonacciFeedback{Sequence: seq})
	}
	return FibonacciResult{Sequence: seq}, nil
}

func main() {
	conductor.Run(&FibServer{})
}

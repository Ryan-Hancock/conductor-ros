// MoveIt drives a manipulator from conductor: it brings the arm to a named
// configuration, picks an object off a table, places it, and goes home — all
// through MoveIt's own move_action, with the planning groups and named poses
// taken from the robot's SRDF rather than spelled out here.
//
// It is the second half of the claim examples/nav2 makes for navigation: the
// 20% — planning, kinematics, collision checking — stays in MoveIt, and what
// conductor takes over is deciding what to plan next, in what order, and what
// to do when a plan fails.
//
// Two ways to run it:
//
//	make moveit                             # the stand-in move_group
//	conductor run examples/moveit -env real # a real move_group, launched separately
//
// The manipulation logic is a mission, so `conductor check` prints the machine
// and the dashboard shows which step the arm is on.
package main

import (
	"conductor.dev/conductor"
	_ "conductor.dev/conductor/transport/zenoh"
)

func main() {
	conductor.Run(&Commander{})
}

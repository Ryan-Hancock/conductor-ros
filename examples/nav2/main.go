// Nav2 drives a real Nav2 stack from conductor: it brings the navigation
// nodes up through their lifecycle manager, seeds AMCL with an initial pose,
// patrols a route of waypoints with navigate_to_pose, falls back on Nav2's
// own backup and spin behaviours when a goal is aborted, and takes itself to
// the dock when the battery runs low.
//
// This is conductor's thesis under test. The 20% — the planner, the
// controller, the costmaps, the behaviour tree — stays in Nav2, where years
// of work already live. What conductor takes over is the part that is
// otherwise spread across a launch file, a lifecycle_manager, a behaviour
// tree XML and a Python node: deciding what the stack should be doing next,
// and in what order.
//
// Two ways to run it:
//
//	make nav2                              # the stand-in stack in examples/nav2stub
//	conductor run examples/nav2 -env sim    # a real nav2_bringup, turtlebot3 in Gazebo
//
// The stand-in exists because a Nav2 install is a large thing to ask of
// someone reading an example. It answers the same interfaces, by the same
// type hashes, so the application does not know which one it is talking to —
// and neither does `conductor check`.
package main

import (
	"conductor.dev/conductor"
	_ "conductor.dev/conductor/transport/zenoh"
)

func main() {
	conductor.Run(&Commander{}, &Watchdog{})
}

// Patrol is the Conductor example application: a patroller drives a route of
// waypoints as a declared mission, a simulated localizer feeds a navigator
// steering toward the current one, and a safety monitor watches the command
// stream and an externally published e-stop. Its geometry is declared in
// frames.json and published on tf_static.
package main

import (
	"conductor.dev/conductor"
	"conductor.dev/conductor/msgs"
	_ "conductor.dev/conductor/transport/zenoh"
)

func main() {
	conductor.Run(
		&Localizer{},
		&Patroller{},
		&Navigator{goal: msgs.Point{X: 5, Y: 4}},
		&SafetyMonitor{},
	)
}

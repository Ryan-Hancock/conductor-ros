// Patrol is the Conductor example application: a simulated localizer feeds a
// navigator steering toward a goal, with a safety monitor watching the
// command stream and an externally published e-stop.
package main

import (
	"conductor.dev/conductor"
	"conductor.dev/conductor/msgs"
	_ "conductor.dev/conductor/transport/zenoh"
)

func main() {
	conductor.Run(
		&Localizer{},
		&Navigator{goal: msgs.Point{X: 5, Y: 4}},
		&SafetyMonitor{},
	)
}

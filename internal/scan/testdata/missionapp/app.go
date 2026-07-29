// Sample app used by the mission and frame validation tests. It contains
// deliberate defects: a next: naming a step that does not exist, a Goto to
// another one, an unreachable step, an invalid retry count, a frame tag
// naming an undeclared frame, a frame tag on a message with no header, and a
// static lookup across a dynamic transform.
package main

import (
	"conductor.dev/conductor"
	"conductor.dev/conductor/msgs"
)

//ros:type sample_msgs/msg/Reading
type Reading struct {
	Value float64
}

//ros:type sample_msgs/msg/Cloud
type Cloud struct {
	Header msgs.Header
	Points []float64
}

//conductor:node
type Courier struct {
	Trip conductor.Mission `start:"pickup"`

	Pickup   conductor.Step `next:"transit"`
	Transit  conductor.Step `next:"dropof" fail:"recharge" timeout:"2m"`
	Dropoff  conductor.Step `next:"done"`
	Recharge conductor.Step `next:"transit" retry:"soon"`
	Idle     conductor.Step `next:"done"`
}

func (c *Courier) OnPickup(t *conductor.Task) error { return nil }

func (c *Courier) OnTransit(t *conductor.Task) error {
	if t.Attempt() > 1 {
		return t.Goto("nowhere")
	}
	return nil
}

func (c *Courier) OnDropoff(t *conductor.Task) error  { return nil }
func (c *Courier) OnRecharge(t *conductor.Task) error { return nil }
func (c *Courier) OnIdle(t *conductor.Task) error     { return nil }

//conductor:node
type Perception struct {
	Cloud   conductor.Pub[Cloud]   `topic:"cloud" frame:"camera"`
	Reading conductor.Pub[Reading] `topic:"reading" frame:"laser"`
	Scan    conductor.Sub[Cloud]   `topic:"scan" frame:"laser"`
	TF      conductor.TF
}

func (p *Perception) OnScan(c Cloud) {
	if at, err := p.TF.Lookup("map", "laser"); err == nil {
		_ = at
	}
}

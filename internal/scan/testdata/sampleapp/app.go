// Sample app used by scanner and validation tests. It contains deliberate
// defects: a QoS mismatch on "battery", a subscription to "lidar" that
// nothing publishes, and a missing OnEstop handler.
package main

import (
	"conductor.dev/conductor"
	"conductor.dev/conductor/msgs"
)

//ros:type sample_msgs/msg/Battery
type Battery struct {
	Voltage float64
}

//conductor:node
type Sensor struct {
	Tick conductor.Timer        `rate:"10hz"`
	Batt conductor.Pub[Battery] `topic:"battery" qos:"sensor"`
}

func (s *Sensor) OnTick() {}

//conductor:node
type Monitor struct {
	Batt      conductor.Sub[Battery]   `topic:"battery" qos:"reliable"`
	Estop     conductor.Sub[msgs.Bool] `topic:"estop" qos:"transient"`
	Lidar     conductor.Sub[Battery]   `topic:"lidar" qos:"sensor"`
	Threshold conductor.Param[float64] `name:"low_voltage" default:"11.1"`
}

func (m *Monitor) OnBatt(b Battery) {}

func (m *Monitor) OnLidar(b Battery) {}

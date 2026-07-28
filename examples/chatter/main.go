// Chatter is a minimal interop example: it subscribes to the classic ROS 2
// /chatter topic (published by e.g. `ros2 topic pub /chatter
// std_msgs/msg/String "{data: hi}"`) and logs what it hears. Run it against
// a live ROS graph with:
//
//	chatter -transport zenoh
package main

import (
	"log/slog"

	"conductor.dev/conductor"
	"conductor.dev/conductor/msgs"
	_ "conductor.dev/conductor/transport/zenoh"
)

//conductor:node
type Listener struct {
	Chatter conductor.Sub[msgs.String] `topic:"chatter" qos:"reliable"`
}

func (l *Listener) OnChatter(m msgs.String) {
	slog.Info("heard", "data", m.Data)
}

func main() {
	conductor.Run(&Listener{})
}

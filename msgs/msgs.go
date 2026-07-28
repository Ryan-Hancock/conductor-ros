// Package msgs provides Go representations of common ROS 2 interface types.
//
// Each type carries a //ros:type directive naming its ROS interface; the
// conductor CLI reads these when building the static graph, and user packages
// can declare their own message types the same way. v0.1 ships a small
// hand-written set; generating this package from .msg/.idl definitions is on
// the roadmap (see DESIGN.md).
package msgs

import "time"

//ros:type std_msgs/msg/Header
type Header struct {
	Stamp   time.Time
	FrameID string
}

//ros:type std_msgs/msg/Bool
type Bool struct {
	Data bool
}

//ros:type std_msgs/msg/String
type String struct {
	Data string
}

//ros:type geometry_msgs/msg/Vector3
type Vector3 struct {
	X, Y, Z float64
}

//ros:type geometry_msgs/msg/Point
type Point struct {
	X, Y, Z float64
}

//ros:type geometry_msgs/msg/Quaternion
type Quaternion struct {
	X, Y, Z, W float64
}

//ros:type geometry_msgs/msg/Pose
type Pose struct {
	Position    Point
	Orientation Quaternion
}

//ros:type geometry_msgs/msg/PoseStamped
type PoseStamped struct {
	Header Header
	Pose   Pose
}

//ros:type geometry_msgs/msg/Twist
type Twist struct {
	Linear  Vector3
	Angular Vector3
}

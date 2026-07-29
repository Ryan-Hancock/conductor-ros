package conductor

import "time"

// Wire types for tf2. Like the lifecycle and action wire types they are
// hand-written in this package rather than generated, because the runtime
// publishes them and generated message packages import this one; the hashes
// are checked against the installed distro in tf_test.go.

type headerMsg struct {
	Stamp   time.Time
	FrameId string
}

type vector3Msg struct {
	X, Y, Z float64
}

type quaternionMsg struct {
	X, Y, Z, W float64
}

type transformMsg struct {
	Translation vector3Msg
	Rotation    quaternionMsg
}

type transformStampedMsg struct {
	Header       headerMsg
	ChildFrameId string
	Transform    transformMsg
}

type tfMessageMsg struct {
	Transforms []transformStampedMsg
}

// RIHS01 hashes from the ROS 2 Lyrical type descriptions.
const tfMessageHash = "RIHS01_e369d0f05a23ae52508854b66f6aa0437f3449d652e8cbf22d5abe85d020f087"

func init() {
	RegisterMessage[tfMessageMsg]("tf2_msgs/msg/TFMessage", tfMessageHash)
}

// tfStatic renders declared transforms as one TFMessage.
func tfStatic(transforms []Transform, stamp time.Time) tfMessageMsg {
	msg := tfMessageMsg{Transforms: make([]transformStampedMsg, 0, len(transforms))}
	for _, tf := range transforms {
		q := quatFromRPY(tf.RPY)
		msg.Transforms = append(msg.Transforms, transformStampedMsg{
			Header:       headerMsg{Stamp: stamp, FrameId: tf.Parent},
			ChildFrameId: tf.Child,
			Transform: transformMsg{
				Translation: vector3Msg{X: tf.XYZ[0], Y: tf.XYZ[1], Z: tf.XYZ[2]},
				Rotation:    quaternionMsg{X: q[0], Y: q[1], Z: q[2], W: q[3]},
			},
		})
	}
	return msg
}

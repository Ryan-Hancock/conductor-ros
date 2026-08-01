package conductor

import (
	"log/slog"
	"os"
	"reflect"
)

// Every ROS tool that draws a robot — rviz, MoveIt, a joint state publisher —
// finds its model on `/robot_description`: a latched topic carrying the URDF as
// a string. Nothing derives it, nothing checks it; it is simply expected to be
// there, and on most robots it is there because robot_state_publisher was
// launched with the file.
//
// Conductor already reads that file: `conductor frames -from robot.urdf` derives
// the transform tree from it. So the application can publish it too, and a
// conductor-only robot becomes legible to tools that know nothing about
// conductor.
//
// The question worth getting right is *when*. A robot with a
// robot_state_publisher already has this topic, and a second latched publisher
// on it is the same fault as two static transforms for one child. The transform
// tree answers it: the same declaration that says "these transforms are ours to
// publish" says the description is ours too. So this is published exactly when
// tf_static is — by the node that publishes the tree, when it goes active,
// latched — and not at all when the tree is attributed to somebody else.

// descriptionTopic is where ROS 2 expects a robot's model.
const descriptionTopic = "robot_description"

// stringMsg is std_msgs/msg/String, declared here rather than imported so the
// runtime does not depend on the msgs package.
type stringMsg struct {
	Data string
}

const stringHash = "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"

func init() {
	RegisterMessage[stringMsg]("std_msgs/msg/String", stringHash)
}

// loadDescription reads a robot description to publish. A missing file is not
// an error: most applications are not the robot's description owner.
func loadDescription(path string) (string, error) {
	b, err := os.ReadFile(path)
	if err != nil {
		if os.IsNotExist(err) {
			return "", nil
		}
		return "", err
	}
	return string(b), nil
}

// publishDescription wires the latched /robot_description publisher onto the
// node that owns the transform tree. It reports whether it did.
func publishDescription(rt *runtimeState, nr *nodeRuntime) bool {
	if rt.description == "" || rt.descriptionPublisher != "" {
		return false
	}
	q, _ := QoSProfile("transient")
	publish, err := rt.transport.Publisher(TopicSpec{
		Topic: descriptionTopic, QoS: q, Type: reflect.TypeFor[stringMsg](), Node: nr.name,
	})
	if err != nil {
		slog.Warn("conductor: cannot publish the robot description", "node", nr.name, "err", err)
		return false
	}
	rt.descriptionPublisher = nr.name
	rt.recordProvides(nr.name, descriptionTopic)
	rt.recordEndpoint(Endpoint{
		Node: nr.name, Kind: EndpointPub, Field: "RobotDescription", Name: descriptionTopic,
		Type: "std_msgs/msg/String", QoS: q.Name,
	})

	description := rt.description
	// Published once, when the node goes active, and latched — the same
	// treatment tf_static gets, for the same reason: a tool started later needs
	// the model, and a model is state rather than an event.
	nr.onActive = append(nr.onActive, func() {
		nr.enqueue(func() {
			if !nr.active() {
				return
			}
			if err := publish(stringMsg{Data: description}, Metadata{}); err != nil {
				slog.Warn("conductor: robot_description publish failed", "node", nr.name, "err", err)
				return
			}
			slog.Info("conductor: published the robot description",
				"node", nr.name, "topic", descriptionTopic, "bytes", len(description))
		})
	})
	return true
}

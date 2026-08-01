package conductor

import (
	"reflect"
	"sync/atomic"
	"testing"
	"time"
)

// heldSeen records what the watcher below received, from the executor.
var heldSeen atomic.Int64

// Latching is what "transient local" means: a subscriber that starts after the
// message was published still gets it. /tf_static is the reason it matters —
// the transform tree is published once, when the robot starts, and an rviz
// opened an hour later still has to learn where the lidar is.

type latchedMsg struct{ N int }

func transientSpec(topic, node string) TopicSpec {
	q, _ := QoSProfile("transient")
	return TopicSpec{Topic: topic, QoS: q, Type: reflect.TypeFor[latchedMsg](), Node: node}
}

func volatileSpec(topic, node string) TopicSpec {
	q, _ := QoSProfile("reliable")
	return TopicSpec{Topic: topic, QoS: q, Type: reflect.TypeFor[latchedMsg](), Node: node}
}

func TestTransientLocalReachesALateSubscriber(t *testing.T) {
	bus := newInproc()
	publish, err := bus.Publisher(transientSpec("tree", "publisher"))
	if err != nil {
		t.Fatal(err)
	}
	if err := publish(latchedMsg{N: 7}, Metadata{}); err != nil {
		t.Fatal(err)
	}

	// Nobody was listening; the message is kept anyway.
	var got []latchedMsg
	err = bus.Subscribe(transientSpec("tree", "late"), func(m any, _ Metadata) {
		got = append(got, m.(latchedMsg))
	})
	if err != nil {
		t.Fatal(err)
	}
	if len(got) != 1 || got[0].N != 7 {
		t.Fatalf("late subscriber got %v, want the message published before it existed", got)
	}

	// And it still receives what comes next, in order.
	if err := publish(latchedMsg{N: 8}, Metadata{}); err != nil {
		t.Fatal(err)
	}
	if len(got) != 2 || got[1].N != 8 {
		t.Fatalf("got %v, want the history followed by the live message", got)
	}
}

// Depth is how much history is kept: conductor's transient profile keeps one,
// which is what a latched tree needs.
func TestTransientLocalKeepsTheProfileDepth(t *testing.T) {
	bus := newInproc()
	spec := transientSpec("tree", "publisher")
	if spec.QoS.Depth != 1 {
		t.Fatalf("the transient profile keeps %d, want 1", spec.QoS.Depth)
	}
	publish, err := bus.Publisher(spec)
	if err != nil {
		t.Fatal(err)
	}
	for i := 1; i <= 3; i++ {
		if err := publish(latchedMsg{N: i}, Metadata{}); err != nil {
			t.Fatal(err)
		}
	}

	var got []latchedMsg
	if err := bus.Subscribe(transientSpec("tree", "late"), func(m any, _ Metadata) {
		got = append(got, m.(latchedMsg))
	}); err != nil {
		t.Fatal(err)
	}
	if len(got) != 1 || got[0].N != 3 {
		t.Fatalf("got %v, want only the most recent message", got)
	}
}

// A volatile topic is not kept, and a volatile subscriber does not ask for
// history even when the publisher has some: requesting less durability than is
// offered is legal, and it means "only what happens from now on".
func TestVolatileEndpointsDoNotLatch(t *testing.T) {
	bus := newInproc()
	loud, err := bus.Publisher(volatileSpec("chatter", "publisher"))
	if err != nil {
		t.Fatal(err)
	}
	if err := loud(latchedMsg{N: 1}, Metadata{}); err != nil {
		t.Fatal(err)
	}
	var got []latchedMsg
	if err := bus.Subscribe(volatileSpec("chatter", "late"), func(m any, _ Metadata) {
		got = append(got, m.(latchedMsg))
	}); err != nil {
		t.Fatal(err)
	}
	if len(got) != 0 {
		t.Fatalf("a volatile subscriber received %v from before it existed", got)
	}

	latched, err := bus.Publisher(transientSpec("tree", "publisher"))
	if err != nil {
		t.Fatal(err)
	}
	if err := latched(latchedMsg{N: 2}, Metadata{}); err != nil {
		t.Fatal(err)
	}
	got = nil
	if err := bus.Subscribe(volatileSpec("tree", "volatile"), func(m any, _ Metadata) {
		got = append(got, m.(latchedMsg))
	}); err != nil {
		t.Fatal(err)
	}
	if len(got) != 0 {
		t.Fatalf("a volatile subscriber asked for history and got %v", got)
	}
}

// The transform tree is published once, when the node goes active, and stays
// available: no republishing timer, and a node wired afterwards still sees it.
func TestStaticTransformsAreLatchedNotRepublished(t *testing.T) {
	tree := &FrameTree{Path: "frames.json", Transforms: []Transform{
		{Parent: "base_link", Child: "laser", XYZ: [3]float64{0.12, 0, 0.19}},
	}}
	ta, err := NewTestApp(TestOptions{Frames: tree}, &framePublisher{})
	if err != nil {
		t.Fatal(err)
	}
	defer ta.Close()
	ta.Settle() // the tree is published from the executor when the node goes active

	// No timer was registered for the tree: it is published on activation and
	// kept by the transport, not resent every second.
	for _, th := range ta.a.rt.timers {
		if th.node.name == "frame_publisher" {
			t.Errorf("the transform publisher registered a timer; tf_static is latched now")
		}
	}

	var seen []tfMessageMsg
	q, _ := QoSProfile("transient")
	err = ta.a.rt.transport.Subscribe(TopicSpec{
		Topic: tfStaticTopic, QoS: q, Type: reflect.TypeFor[tfMessageMsg](), Node: "late",
	}, func(m any, _ Metadata) { seen = append(seen, m.(tfMessageMsg)) })
	if err != nil {
		t.Fatal(err)
	}
	if len(seen) != 1 {
		t.Fatalf("a subscriber wired after startup received %d tf_static messages, want the latched one", len(seen))
	}
	if len(seen[0].Transforms) != 1 || seen[0].Transforms[0].ChildFrameId != "laser" {
		t.Errorf("latched tree = %+v", seen[0].Transforms)
	}
	// Waiting does not produce another: the 1 Hz republish is gone.
	time.Sleep(50 * time.Millisecond)
	ta.Settle()
	if len(seen) != 1 {
		t.Errorf("received %d messages, want no republishing", len(seen))
	}
}

type framePublisher struct {
	TF TF
}

// A latched message arrives when the subscription is declared, which is before
// the node is active. An event dropped there is gone for good — but a
// transient-local message is state, still true, and not about to be sent again,
// so it is held and delivered on activation. Without this, a conductor node
// never receives a latched topic at all.
func TestLatchedMessageSurvivesAnInactiveNode(t *testing.T) {
	ta, err := NewTestApp(TestOptions{ManualLifecycle: true}, &estopWatcher{})
	if err != nil {
		t.Fatal(err)
	}
	defer ta.Close()

	// Published while the node is still unconfigured, as a latched publisher
	// that started before it would have done.
	q, _ := QoSProfile("transient")
	publish, err := ta.a.rt.transport.Publisher(TopicSpec{
		Topic: "estop", QoS: q, Type: reflect.TypeFor[latchedMsg](), Node: "outside",
	})
	if err != nil {
		t.Fatal(err)
	}
	if err := publish(latchedMsg{N: 42}, Metadata{}); err != nil {
		t.Fatal(err)
	}

	for _, tr := range []Transition{TransitionConfigure, TransitionActivate} {
		if err := ta.Transition("estop_watcher", tr); err != nil {
			t.Fatal(err)
		}
	}
	ta.Settle()

	if seen := heldSeen.Load(); seen != 42 {
		t.Fatalf("the node saw %d after activating, want the latched 42", seen)
	}
}

type estopWatcher struct {
	Estop Sub[latchedMsg] `topic:"estop" qos:"transient"`
}

func (e *estopWatcher) OnEstop(m latchedMsg) { heldSeen.Store(int64(m.N)) }

// A robot's model belongs on /robot_description, latched, so that a tool
// started later can draw it. It is published exactly when the transform tree is
// — by the application that owns the description — because a second latched
// publisher on that topic is the same fault as two static transforms for one
// child.
func TestRobotDescriptionIsPublishedWithTheTree(t *testing.T) {
	const urdf = `<?xml version="1.0"?><robot name="test"><link name="base_link"/></robot>`
	ours := &FrameTree{Path: "frames.json", Transforms: []Transform{
		{Parent: "base_link", Child: "laser", XYZ: [3]float64{0.1, 0, 0.2}},
	}}
	ta, err := NewTestApp(TestOptions{Frames: ours, Description: urdf}, &framePublisher{})
	if err != nil {
		t.Fatal(err)
	}
	defer ta.Close()
	ta.Settle()

	var seen []stringMsg
	q, _ := QoSProfile("transient")
	err = ta.a.rt.transport.Subscribe(TopicSpec{
		Topic: descriptionTopic, QoS: q, Type: reflect.TypeFor[stringMsg](), Node: "rviz",
	}, func(m any, _ Metadata) { seen = append(seen, m.(stringMsg)) })
	if err != nil {
		t.Fatal(err)
	}
	if len(seen) != 1 {
		t.Fatalf("a tool started afterwards received %d descriptions, want the latched one", len(seen))
	}
	if seen[0].Data != urdf {
		t.Errorf("published %q, want the description verbatim", seen[0].Data)
	}
}

// A robot whose transforms are somebody else's publishes no description: that
// robot has a robot_state_publisher, and it is already publishing this topic.
func TestRobotDescriptionIsNotPublishedWhenTheTreeIsNotOurs(t *testing.T) {
	theirs := &FrameTree{Path: "frames.json", Transforms: []Transform{
		{Parent: "base_link", Child: "laser", XYZ: [3]float64{0.1, 0, 0.2}, By: "robot_state_publisher"},
	}}
	ta, err := NewTestApp(TestOptions{
		Frames: theirs, Description: `<robot name="test"/>`,
	}, &framePublisher{})
	if err != nil {
		t.Fatal(err)
	}
	defer ta.Close()
	ta.Settle()

	var seen int
	q, _ := QoSProfile("transient")
	err = ta.a.rt.transport.Subscribe(TopicSpec{
		Topic: descriptionTopic, QoS: q, Type: reflect.TypeFor[stringMsg](), Node: "rviz",
	}, func(any, Metadata) { seen++ })
	if err != nil {
		t.Fatal(err)
	}
	if seen != 0 {
		t.Errorf("published %d description(s) for a robot whose model belongs to robot_state_publisher", seen)
	}
}

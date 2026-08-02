package conductor

import (
	"fmt"
	"log/slog"
	"reflect"
	"sync"
	"sync/atomic"
	"time"
)

// tfStaticTopic is where ROS 2 expects latched static transforms.
const tfStaticTopic = "tf_static"

// TF gives a node access to the application's declared transform tree: the
// frames it may name, and the static transforms between them.
//
//	//conductor:node
//	type Perception struct {
//	    Scan  conductor.Sub[LaserScan]   `topic:"scan" frame:"laser"`
//	    Cloud conductor.Pub[PointCloud2] `topic:"cloud" frame:"base_link"`
//	    TF    conductor.TF
//	}
//
//	func (p *Perception) OnScan(s LaserScan) {
//	    at, err := p.TF.Lookup("base_link", "laser")   // checked by conductor check
//	    ...
//	}
//
// Declaring TF also makes this process publish the static transforms on
// tf_static, so there is no static_transform_publisher to launch and no
// chance of the launch file and the code disagreeing about where the lidar is.
type TF struct {
	tree      *FrameTree
	node      *nodeRuntime
	published atomic.Uint64
}

// Lookup returns the transform taking points in the source frame to the
// target frame. It resolves the declared static links only: a path that
// crosses a transform someone else publishes at runtime is an error that says
// so, rather than a silently wrong answer.
func (f *TF) Lookup(target, source string) (Isometry, error) {
	if f.tree == nil {
		return Isometry{}, fmt.Errorf("no transform tree is declared (add frames.json beside conductor.json, or pass -frames <file>)")
	}
	return f.tree.Lookup(target, source)
}

// Frames lists the declared frames.
func (f *TF) Frames() []string { return f.tree.Frames() }

// Tree returns the declared tree, for the rare code that wants to walk it.
func (f *TF) Tree() *FrameTree { return f.tree }

func (f *TF) bind(rt *runtimeState, nr *nodeRuntime, field reflect.StructField, ownerPtr reflect.Value) error {
	f.tree, f.node = rt.frames, nr
	// Only what this application owns: a fixed transform attributed to somebody
	// else is theirs to publish, and publishing it too would put two static
	// transforms with the same child on tf_static.
	statics := rt.frames.Published()
	rt.recordEndpoint(Endpoint{
		Node: nr.name, Kind: EndpointTF, Field: field.Name, Name: tfStaticTopic,
		Type: "tf2_msgs/msg/TFMessage", Rate: describeFrames(rt.frames),
		count: countOf(f.published.Load),
	})
	if len(statics) == 0 || rt.tfPublisher != "" {
		// Another node in this process already publishes the tree; one
		// publisher per process is enough, and duplicate static transforms
		// are noise on the graph.
		return nil
	}
	rt.tfPublisher = nr.name
	// The same declaration that says these transforms are ours to publish says
	// the description they came from is ours too.
	publishDescription(rt, nr)

	q, _ := QoSProfile("transient")
	publish, err := rt.transport.Publisher(TopicSpec{
		Topic: tfStaticTopic, QoS: q, Type: reflect.TypeFor[tfMessageMsg](), Node: nr.name,
	})
	if err != nil {
		return err
	}
	send := func() {
		if !nr.active() {
			return
		}
		if err := publish(tfStatic(statics, rt.clock.Now()), Metadata{}); err != nil {
			slog.Warn("conductor: tf_static publish failed", "node", nr.name, "err", err)
			return
		}
		f.published.Add(1)
	}
	rt.recordProvides(nr.name, tfStaticTopic)
	// Published once, when the node becomes active, and latched: the topic is
	// transient-local, so a subscriber that starts later asks for it and gets
	// it. This used to be a 1 Hz republish, which is what a transport without
	// durability leaves you doing — a late joiner waited up to a second, and
	// every other second of the robot's life carried a message nobody needed.
	nr.onActive = append(nr.onActive, func() { nr.enqueue(send) })
	slog.Info("conductor: publishing static transforms", "node", nr.name, "count", len(statics))
	return nil
}

func describeFrames(t *FrameTree) string {
	if t == nil {
		return ""
	}
	return fmt.Sprintf("%d published, %d fixed, %d dynamic",
		len(t.Published()), len(t.Fixed()), len(t.Transforms)-len(t.Fixed()))
}

// --- frame stamping and checking -------------------------------------------
//
// A frame tag says which frame an endpoint's messages are in. On a publisher
// the runtime fills the header with it, so the declaration is what reaches the
// wire; on a subscription it verifies incoming messages, so a peer sending
// another frame is reported once, by name, instead of quietly corrupting a
// transform later.

// headerAccess is how to reach a message type's std_msgs/Header fields.
type headerAccess struct {
	header []int // field index path to Header
	frame  []int // index path to the frame id string
	stamp  []int // index path to the timestamp, nil if absent
}

var (
	headerMu    sync.Mutex
	headerCache = map[reflect.Type]*headerAccess{}
)

// headerOf finds a message type's Header, or nil if it has none. Results are
// cached: this is reflection on the hot path otherwise.
func headerOf(t reflect.Type) *headerAccess {
	headerMu.Lock()
	defer headerMu.Unlock()
	if a, ok := headerCache[t]; ok {
		return a
	}
	a := findHeader(t)
	headerCache[t] = a
	return a
}

func findHeader(t reflect.Type) *headerAccess {
	if t.Kind() != reflect.Struct {
		return nil
	}
	hf, ok := t.FieldByName("Header")
	if !ok || hf.Type.Kind() != reflect.Struct {
		return nil
	}
	a := &headerAccess{header: hf.Index}
	for _, name := range []string{"FrameID", "FrameId", "FrameID_"} {
		if f, ok := hf.Type.FieldByName(name); ok && f.Type.Kind() == reflect.String {
			a.frame = append(append([]int{}, hf.Index...), f.Index...)
			break
		}
	}
	if a.frame == nil {
		return nil
	}
	if f, ok := hf.Type.FieldByName("Stamp"); ok && f.Type == reflect.TypeFor[time.Time]() {
		a.stamp = append(append([]int{}, hf.Index...), f.Index...)
	}
	return a
}

// stampFrame fills an outgoing message's frame id and timestamp, leaving
// anything the handler set itself alone.
func (a *headerAccess) stampFrame(v reflect.Value, frame string, clock Clock) {
	f := v.FieldByIndex(a.frame)
	if f.String() == "" {
		f.SetString(frame)
	}
	if a.stamp != nil {
		s := v.FieldByIndex(a.stamp)
		if s.Interface().(time.Time).IsZero() {
			// The robot's clock, not the machine's: a stamp in wall time on a
			// simulated robot is what makes every consumer's transform lookup
			// fail by hours.
			s.Set(reflect.ValueOf(clock.Now()))
		}
	}
}

// frameOf reads an incoming message's frame id.
func (a *headerAccess) frameOf(v reflect.Value) string { return v.FieldByIndex(a.frame).String() }

// frameChecker reports messages arriving in a frame the subscription did not
// declare — once per offending frame, because a mismatched peer sends at the
// topic's rate and the log is not the place to learn that twice a second.
type frameChecker struct {
	node, topic, want string
	access            *headerAccess

	mu   sync.Mutex
	seen map[string]bool
}

func newFrameChecker(node, topic, want string, a *headerAccess) *frameChecker {
	return &frameChecker{node: node, topic: topic, want: want, access: a, seen: map[string]bool{}}
}

func (c *frameChecker) check(msg any) {
	got := c.access.frameOf(reflect.ValueOf(msg))
	if got == "" || got == c.want {
		return
	}
	counter("conductor_frame_mismatches_total", "node", c.node, "topic", c.topic, "frame", got).Add(1)
	c.mu.Lock()
	first := !c.seen[got]
	c.seen[got] = true
	c.mu.Unlock()
	if first {
		slog.Warn("conductor: message arrived in an unexpected frame",
			"node", c.node, "topic", c.topic, "declared", c.want, "received", got)
	}
}

// frameTag resolves an endpoint's frame tag against the declared tree,
// refusing frames the tree does not have and types that cannot carry one.
func frameTag(rt *runtimeState, field reflect.StructField, t reflect.Type) (string, *headerAccess, error) {
	frame := field.Tag.Get("frame")
	if frame == "" {
		return "", nil, nil
	}
	a := headerOf(t)
	if a == nil {
		return "", nil, fmt.Errorf("frame:%q, but %s has no std_msgs/Header field to carry it", frame, t)
	}
	if rt.frames != nil && !rt.frames.Has(frame) {
		return "", nil, fmt.Errorf("frame %q is not declared in %s", frame, rt.frames.Path)
	}
	return frame, a, nil
}

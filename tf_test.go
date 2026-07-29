package conductor

import (
	"os"
	"strings"
	"testing"
	"time"

	"conductor.dev/conductor/internal/msggen"
)

// Verifies the hardcoded tf2 wire hash against the installed ROS distro
// (skipped when none is present).
func TestTFWireHashes(t *testing.T) {
	share := "/opt/ros/lyrical/share"
	if _, err := os.Stat(share); err != nil {
		t.Skip("no ROS distro installed")
	}
	r := msggen.NewResolver([]string{share})
	td, err := r.Describe("tf2_msgs/msg/TFMessage")
	if err != nil {
		t.Fatal(err)
	}
	if got := td.Hash(); got != tfMessageHash {
		t.Errorf("tf2_msgs/msg/TFMessage:\n got %s\nwant hardcoded %s", got, tfMessageHash)
	}
}

type stampedMsg struct {
	Header headerMsg
	Value  int
}

// perception publishes and consumes stamped messages in declared frames.
type perception struct {
	Tick Timer           `rate:"10hz"`
	Out  Pub[stampedMsg] `topic:"cloud" frame:"laser"`
	In   Sub[stampedMsg] `topic:"echo" frame:"base_link"`
	Tree TF

	got []stampedMsg
}

func (p *perception) OnTick()           { p.Out.Publish(stampedMsg{Value: 1}) }
func (p *perception) OnIn(m stampedMsg) { p.got = append(p.got, m) }

// cloudProbe records what perception publishes. Probe handlers run on the
// probe's own executor, so reading after Settle is ordered.
type cloudProbe struct {
	Cloud Sub[stampedMsg] `topic:"cloud"`
	Echo  Pub[stampedMsg] `topic:"echo"`

	msgs []stampedMsg
}

func (p *cloudProbe) OnCloud(m stampedMsg) { p.msgs = append(p.msgs, m) }

type tfProbe struct {
	Static Sub[tfMessageMsg] `topic:"tf_static" qos:"transient"`

	msgs []tfMessageMsg
}

func (p *tfProbe) OnStatic(m tfMessageMsg) { p.msgs = append(p.msgs, m) }

// The frame tag is what reaches the wire: the publisher stamps it, and a
// timestamp comes with it.
func TestFrameTagStampsPublishedMessages(t *testing.T) {
	app, err := NewTestApp(TestOptions{Frames: loadTree(t, robotFrames)}, &perception{})
	if err != nil {
		t.Fatal(err)
	}
	defer app.Close()

	probe := &cloudProbe{}
	if err := app.BindProbe("probe", probe); err != nil {
		t.Fatal(err)
	}
	if _, err := app.Tick("perception"); err != nil {
		t.Fatal(err)
	}
	app.Settle()

	if len(probe.msgs) == 0 {
		t.Fatal("nothing published")
	}
	got := probe.msgs[0]
	if got.Header.FrameId != "laser" {
		t.Fatalf("frame %q, want the declared %q", got.Header.FrameId, "laser")
	}
	if got.Header.Stamp.IsZero() {
		t.Fatal("no timestamp stamped")
	}
}

// A message arriving in another frame is counted and named, rather than
// quietly becoming a wrong transform later.
func TestFrameMismatchIsCounted(t *testing.T) {
	app, err := NewTestApp(TestOptions{Frames: loadTree(t, robotFrames)}, &perception{})
	if err != nil {
		t.Fatal(err)
	}
	defer app.Close()

	probe := &cloudProbe{}
	if err := app.BindProbe("peer", probe); err != nil {
		t.Fatal(err)
	}
	before := metricValue(t, "conductor_frame_mismatches_total", "odom")
	probe.Echo.Publish(stampedMsg{Header: headerMsg{FrameId: "odom", Stamp: time.Now()}})
	app.Settle()

	if got := metricValue(t, "conductor_frame_mismatches_total", "odom"); got <= before {
		t.Fatalf("mismatch counter %v, want it above %v", got, before)
	}
}

// metricValue sums a metric's samples whose labels mention want.
func metricValue(t *testing.T, name, want string) float64 {
	t.Helper()
	total := 0.0
	for _, m := range MetricsSnapshot() {
		if m.Name != name {
			continue
		}
		for _, v := range m.Labels {
			if v == want {
				total += m.Value
			}
		}
	}
	return total
}

// Declaring a frame on a message with no header is a wiring error, not a
// silent no-op.
func TestFrameTagNeedsAHeader(t *testing.T) {
	type headless struct {
		Out Pub[ping] `topic:"pings" frame:"laser"`
	}
	app, err := NewTestApp(TestOptions{Frames: loadTree(t, robotFrames)}, &headless{})
	if err == nil {
		app.Close()
		t.Fatal("wiring succeeded, want an error")
	}
	if !strings.Contains(err.Error(), "no std_msgs/Header") {
		t.Fatalf("error %q, want it to explain the missing header", err)
	}
}

// A frame that is not in the tree is a wiring error too: the point of
// declaring the tree is that frame names stop being free-form strings.
func TestFrameTagMustNameADeclaredFrame(t *testing.T) {
	type wrong struct {
		Out Pub[stampedMsg] `topic:"cloud" frame:"camera"`
	}
	app, err := NewTestApp(TestOptions{Frames: loadTree(t, robotFrames)}, &wrong{})
	if err == nil {
		app.Close()
		t.Fatal("wiring succeeded, want an error")
	}
	if !strings.Contains(err.Error(), `frame "camera" is not declared`) {
		t.Fatalf("error %q, want it to name the undeclared frame", err)
	}
}

// The declared static transforms go out on tf_static, so there is no
// static_transform_publisher to launch and nothing to keep in step with the
// code.
func TestStaticTransformsArePublished(t *testing.T) {
	app, err := NewTestApp(TestOptions{Frames: loadTree(t, robotFrames), ManualLifecycle: true}, &perception{})
	if err != nil {
		t.Fatal(err)
	}
	defer app.Close()

	probe := &tfProbe{}
	if err := app.BindProbe("tf_listener", probe); err != nil {
		t.Fatal(err)
	}
	for _, tr := range []Transition{TransitionConfigure, TransitionActivate} {
		if err := app.Transition("perception", tr); err != nil {
			t.Fatal(err)
		}
	}
	app.Settle()

	if len(probe.msgs) == 0 {
		t.Fatal("nothing published on tf_static")
	}
	var children []string
	for _, tf := range probe.msgs[0].Transforms {
		children = append(children, tf.ChildFrameId)
	}
	if got, want := strings.Join(children, ","), "laser,imu"; got != want {
		t.Fatalf("published transforms for %s, want %s", got, want)
	}
	if x := probe.msgs[0].Transforms[0].Transform.Translation.X; x != 0.12 {
		t.Fatalf("laser x %v, want 0.12 from the declaration", x)
	}
}

// The runtime's lookup is the same one the checker resolves at build time.
func TestTFFieldLooksUpThroughTheDeclaredTree(t *testing.T) {
	p := &perception{}
	app, err := NewTestApp(TestOptions{Frames: loadTree(t, robotFrames)}, p)
	if err != nil {
		t.Fatal(err)
	}
	defer app.Close()

	at, err := p.Tree.Lookup("base_link", "imu")
	if err != nil {
		t.Fatal(err)
	}
	if at.Translation != [3]float64{0, 0, 0.05} {
		t.Fatalf("imu at %v, want [0 0 0.05]", at.Translation)
	}
	if _, err := p.Tree.Lookup("base_link", "camera"); err == nil {
		t.Fatal("lookup of an undeclared frame succeeded")
	}
}

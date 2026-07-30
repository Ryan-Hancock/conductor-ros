package urdf

import (
	"math"
	"path/filepath"
	"strings"
	"testing"

	conductor "conductor.dev/conductor"
)

// The fixture is the real TurtleBot3 waffle description from ROBOTIS, in both
// the form they ship (xacro) and the form xacro produces — which is also what
// /robot_description carries on a running robot. Parsing somebody else's real
// file is the test that matters; a URDF written to suit the parser would prove
// nothing.
func waffle(t *testing.T) *Robot {
	t.Helper()
	r, err := Load(filepath.Join("testdata", "turtlebot3_waffle.urdf"))
	if err != nil {
		t.Fatal(err)
	}
	return r
}

func TestParseTheRealTurtlebot3(t *testing.T) {
	r := waffle(t)
	if r.Name != "turtlebot3_waffle" {
		t.Errorf("robot name = %q", r.Name)
	}
	if len(r.Joints) != 12 {
		t.Errorf("%d joints, want the description's 12", len(r.Joints))
	}

	// The lidar's mounting: the number a hand-written frames.json gets wrong.
	var scan *Joint
	for i := range r.Joints {
		if r.Joints[i].Child == "base_scan" {
			scan = &r.Joints[i]
		}
	}
	if scan == nil {
		t.Fatal("no joint reaches base_scan")
	}
	if scan.Parent != "base_link" || !scan.Fixed() {
		t.Errorf("scan joint = %+v, want a fixed joint under base_link", scan)
	}
	for i, want := range [3]float64{-0.064, 0, 0.122} {
		if math.Abs(scan.XYZ[i]-want) > 1e-9 {
			t.Errorf("scan origin = %v, want %v", scan.XYZ, [3]float64{-0.064, 0, 0.122})
			break
		}
	}

	// Wheels are continuous, so their transforms are joint state.
	var moving int
	for _, j := range r.Joints {
		if !j.Fixed() {
			moving++
		}
	}
	if moving != 2 {
		t.Errorf("%d movable joints, want the two wheels", moving)
	}
}

// A xacro document is refused rather than half-understood: a `${...}` where a
// number belongs would otherwise parse as zero, which is the exact class of
// mistake deriving the tree is meant to remove.
func TestXacroIsRefusedWithTheFix(t *testing.T) {
	_, err := Load(filepath.Join("testdata", "turtlebot3_waffle.urdf.xacro"))
	if err == nil {
		t.Fatal("a xacro document parsed as plain URDF")
	}
	for _, want := range []string{"xacro", "expand it first", "robot_description"} {
		if !strings.Contains(err.Error(), want) {
			t.Errorf("error does not mention %q: %v", want, err)
		}
	}
}

// A substitution in a comment is not a substitution: the shipped TurtleBot3
// description keeps commented-out xacro beside the expanded origin that
// replaced it, and refusing that file would refuse most real robots.
func TestCommentedXacroIsNotXacro(t *testing.T) {
	doc := `<robot name="x">
	  <joint name="j" type="fixed">
	    <origin rpy="0 0 0" xyz="0.005 0.018 0.013"/>
	    <!-- <origin xyz="${cam_px} ${cam_py} ${cam_pz}" rpy="0 0 0"/> -->
	    <parent link="a"/><child link="b"/>
	  </joint>
	</robot>`
	r, err := Parse([]byte(doc))
	if err != nil {
		t.Fatalf("a comment made this look like xacro: %v", err)
	}
	if r.Joints[0].XYZ != [3]float64{0.005, 0.018, 0.013} {
		t.Errorf("origin = %v, want the uncommented one", r.Joints[0].XYZ)
	}

	// Uncommented, the same substitution is refused.
	live := strings.Replace(doc, "<!-- <origin", "<origin", 1)
	live = strings.Replace(live, `rpy="0 0 0"/> -->`, `rpy="0 0 0"/>`, 1)
	if _, err := Parse([]byte(live)); err == nil {
		t.Error("an unexpanded substitution parsed without complaint")
	}
}

func TestParseRejectsWhatItCannotUnderstand(t *testing.T) {
	cases := map[string]string{
		"not xml":        `robot: turtlebot`,
		"wrong root":     `<scene name="x"><link name="a"/></scene>`,
		"no joints":      `<robot name="x"><link name="a"/></robot>`,
		"joint no type":  `<robot name="x"><joint name="j"><parent link="a"/><child link="b"/></joint></robot>`,
		"joint no child": `<robot name="x"><joint name="j" type="fixed"><parent link="a"/></joint></robot>`,
		"bad origin": `<robot name="x"><joint name="j" type="fixed"><parent link="a"/><child link="b"/>` +
			`<origin xyz="0 0"/></joint></robot>`,
		"origin not a number": `<robot name="x"><joint name="j" type="fixed"><parent link="a"/><child link="b"/>` +
			`<origin xyz="0 0 up"/></joint></robot>`,
	}
	for name, doc := range cases {
		if _, err := Parse([]byte(doc)); err == nil {
			t.Errorf("%s: parsed without complaint", name)
		}
	}
}

// A joint with no <origin> is the identity, which is what URDF means by
// omitting it.
func TestMissingOriginIsIdentity(t *testing.T) {
	r, err := Parse([]byte(`<robot name="x">
	  <joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint>
	</robot>`))
	if err != nil {
		t.Fatal(err)
	}
	if r.Joints[0].XYZ != [3]float64{} || r.Joints[0].RPY != [3]float64{} {
		t.Errorf("joint = %+v, want the identity", r.Joints[0])
	}
}

// Fixed joints become fixed transforms carrying the URDF's offsets, attributed
// to whoever publishes them; movable joints become dynamic ones carrying no
// offset at all, because a wheel's transform is its joint state.
func TestFramesFromTheRealTurtlebot3(t *testing.T) {
	tree, notes := Frames(waffle(t), Options{})

	if len(tree.Transforms) != 12 {
		t.Fatalf("%d transforms, want one per joint", len(tree.Transforms))
	}
	if got := len(tree.Fixed()); got != 10 {
		t.Errorf("%d fixed transforms, want 10", got)
	}
	// Nothing is ours: a robot with a URDF has a robot_state_publisher, and
	// publishing these too would put two static transforms on one child.
	if got := len(tree.Published()); got != 0 {
		t.Errorf("%d transforms claimed as ours, want none by default", got)
	}
	for _, tf := range tree.Transforms {
		if tf.By != "robot_state_publisher" {
			t.Errorf("%s is attributed to %q", tf, tf.By)
		}
		if tf.Dynamic && (tf.XYZ != [3]float64{} || tf.RPY != [3]float64{}) {
			t.Errorf("%s carries an offset, but its value is joint state", tf)
		}
	}

	// The whole point: the geometry is resolvable even though somebody else
	// publishes it, because the description says what it is.
	at, err := tree.Lookup("base_link", "base_scan")
	if err != nil {
		t.Fatalf("looking up the lidar: %v", err)
	}
	if math.Abs(at.Translation[0]-(-0.064)) > 1e-9 || math.Abs(at.Translation[2]-0.122) > 1e-9 {
		t.Errorf("base_link -> base_scan = %v, want the URDF's -0.064, 0, 0.122", at.Translation)
	}

	// And a path across a wheel is still refused, because that value is not in
	// the description.
	if _, err := tree.Lookup("base_link", "wheel_left_link"); err == nil {
		t.Error("a lookup across a continuous joint was resolved")
	}

	if !strings.Contains(strings.Join(notesText(notes), " "), "robot_state_publisher") {
		t.Errorf("notes do not say who publishes the tree: %v", notes)
	}
}

// -publish claims the fixed joints for this application, which is right when
// nothing else is publishing them.
func TestFramesCanBeClaimed(t *testing.T) {
	tree, notes := Frames(waffle(t), Options{Ours: true})
	if got := len(tree.Published()); got != 10 {
		t.Errorf("%d transforms to publish, want the 10 fixed joints", got)
	}
	// The movable ones are still somebody else's: claiming them would be
	// claiming to know the wheel angles.
	for _, tf := range tree.Transforms {
		if tf.Dynamic && tf.Ours() {
			t.Errorf("%s is claimed as ours, but its value is joint state", tf)
		}
	}
	if !strings.Contains(strings.Join(notesText(notes), " "), "published twice") {
		t.Errorf("notes do not warn about a second publisher: %v", notes)
	}
}

// The derivation reports a description that is not a tree, rather than emitting
// a frames.json the checker will reject.
func TestFramesReportsAMalformedDescription(t *testing.T) {
	r, err := Parse([]byte(`<robot name="x">
	  <link name="spare"/>
	  <joint name="a" type="fixed"><parent link="base"/><child link="tool"/></joint>
	  <joint name="b" type="fixed"><parent link="arm"/><child link="tool"/></joint>
	</robot>`))
	if err != nil {
		t.Fatal(err)
	}
	_, notes := Frames(r, Options{})
	text := strings.Join(notesText(notes), " ")
	if !strings.Contains(text, "not well formed") || !strings.Contains(text, "parents") {
		t.Errorf("notes do not report the two parents: %v", notes)
	}
	if !strings.Contains(text, "spare") {
		t.Errorf("notes do not report the orphan link: %v", notes)
	}
}

// The tree the runtime loads and the tree derived here are the same type, so a
// derived file round-trips through frames.json unchanged.
func TestDerivedTreeIsTheRuntimeTree(t *testing.T) {
	tree, _ := Frames(waffle(t), Options{})
	var _ *conductor.FrameTree = tree
	if problems := tree.Check(); len(problems) != 0 {
		t.Errorf("the real turtlebot3 tree is malformed: %v", problems)
	}
	if roots := tree.Roots(); len(roots) != 1 || roots[0] != "base_footprint" {
		t.Errorf("roots = %v, want just base_footprint", roots)
	}
}

func notesText(notes []Note) []string {
	out := make([]string, len(notes))
	for i, n := range notes {
		out[i] = string(n)
	}
	return out
}

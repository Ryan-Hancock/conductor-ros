package conductor

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"sort"
	"strings"
)

// TF is the other half of the ROS 2 folklore problem: frame ids are magic
// strings in message headers, the static transforms between them live as
// positional arguments to static_transform_publisher in a launch file, and
// a wrong one is discovered in RViz, on a robot, later.
//
// Conductor declares the transform tree in frames.json beside conductor.json:
//
//	{
//	  "static": [
//	    {"parent": "base_link", "child": "laser", "xyz": [0.12, 0, 0.19], "rpy": [0, 0, 3.14159]}
//	  ],
//	  "dynamic": [
//	    {"parent": "map", "child": "odom", "by": "amcl"},
//	    {"parent": "odom", "child": "base_link", "by": "ekf"}
//	  ]
//	}
//
// Static links are ours: the runtime publishes them on tf_static, and
// TF.Lookup composes them. Dynamic links are someone else's — a localizer, an
// odometry filter — and declaring them is what makes the tree whole, so the
// checker can tell "that frame does not exist" from "that frame exists but
// only a dynamic transform reaches it".

// Transform is one link of the frame tree: the pose of Child expressed in
// Parent. Rotation is roll-pitch-yaw in radians, the same convention as
// static_transform_publisher and URDF.
type Transform struct {
	Parent string     `json:"parent"`
	Child  string     `json:"child"`
	XYZ    [3]float64 `json:"xyz"`
	RPY    [3]float64 `json:"rpy"`

	// Dynamic links are published at runtime by someone else; By names them,
	// for the error message that matters when a lookup cannot be resolved
	// statically.
	Dynamic bool   `json:"-"`
	By      string `json:"by,omitempty"`
}

func (t Transform) String() string {
	if t.Dynamic {
		return fmt.Sprintf("%s -> %s (dynamic, by %s)", t.Parent, t.Child, orUnknown(t.By))
	}
	return fmt.Sprintf("%s -> %s", t.Parent, t.Child)
}

// Isometry returns the transform as a rigid motion.
func (t Transform) Isometry() Isometry {
	return Isometry{Translation: t.XYZ, Rotation: quatFromRPY(t.RPY)}
}

// FrameTree is the declared transform tree.
type FrameTree struct {
	Path       string
	Transforms []Transform
}

type framesFile struct {
	Static  []Transform `json:"static"`
	Dynamic []Transform `json:"dynamic"`
}

// LoadFrames reads a frames.json file. A missing file is not an error: an
// application that declares no frames simply has none.
func LoadFrames(path string) (*FrameTree, error) {
	b, err := os.ReadFile(path)
	if err != nil {
		if os.IsNotExist(err) {
			return nil, nil
		}
		return nil, err
	}
	var f framesFile
	if err := json.Unmarshal(b, &f); err != nil {
		return nil, fmt.Errorf("%s: %w", path, err)
	}
	t := &FrameTree{Path: path}
	for _, tf := range f.Static {
		tf.Dynamic = false
		t.Transforms = append(t.Transforms, tf)
	}
	for _, tf := range f.Dynamic {
		tf.Dynamic = true
		t.Transforms = append(t.Transforms, tf)
	}
	for _, tf := range t.Transforms {
		if tf.Parent == "" || tf.Child == "" {
			return nil, fmt.Errorf("%s: every transform needs a parent and a child", path)
		}
	}
	return t, nil
}

// Static returns the transforms this application publishes itself.
func (t *FrameTree) Static() []Transform {
	if t == nil {
		return nil
	}
	var out []Transform
	for _, tf := range t.Transforms {
		if !tf.Dynamic {
			out = append(out, tf)
		}
	}
	return out
}

// Frames lists every frame named by a transform, sorted.
func (t *FrameTree) Frames() []string {
	if t == nil {
		return nil
	}
	seen := map[string]bool{}
	var out []string
	for _, tf := range t.Transforms {
		for _, f := range []string{tf.Parent, tf.Child} {
			if !seen[f] {
				seen[f] = true
				out = append(out, f)
			}
		}
	}
	sort.Strings(out)
	return out
}

// Has reports whether the tree declares a frame.
func (t *FrameTree) Has(frame string) bool {
	if t == nil {
		return false
	}
	for _, tf := range t.Transforms {
		if tf.Parent == frame || tf.Child == frame {
			return true
		}
	}
	return false
}

// Roots returns the frames with no parent. A well-formed tree has exactly one.
func (t *FrameTree) Roots() []string {
	if t == nil {
		return nil
	}
	hasParent := map[string]bool{}
	for _, tf := range t.Transforms {
		hasParent[tf.Child] = true
	}
	var out []string
	for _, f := range t.Frames() {
		if !hasParent[f] {
			out = append(out, f)
		}
	}
	return out
}

// FrameProblem is a structural fault in the declared tree. The kinds are
// stable so the checker can map them to its own codes and positions.
type FrameProblem struct {
	Kind string // "multiple_parents", "cycle", "disconnected"
	Msg  string
}

// Check reports the ways a transform tree can be malformed. These are exactly
// the faults that, undeclared, show up as a broken TF tree at runtime.
func (t *FrameTree) Check() []FrameProblem {
	if t == nil || len(t.Transforms) == 0 {
		return nil
	}
	var out []FrameProblem

	parents := map[string][]Transform{}
	for _, tf := range t.Transforms {
		parents[tf.Child] = append(parents[tf.Child], tf)
	}
	children := make([]string, 0, len(parents))
	for c := range parents {
		children = append(children, c)
	}
	sort.Strings(children)
	for _, c := range children {
		if len(parents[c]) > 1 {
			var from []string
			for _, tf := range parents[c] {
				from = append(from, tf.Parent)
			}
			out = append(out, FrameProblem{"multiple_parents",
				fmt.Sprintf("frame %q has %d parents (%s); in a transform tree a frame has exactly one",
					c, len(from), strings.Join(from, ", "))})
		}
	}

	// A cycle is a frame reachable from itself by walking parent links.
	for _, f := range t.Frames() {
		seen := map[string]bool{f: true}
		cur := f
		for {
			ps := parents[cur]
			if len(ps) == 0 {
				break
			}
			cur = ps[0].Parent
			if cur == f {
				out = append(out, FrameProblem{"cycle",
					fmt.Sprintf("frames %q and its parents form a cycle", f)})
				break
			}
			if seen[cur] {
				break
			}
			seen[cur] = true
		}
	}

	if roots := t.Roots(); len(roots) > 1 {
		out = append(out, FrameProblem{"disconnected",
			fmt.Sprintf("the tree has %d roots (%s); a transform tree has one, and frames under different roots cannot be transformed between",
				len(roots), strings.Join(roots, ", "))})
	}
	return out
}

// link is one edge of a lookup path, with the direction it is traversed.
type link struct {
	tf      Transform
	inverse bool // traversed child -> parent
}

// path finds the route from source to target through the tree, ignoring link
// direction (a lookup may go up and back down).
func (t *FrameTree) path(target, source string) ([]link, error) {
	if t == nil {
		return nil, fmt.Errorf("no transform tree is declared (add frames.json)")
	}
	if !t.Has(target) {
		return nil, fmt.Errorf("unknown frame %q", target)
	}
	if !t.Has(source) {
		return nil, fmt.Errorf("unknown frame %q", source)
	}
	if target == source {
		return nil, nil
	}
	// Breadth-first from target so the accumulated transform reads
	// target <- ... <- source, which is what a lookup means.
	type step struct {
		frame string
		via   []link
	}
	queue := []step{{frame: target}}
	seen := map[string]bool{target: true}
	for len(queue) > 0 {
		cur := queue[0]
		queue = queue[1:]
		for _, tf := range t.Transforms {
			var next string
			var l link
			switch {
			case tf.Parent == cur.frame:
				next, l = tf.Child, link{tf: tf}
			case tf.Child == cur.frame:
				next, l = tf.Parent, link{tf: tf, inverse: true}
			default:
				continue
			}
			if seen[next] {
				continue
			}
			seen[next] = true
			via := append(append([]link{}, cur.via...), l)
			if next == source {
				return via, nil
			}
			queue = append(queue, step{frame: next, via: via})
		}
	}
	return nil, fmt.Errorf("no transform connects %q and %q", target, source)
}

// Lookup returns the transform that takes points in the source frame to the
// target frame, composed from the declared static links. It fails if the path
// crosses a dynamic transform, naming the link and its publisher: that is a
// runtime lookup against tf, not something a declaration can answer.
func (t *FrameTree) Lookup(target, source string) (Isometry, error) {
	links, err := t.path(target, source)
	if err != nil {
		return Isometry{}, err
	}
	out := Isometry{Rotation: [4]float64{0, 0, 0, 1}}
	for _, l := range links {
		if l.tf.Dynamic {
			return Isometry{}, fmt.Errorf("the path from %s to %s crosses the dynamic transform %s; look it up against tf at runtime",
				source, target, l.tf)
		}
		iso := l.tf.Isometry()
		if l.inverse {
			iso = iso.Inverse()
		}
		out = out.Then(iso)
	}
	return out, nil
}

// Connects reports whether any path — static or dynamic — joins two frames,
// which is what makes a frame mismatch between two endpoints merely something
// a consumer must transform rather than a broken graph.
func (t *FrameTree) Connects(a, b string) error {
	_, err := t.path(a, b)
	return err
}

// Isometry is a rigid transform: a translation and a rotation, the rotation
// as a quaternion (x, y, z, w) so composition is exact.
type Isometry struct {
	Translation [3]float64
	Rotation    [4]float64
}

// Then composes: (a.Then(b)) maps points from b's frame to a's parent frame.
func (a Isometry) Then(b Isometry) Isometry {
	return Isometry{
		Translation: add3(a.Translation, rotate(a.Rotation, b.Translation)),
		Rotation:    quatMul(a.Rotation, b.Rotation),
	}
}

// Inverse returns the transform in the opposite direction.
func (a Isometry) Inverse() Isometry {
	q := quatConj(a.Rotation)
	t := rotate(q, a.Translation)
	return Isometry{Translation: [3]float64{-t[0], -t[1], -t[2]}, Rotation: q}
}

// Apply transforms a point.
func (a Isometry) Apply(p [3]float64) [3]float64 {
	return add3(a.Translation, rotate(a.Rotation, p))
}

// RPY returns the rotation as roll, pitch, yaw in radians.
func (a Isometry) RPY() [3]float64 {
	x, y, z, w := a.Rotation[0], a.Rotation[1], a.Rotation[2], a.Rotation[3]
	sinrCosp := 2 * (w*x + y*z)
	cosrCosp := 1 - 2*(x*x+y*y)
	sinp := 2 * (w*y - z*x)
	pitch := math.Asin(clamp(sinp, -1, 1))
	sinyCosp := 2 * (w*z + x*y)
	cosyCosp := 1 - 2*(y*y+z*z)
	return [3]float64{math.Atan2(sinrCosp, cosrCosp), pitch, math.Atan2(sinyCosp, cosyCosp)}
}

func quatFromRPY(rpy [3]float64) [4]float64 {
	cr, sr := math.Cos(rpy[0]/2), math.Sin(rpy[0]/2)
	cp, sp := math.Cos(rpy[1]/2), math.Sin(rpy[1]/2)
	cy, sy := math.Cos(rpy[2]/2), math.Sin(rpy[2]/2)
	return [4]float64{
		sr*cp*cy - cr*sp*sy,
		cr*sp*cy + sr*cp*sy,
		cr*cp*sy - sr*sp*cy,
		cr*cp*cy + sr*sp*sy,
	}
}

func quatMul(a, b [4]float64) [4]float64 {
	ax, ay, az, aw := a[0], a[1], a[2], a[3]
	bx, by, bz, bw := b[0], b[1], b[2], b[3]
	return [4]float64{
		aw*bx + ax*bw + ay*bz - az*by,
		aw*by - ax*bz + ay*bw + az*bx,
		aw*bz + ax*by - ay*bx + az*bw,
		aw*bw - ax*bx - ay*by - az*bz,
	}
}

func quatConj(q [4]float64) [4]float64 { return [4]float64{-q[0], -q[1], -q[2], q[3]} }

func rotate(q [4]float64, v [3]float64) [3]float64 {
	// v' = v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
	u := [3]float64{q[0], q[1], q[2]}
	t := cross(u, v)
	t = [3]float64{2 * (t[0] + q[3]*v[0]), 2 * (t[1] + q[3]*v[1]), 2 * (t[2] + q[3]*v[2])}
	// The doubling above folds the factor of 2 in before the second cross.
	c := cross(u, t)
	return [3]float64{v[0] + c[0], v[1] + c[1], v[2] + c[2]}
}

func cross(a, b [3]float64) [3]float64 {
	return [3]float64{
		a[1]*b[2] - a[2]*b[1],
		a[2]*b[0] - a[0]*b[2],
		a[0]*b[1] - a[1]*b[0],
	}
}

func add3(a, b [3]float64) [3]float64 {
	return [3]float64{a[0] + b[0], a[1] + b[1], a[2] + b[2]}
}

func clamp(v, lo, hi float64) float64 { return math.Max(lo, math.Min(hi, v)) }

func orUnknown(s string) string {
	if s == "" {
		return "an external node"
	}
	return s
}

// Package urdf reads the part of a robot description conductor already
// declares elsewhere: the transform tree.
//
// frames.json and a URDF say overlapping things. The URDF names every link and
// every joint between them; its *fixed* joints are exactly the static
// transforms conductor publishes on tf_static, and its movable joints are
// exactly the dynamic links a robot_state_publisher provides. Writing both by
// hand is duplication conductor introduced, and the numbers in it — a lidar
// 64mm behind base_link and 122mm up — are precisely the ones a person
// transcribes wrongly.
//
// So derive: `conductor frames -from robot.urdf` emits the tree, and the frame
// checks then apply to the robot's real description rather than to a file
// beside it.
//
// The subset read here is links and joints: names, the parent/child pair, the
// joint type, and the origin. Everything else in a URDF — geometry, inertia,
// materials, transmissions, gazebo tags — describes mass and appearance, which
// is nothing conductor has an opinion about.
package urdf

import (
	"encoding/xml"
	"fmt"
	"os"
	"strconv"
	"strings"
)

// Robot is the parsed subset of a URDF.
type Robot struct {
	Name   string
	Links  []string
	Joints []Joint
	Path   string
}

// Joint is one link of the robot's tree.
type Joint struct {
	Name   string
	Type   string // fixed, revolute, continuous, prismatic, floating, planar
	Parent string
	Child  string
	XYZ    [3]float64
	RPY    [3]float64
}

// Fixed reports whether the joint never moves, which is what makes its
// transform knowable without asking tf at runtime.
func (j Joint) Fixed() bool { return j.Type == "fixed" }

// xml shapes, kept private: the exported model above is the subset conductor
// cares about, and it should not change when a URDF grows another tag.
type xmlRobot struct {
	XMLName xml.Name   `xml:"robot"`
	Name    string     `xml:"name,attr"`
	Links   []xmlLink  `xml:"link"`
	Joints  []xmlJoint `xml:"joint"`
}

type xmlLink struct {
	Name string `xml:"name,attr"`
}

type xmlJoint struct {
	Name   string    `xml:"name,attr"`
	Type   string    `xml:"type,attr"`
	Parent xmlLinkOf `xml:"parent"`
	Child  xmlLinkOf `xml:"child"`
	Origin *xmlPose  `xml:"origin"`
}

type xmlLinkOf struct {
	Link string `xml:"link,attr"`
}

type xmlPose struct {
	XYZ string `xml:"xyz,attr"`
	RPY string `xml:"rpy,attr"`
}

// Load reads a plain URDF file.
func Load(path string) (*Robot, error) {
	b, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	r, err := Parse(b)
	if err != nil {
		return nil, fmt.Errorf("%s: %w", path, err)
	}
	r.Path = path
	return r, nil
}

// Parse reads a plain URDF document.
//
// A URDF that still has xacro in it is refused rather than half-understood: a
// `${wheel_separation}` where a number should be would otherwise become a
// silent zero, which is the class of mistake this whole package exists to
// remove. Expanding xacro is a job for xacro — a build step, or a `requires`
// process — and the error says so.
func Parse(doc []byte) (*Robot, error) {
	if marker, ok := unexpandedXacro(doc); ok {
		return nil, fmt.Errorf("this is a xacro document, not a plain URDF (found %s): expand it first, "+
			"e.g. `xacro robot.urdf.xacro > robot.urdf`, and derive frames from the result — "+
			"which is also what /robot_description carries on a running robot", marker)
	}

	var x xmlRobot
	if err := xml.Unmarshal(doc, &x); err != nil {
		return nil, fmt.Errorf("not a URDF: %w", err)
	}
	if x.XMLName.Local != "robot" {
		return nil, fmt.Errorf("not a URDF: root element is <%s>, want <robot>", x.XMLName.Local)
	}

	r := &Robot{Name: x.Name}
	for _, l := range x.Links {
		if l.Name == "" {
			return nil, fmt.Errorf("a <link> has no name")
		}
		r.Links = append(r.Links, l.Name)
	}
	for _, j := range x.Joints {
		joint := Joint{
			Name:   j.Name,
			Type:   strings.TrimSpace(j.Type),
			Parent: j.Parent.Link,
			Child:  j.Child.Link,
		}
		if joint.Name == "" {
			return nil, fmt.Errorf("a <joint> has no name")
		}
		if joint.Parent == "" || joint.Child == "" {
			return nil, fmt.Errorf("joint %q needs both a <parent link> and a <child link>", joint.Name)
		}
		if joint.Type == "" {
			return nil, fmt.Errorf("joint %q has no type", joint.Name)
		}
		if j.Origin != nil {
			var err error
			if joint.XYZ, err = triple("joint "+joint.Name+" origin xyz", j.Origin.XYZ); err != nil {
				return nil, err
			}
			if joint.RPY, err = triple("joint "+joint.Name+" origin rpy", j.Origin.RPY); err != nil {
				return nil, err
			}
		}
		r.Joints = append(r.Joints, joint)
	}
	if len(r.Joints) == 0 {
		return nil, fmt.Errorf("robot %q declares no joints, so it describes no transforms", r.Name)
	}
	return r, nil
}

// unexpandedXacro reports whether the document still needs expanding, and what
// gave it away.
//
// The search runs over the document with comments removed, because xacro does
// not expand comments and real descriptions are full of commented-out
// alternatives: the shipped TurtleBot3 waffle keeps `${r200_cam_rgb_px}` in one,
// beside the expanded origin that replaced it. A comment is not markup.
func unexpandedXacro(doc []byte) (string, bool) {
	s := stripComments(string(doc))
	if i := strings.Index(s, "${"); i >= 0 {
		end := i + 2
		for end < len(s) && s[end] != '}' && end-i < 40 {
			end++
		}
		return fmt.Sprintf("the substitution %q", s[i:min(end+1, len(s))]), true
	}
	// The xmlns:xacro declaration alone is harmless — xacro leaves it behind on
	// documents it has already expanded — so this looks for elements in the
	// namespace rather than for the namespace itself.
	for _, tag := range []string{"<xacro:", "<$(", "$(find "} {
		if strings.Contains(s, tag) {
			return fmt.Sprintf("the unexpanded %s", strings.Trim(tag, "<")), true
		}
	}
	return "", false
}

// stripComments removes XML comments, leaving everything else — including the
// whitespace — where it was.
func stripComments(s string) string {
	var b strings.Builder
	for {
		start := strings.Index(s, "<!--")
		if start < 0 {
			b.WriteString(s)
			return b.String()
		}
		b.WriteString(s[:start])
		end := strings.Index(s[start:], "-->")
		if end < 0 {
			return b.String() // unterminated comment: the rest is comment
		}
		s = s[start+end+len("-->"):]
	}
}

// triple parses a URDF "x y z" attribute. A missing attribute is the identity,
// which is what URDF means by omitting it.
func triple(what, value string) ([3]float64, error) {
	var out [3]float64
	if strings.TrimSpace(value) == "" {
		return out, nil
	}
	fields := strings.Fields(value)
	if len(fields) != 3 {
		return out, fmt.Errorf("%s=%q: want three numbers", what, value)
	}
	for i, f := range fields {
		v, err := strconv.ParseFloat(f, 64)
		if err != nil {
			return out, fmt.Errorf("%s=%q: %w", what, value, err)
		}
		out[i] = v
	}
	return out, nil
}

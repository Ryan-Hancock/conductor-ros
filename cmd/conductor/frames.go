package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"strings"

	"conductor.dev/conductor"
	"conductor.dev/conductor/internal/urdf"
)

// runFrames derives a transform tree from a robot description.
//
// frames.json and a URDF overlap: the URDF's fixed joints are the tree's fixed
// transforms, and its movable joints are the dynamic ones a
// robot_state_publisher provides. Declaring both by hand is duplication
// conductor introduced, and the numbers involved — a lidar 64mm behind
// base_link — are exactly the ones a person transcribes wrongly. So derive
// them, and let the frame checks apply to the robot's real description.
func runFrames(args []string) error {
	fs := flag.NewFlagSet("frames", flag.ExitOnError)
	from := fs.String("from", "", "robot description to derive the tree from (a plain URDF)")
	out := fs.String("o", "", "write frames.json here (default: stdout)")
	publisher := fs.String("by", "robot_state_publisher", "who publishes the robot's transforms")
	publish := fs.Bool("publish", false,
		"claim the fixed joints for this application, so the runtime publishes them on tf_static")
	fixedOnly := fs.Bool("fixed-only", false,
		"leave the movable joints out: nothing publishes joint state on this robot")
	if err := fs.Parse(args); err != nil {
		return err
	}
	if *from == "" {
		return fmt.Errorf("frames: -from <robot.urdf> is required")
	}

	robot, err := urdf.Load(*from)
	if err != nil {
		return err
	}
	tree, notes := urdf.Frames(robot, urdf.Options{
		Publisher: *publisher, Ours: *publish, FixedOnly: *fixedOnly,
	})

	// A URDF describes the robot, not the world it is in: map -> odom comes
	// from a localizer and odom -> base_footprint from an odometry source, and
	// neither is anywhere in the description. Those are added by hand once, so
	// re-deriving after the robot changes must not throw them away.
	if *out != "" {
		kept, err := keepWorldLinks(*out, robot)
		if err != nil {
			return err
		}
		if len(kept) > 0 {
			tree.Transforms = append(kept, tree.Transforms...)
			names := make([]string, len(kept))
			for i, tf := range kept {
				names[i] = tf.String()
			}
			notes = append(notes, urdf.Note(fmt.Sprintf(
				"kept %d transform(s) already in %s that the description does not mention: %s",
				len(kept), *out, strings.Join(names, "; "))))
		}
	}

	doc, err := renderFrames(tree)
	if err != nil {
		return err
	}
	if *out == "" {
		fmt.Print(string(doc))
		reportFrames(os.Stderr, robot, tree, notes)
		return nil
	}
	if err := os.MkdirAll(filepath.Dir(*out), 0o755); err != nil {
		return err
	}
	if err := os.WriteFile(*out, doc, 0o644); err != nil {
		return err
	}
	fmt.Printf("wrote %s from %s\n", *out, robot.Path)
	reportFrames(os.Stdout, robot, tree, notes)
	return nil
}

// keepWorldLinks reads an existing frames.json and returns the transforms the
// robot description does not produce.
//
// The test is whether a joint has that frame as its *child*: those are the
// transforms the description defines, and re-deriving them is the point. A
// transform *into* the robot — odom -> base_link, published by an odometry
// source — has the robot's root link as its child, and no joint produces it, so
// it survives. That is the distinction between the robot and the world it is in,
// and the URDF only describes the first.
func keepWorldLinks(path string, robot *urdf.Robot) ([]conductor.Transform, error) {
	existing, err := conductor.LoadFrames(path)
	if err != nil || existing == nil {
		return nil, err // a missing file is not an error: there is nothing to keep
	}
	jointed := map[string]bool{}
	for _, j := range robot.Joints {
		jointed[j.Child] = true
	}
	var kept []conductor.Transform
	for _, tf := range existing.Transforms {
		if !jointed[tf.Child] {
			kept = append(kept, tf)
		}
	}
	return kept, nil
}

// renderFrames writes the tree in the form LoadFrames reads, splitting on the
// axis the file splits on: whether the value is known.
//
// The rendering is by hand because the point of a generated frames.json is that
// someone reads and edits it afterwards. json.MarshalIndent puts every element
// of an xyz on its own line, which turns a six-joint robot into a hundred lines
// nobody will diff.
func renderFrames(tree *conductor.FrameTree) ([]byte, error) {
	var fixed, dynamic []conductor.Transform
	for _, tf := range tree.Transforms {
		if tf.Dynamic {
			dynamic = append(dynamic, tf)
		} else {
			fixed = append(fixed, tf)
		}
	}

	var b strings.Builder
	b.WriteString("{\n")
	sections := []struct {
		name       string
		transforms []conductor.Transform
	}{{"static", fixed}, {"dynamic", dynamic}}
	written := 0
	for _, s := range sections {
		if len(s.transforms) == 0 {
			continue
		}
		if written > 0 {
			b.WriteString(",\n")
		}
		fmt.Fprintf(&b, "  %q: [\n", s.name)
		for i, tf := range s.transforms {
			line, err := renderTransform(tf)
			if err != nil {
				return nil, err
			}
			b.WriteString("    " + line)
			if i < len(s.transforms)-1 {
				b.WriteString(",")
			}
			b.WriteString("\n")
		}
		b.WriteString("  ]")
		written++
	}
	b.WriteString("\n}\n")
	return []byte(b.String()), nil
}

// renderTransform is one transform on one line. A dynamic entry carries no
// offset: its value is joint state, and writing zeros there would read as an
// answer.
func renderTransform(tf conductor.Transform) (string, error) {
	parent, err := json.Marshal(tf.Parent)
	if err != nil {
		return "", err
	}
	child, err := json.Marshal(tf.Child)
	if err != nil {
		return "", err
	}
	var b strings.Builder
	fmt.Fprintf(&b, "{\"parent\": %s, \"child\": %s", parent, child)
	if !tf.Dynamic {
		fmt.Fprintf(&b, ", \"xyz\": %s, \"rpy\": %s", triple(tf.XYZ), triple(tf.RPY))
	}
	if tf.By != "" {
		by, err := json.Marshal(tf.By)
		if err != nil {
			return "", err
		}
		fmt.Fprintf(&b, ", \"by\": %s", by)
	}
	b.WriteString("}")
	return b.String(), nil
}

func triple(v [3]float64) string {
	return fmt.Sprintf("[%s, %s, %s]", number(v[0]), number(v[1]), number(v[2]))
}

// number prints a coordinate the way the description wrote it, without
// exponents for the magnitudes a robot has.
func number(v float64) string {
	return strconv.FormatFloat(v, 'f', -1, 64)
}

// reportFrames says what was derived and what to look at, on stderr when the
// file itself is going to stdout.
func reportFrames(w *os.File, robot *urdf.Robot, tree *conductor.FrameTree, notes []urdf.Note) {
	fmt.Fprintf(w, "\n%s: %d joint(s) -> %d transform(s), %d frame(s)\n",
		robot.Name, len(robot.Joints), len(tree.Transforms), len(tree.Frames()))
	if roots := tree.Roots(); len(roots) > 0 {
		fmt.Fprintf(w, "  root: %s\n", strings.Join(roots, ", "))
	}
	for _, n := range notes {
		fmt.Fprintf(w, "  %s\n", n)
	}
}

package urdf

import (
	"fmt"
	"sort"

	conductor "conductor.dev/conductor"
)

// Deriving the tree forced a distinction the frames model had been eliding, and
// the Nav2 example is where it showed: conductor's `dynamic` meant "somebody
// else publishes this", and it was also the only way to say "the value is not
// knowable statically". Those are different facts, and a URDF has plenty of
// links that are the first without being the second — every fixed joint of a
// robot whose robot_state_publisher is already publishing tf_static.
//
// So a transform now carries both: whether it moves (dynamic) and who publishes
// it (by). A fixed joint attributed to robot_state_publisher is not ours to
// publish, and TF.Lookup can still resolve it, because the URDF says what it is.

// Options controls what the derived tree claims about ownership.
type Options struct {
	// Publisher names whoever publishes these transforms. The default,
	// robot_state_publisher, is who publishes them on any robot that has a
	// URDF at all.
	Publisher string

	// Ours claims the fixed joints for this application: the runtime will
	// publish them on tf_static itself. Right for a robot with no
	// robot_state_publisher running — and wrong, in a way that puts two
	// publishers on one static transform, for a robot that has one.
	Ours bool

	// FixedOnly leaves the movable joints out of the tree entirely. A robot
	// with nothing publishing joint state has no transform for its wheels, and
	// declaring one would tell the checker that a frame exists which never
	// appears on tf — a lie in the direction that hides mistakes.
	FixedOnly bool
}

// Note is something worth saying about a derivation that is not an error.
type Note string

// Frames derives a transform tree from a robot description.
//
// Fixed joints become fixed transforms carrying the URDF's own offsets; every
// other joint type becomes a dynamic one, because a revolute joint's transform
// is whatever the joint is doing right now, which only tf knows.
func Frames(r *Robot, opts Options) (*conductor.FrameTree, []Note) {
	if opts.Publisher == "" {
		opts.Publisher = "robot_state_publisher"
	}
	tree := &conductor.FrameTree{Path: r.Path}
	var notes []Note

	fixed, moving := 0, 0
	for _, j := range r.Joints {
		tf := conductor.Transform{
			Parent: j.Parent,
			Child:  j.Child,
			XYZ:    j.XYZ,
			RPY:    j.RPY,
		}
		if j.Fixed() {
			fixed++
			if !opts.Ours {
				tf.By = opts.Publisher
			}
		} else {
			moving++
			if opts.FixedOnly {
				continue
			}
			// A movable joint's offset is not the transform; the joint state is.
			// Carrying the URDF's origin here would look like an answer.
			tf.Dynamic = true
			tf.XYZ, tf.RPY = [3]float64{}, [3]float64{}
			tf.By = opts.Publisher
		}
		tree.Transforms = append(tree.Transforms, tf)
	}

	if opts.Ours {
		notes = append(notes, Note(fmt.Sprintf(
			"%d fixed joint(s) are claimed by this application, which will publish them on tf_static; "+
				"if a robot_state_publisher is running for this URDF, drop -publish so they are attributed "+
				"to it instead of published twice", fixed)))
	} else {
		notes = append(notes, Note(fmt.Sprintf(
			"%d fixed joint(s) attributed to %s: not published by this application, but resolvable by "+
				"TF.Lookup, because the URDF says what they are", fixed, opts.Publisher)))
	}
	if opts.FixedOnly {
		notes = append(notes, Note(fmt.Sprintf(
			"%d movable joint(s) left out (-fixed-only): with nothing publishing joint state, their "+
				"frames never appear on tf, and declaring them would tell the checker otherwise", moving)))
	} else {
		notes = append(notes, Note(fmt.Sprintf(
			"%d movable joint(s) are dynamic: their transforms are joint state, so look them up against "+
				"tf at runtime", moving)))
	}

	// The robot's own description can be malformed too, and it is better to say
	// so here than to emit a frames.json the checker will reject.
	for _, p := range tree.Check() {
		notes = append(notes, Note("the description's tree is not well formed: "+p.Msg))
	}
	if orphans := orphanLinks(r, tree); len(orphans) > 0 {
		notes = append(notes, Note(fmt.Sprintf(
			"%d link(s) are declared but no joint reaches them, so they are not frames: %v",
			len(orphans), orphans)))
	}
	return tree, notes
}

// orphanLinks are links no *joint* mentions: legal URDF, usually a mistake, and
// reported rather than invented into the tree. The comparison is against the
// joints rather than against the emitted tree, so a wheel left out by -fixed-only
// is not reported as an orphan — it is jointed, just not on tf.
func orphanLinks(r *Robot, _ *conductor.FrameTree) []string {
	jointed := map[string]bool{}
	for _, j := range r.Joints {
		jointed[j.Parent], jointed[j.Child] = true, true
	}
	var out []string
	for _, l := range r.Links {
		if !jointed[l] {
			out = append(out, l)
		}
	}
	sort.Strings(out)
	return out
}

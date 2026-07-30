package graph

import (
	"fmt"
	"sort"
	"strings"

	"conductor.dev/conductor"
	"conductor.dev/conductor/internal/scan"
)

// Frames are the other half of a ROS graph: a topic carries a message, and
// the message is only meaningful in a frame. Conductor declares the transform
// tree, so the checker can hold both the tags and the lookups written in code
// to it — the "unknown frame" and "no transform between these" failures that
// otherwise wait for a robot and RViz.

// validateFrames checks the declared transform tree and everything that names
// a frame.
func validateFrames(add func(Severity, string, string, string, ...any), app *scan.App) {
	tree := app.Frames

	for _, p := range tree.Check() {
		code := map[string]string{
			"multiple_parents": "CND051",
			"cycle":            "CND052",
			"disconnected":     "CND053",
		}[p.Kind]
		add(Error, code, framesPos(app), "%s: %s", app.FramesFile, p.Msg)
	}

	declaresTF := false
	for _, n := range app.Nodes {
		if n.TF != nil {
			declaresTF = true
		}
		for _, kind := range []struct {
			what string
			eps  []scan.Endpoint
		}{{"publishes", n.Pubs}, {"subscribes to", n.Subs}} {
			for _, e := range kind.eps {
				if e.Frame == "" {
					continue
				}
				pos := fmt.Sprintf("%s:%d", e.File, e.Line)
				if stamped, known := app.Stamped[e.GoType]; known && !stamped {
					add(Error, "CND057", pos,
						"node %s %s %q in frame %q, but %s has no Header field to carry a frame id",
						n.Name, kind.what, e.Topic, e.Frame, e.GoType)
				}
				switch {
				case tree == nil:
					add(Error, "CND050", pos,
						"node %s %s %q in frame %q, but the application declares no transform tree (add frames.json)",
						n.Name, kind.what, e.Topic, e.Frame)
				case !tree.Has(e.Frame):
					add(Error, "CND050", pos, "node %s %s %q in frame %q, which is not declared in %s (declared: %s)",
						n.Name, kind.what, e.Topic, e.Frame, app.FramesFile, strings.Join(tree.Frames(), ", "))
				}
			}
		}
		validateLookups(add, app, n)
	}

	if len(tree.Published()) > 0 && !declaresTF {
		add(Warning, "CND055", framesPos(app),
			"%s declares %d static transform(s) but no node declares a conductor.TF field, so nothing publishes tf_static",
			app.FramesFile, len(tree.Published()))
	}

	validateFramePairs(add, app)
}

// validateLookups resolves the TF.Lookup calls written with literal frames.
// This is the same check `TF.Lookup` performs at runtime, moved to build time
// for the calls that are decidable — which, for a robot's fixed geometry, is
// most of them.
func validateLookups(add func(Severity, string, string, string, ...any), app *scan.App, n *scan.Node) {
	if n.TF == nil {
		return
	}
	for _, c := range n.Calls {
		if c.Method != "Lookup" || len(c.Args) != 2 {
			continue
		}
		// Only lookups on this node's own TF field: Lookup is a common
		// method name, and a syntactic scanner has no types to tell them
		// apart.
		if !strings.HasSuffix(c.Recv, "."+n.TF.Field) {
			continue
		}
		pos := fmt.Sprintf("%s:%d", c.File, c.Line)
		if app.Frames == nil {
			add(Error, "CND050", pos, "node %s looks up %s -> %s, but the application declares no transform tree (add frames.json)",
				n.Name, c.Args[1], c.Args[0])
			continue
		}
		if _, err := app.Frames.Lookup(c.Args[0], c.Args[1]); err != nil {
			add(Error, "CND054", pos, "node %s: Lookup(%q, %q): %v", n.Name, c.Args[0], c.Args[1], err)
		}
	}
}

// validateFramePairs reports topics whose endpoints disagree about the frame
// the messages are in. It is a warning, not an error: consuming data in
// another frame is normal — that is what a transform is for — but doing it
// unknowingly is how a robot drives into a wall.
func validateFramePairs(add func(Severity, string, string, string, ...any), app *scan.App) {
	type endpoint struct {
		node, frame, pos string
	}
	byTopic := map[string][]endpoint{}
	order := []string{}
	for _, n := range app.Nodes {
		for _, eps := range [][]scan.Endpoint{n.Pubs, n.Subs} {
			for _, e := range eps {
				if e.Frame == "" || e.Topic == "" {
					continue
				}
				if _, seen := byTopic[e.Topic]; !seen {
					order = append(order, e.Topic)
				}
				byTopic[e.Topic] = append(byTopic[e.Topic], endpoint{n.Name, e.Frame, fmt.Sprintf("%s:%d", e.File, e.Line)})
			}
		}
	}
	sort.Strings(order)
	for _, topic := range order {
		eps := byTopic[topic]
		frames := map[string]bool{}
		for _, e := range eps {
			frames[e.frame] = true
		}
		if len(frames) < 2 {
			continue
		}
		names := make([]string, 0, len(frames))
		for f := range frames {
			names = append(names, f)
		}
		sort.Strings(names)
		last := eps[len(eps)-1]
		if err := app.Frames.Connects(names[0], names[1]); err != nil {
			add(Error, "CND056", last.pos, "topic %q: endpoints declare frames %s, and %v",
				topic, strings.Join(names, " and "), err)
			continue
		}
		add(Warning, "CND056", last.pos, "topic %q: endpoints declare different frames (%s); the consumer must transform",
			topic, strings.Join(names, " and "))
	}
}

func framesPos(app *scan.App) string {
	if app.FramesFile == "" {
		return ""
	}
	return app.Dir + "/" + app.FramesFile
}

// FrameLinks renders the declared tree for the check report.
func FrameLinks(t *conductor.FrameTree) []string {
	if t == nil {
		return nil
	}
	out := make([]string, 0, len(t.Transforms))
	for _, tf := range t.Transforms {
		kind := "static"
		if tf.Dynamic {
			kind = "dynamic, by " + orExternal(tf.By)
		}
		out = append(out, fmt.Sprintf("%-14s -> %-14s (%s)", tf.Parent, tf.Child, kind))
	}
	return out
}

func orExternal(s string) string {
	if s == "" {
		return "an external node"
	}
	return s
}

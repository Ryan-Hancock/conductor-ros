package graph

import (
	"fmt"
	"strings"

	"conductor.dev/conductor"
	"conductor.dev/conductor/internal/scan"
)

// A planning group is a string in a motion planning request, and a named
// configuration is six joint values copied out of an SRDF. Both are checkable
// against the robot's own semantics, which is what makes them declarations
// rather than folklore — the same move the frame checks make for frame ids.
func validateGroups(add func(Severity, string, string, string, ...any), app *scan.App) {
	declared := 0
	for _, n := range app.Nodes {
		declared += len(n.Groups)
	}
	if declared == 0 {
		return
	}
	if app.Semantics == nil {
		for _, n := range app.Nodes {
			for _, g := range n.Groups {
				add(Error, "CND070", fmt.Sprintf("%s:%d", g.File, g.Line),
					"node %s: field %s declares planning group %q, but the application has no groups.json "+
						"(derive one with `conductor groups -from robot.srdf`)", n.Name, g.Field, g.Name)
			}
		}
		return
	}

	for _, n := range app.Nodes {
		byField := map[string]string{} // field name -> group name, for resolving State calls
		for _, g := range n.Groups {
			pos := fmt.Sprintf("%s:%d", g.File, g.Line)
			if g.Name == "" {
				add(Error, "CND070", pos, "node %s: field %s is missing a group tag "+
					`(e.g. group:"panda_arm")`, n.Name, g.Field)
				continue
			}
			byField[g.Field] = g.Name
			if _, ok := app.Semantics.Group(g.Name); !ok {
				add(Error, "CND070", pos,
					"node %s: planning group %q is not in %s (it declares: %s)",
					n.Name, g.Name, app.GroupsFile, strings.Join(app.Semantics.Names(), ", "))
			}
		}
		validateGroupStates(add, app, n, byField)
	}
}

// validateGroupStates resolves the named configurations written as string
// literals — Arm.State("ready") — against the group that field declares. It is
// the same trick the mission and frame checks use: the branches taken in code
// are held to the declarations, not just the ones written in tags.
func validateGroupStates(add func(Severity, string, string, string, ...any), app *scan.App, n *scan.Node, byField map[string]string) {
	for _, call := range n.Calls {
		if call.Method != "State" || len(call.Args) != 1 {
			continue
		}
		// The receiver is written as "c.Arm" (or "Arm" inside a method on the
		// node); anything else is somebody else's State method.
		field := call.Recv
		if i := strings.LastIndex(field, "."); i >= 0 {
			field = field[i+1:]
		}
		groupName, ok := byField[field]
		if !ok {
			continue
		}
		group, ok := app.Semantics.Group(groupName)
		if !ok {
			continue // already reported as CND070
		}
		if _, found := stateOf(group, call.Args[0]); !found {
			states := make([]string, len(group.States))
			for i, s := range group.States {
				states[i] = s.Name
			}
			where := strings.Join(states, ", ")
			if where == "" {
				where = "none"
			}
			add(Error, "CND071", fmt.Sprintf("%s:%d", call.File, call.Line),
				"node %s: %s.State(%q) in %s: planning group %q has no such configuration in %s (it has: %s)",
				n.Name, call.Recv, call.Args[0], call.In, groupName, app.GroupsFile, where)
		}
	}
}

func stateOf(g conductor.PlanningGroup, name string) (int, bool) {
	for i, s := range g.States {
		if s.Name == name {
			return i, true
		}
	}
	return 0, false
}

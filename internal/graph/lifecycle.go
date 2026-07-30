package graph

import (
	"fmt"
	"strings"
	"time"

	"conductor.dev/conductor/internal/scan"
)

// A conductor.Lifecycle field declares the managed nodes an application drives:
// the list a Nav2 deployment keeps in a lifecycle_manager's `node_names`
// parameter, where nothing checks it.
//
// Most of what could be wrong with it is not decidable here, because the names
// belong to other people's processes — that is what `conductor externals`
// checks against a live graph. What *is* decidable is the shape of the list and
// whether the application is quietly managing itself, and those are worth
// catching before a robot does.
func validateLifecycle(add func(Severity, string, string, string, ...any), app *scan.App) {
	ours := map[string]bool{}
	for _, n := range app.Nodes {
		ours[n.Name] = true
	}

	for _, n := range app.Nodes {
		for _, decl := range n.Lifecycle {
			pos := fmt.Sprintf("%s:%d", decl.File, decl.Line)
			if len(decl.Nodes) == 0 {
				add(Error, "CND060", pos,
					"node %s: field %s has no nodes tag, so it manages nothing "+
						`(e.g. nodes:"map_server,amcl,bt_navigator")`, n.Name, decl.Field)
				continue
			}
			seen := map[string]bool{}
			for _, managed := range decl.Nodes {
				switch {
				case managed == "":
					add(Error, "CND060", pos,
						"node %s: field %s lists an empty node name", n.Name, decl.Field)
				case seen[managed]:
					add(Error, "CND060", pos,
						"node %s: field %s lists %q twice", n.Name, decl.Field, managed)
				case strings.ContainsAny(managed, " \t/"):
					add(Error, "CND060", pos,
						"node %s: field %s: %q is not a node name", n.Name, decl.Field, managed)
				}
				seen[managed] = true

				if ours[managed] {
					add(Warning, "CND061", pos,
						"node %s: field %s manages %q, which is a node of this application; "+
							"its bringup is already derived from the graph, so driving it by hand "+
							"is either redundant or a name collision with the node you meant",
						n.Name, decl.Field, managed)
				}
			}
			if decl.Timeout != "" {
				if d, err := time.ParseDuration(decl.Timeout); err != nil || d <= 0 {
					add(Error, "CND062", pos,
						"node %s: field %s has an invalid timeout %q", n.Name, decl.Field, decl.Timeout)
				}
			}
		}
	}
}

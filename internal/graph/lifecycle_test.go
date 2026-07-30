package graph

import (
	"strings"
	"testing"

	"conductor.dev/conductor/internal/scan"
)

// managerApp is an application whose commander node manages a stack.
func managerApp(decl scan.LifecycleDecl, extra ...*scan.Node) *scan.App {
	nodes := append([]*scan.Node{{
		Name: "commander", StructName: "Commander", File: "commander.go", Line: 10,
		Lifecycle: []scan.LifecycleDecl{decl},
		Methods:   map[string]scan.MethodSig{},
	}}, extra...)
	return &scan.App{Name: "app", Nodes: nodes, Messages: map[string]string{}, Stamped: map[string]bool{}}
}

func lifecycleDecl(nodes []string, timeout string) scan.LifecycleDecl {
	return scan.LifecycleDecl{Field: "Stack", Nodes: nodes, Timeout: timeout,
		File: "commander.go", Line: 12}
}

// The list is a declaration, so its mistakes are build errors: an empty list
// manages nothing, and a name repeated or malformed is a typo worth naming.
func TestLifecycleListIsValidated(t *testing.T) {
	cases := []struct {
		name  string
		decl  scan.LifecycleDecl
		code  string
		wants string
	}{
		{"no nodes", lifecycleDecl(nil, ""), "CND060", "manages nothing"},
		{"empty name", lifecycleDecl([]string{"amcl", ""}, ""), "CND060", "empty node name"},
		{"listed twice", lifecycleDecl([]string{"amcl", "amcl"}, ""), "CND060", "twice"},
		{"not a node name", lifecycleDecl([]string{"amcl/get_state"}, ""), "CND060", "is not a node name"},
		{"bad timeout", lifecycleDecl([]string{"amcl"}, "soon"), "CND062", "invalid timeout"},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			_, issues := Validate(managerApp(c.decl))
			var found bool
			for _, i := range issues {
				if i.Code == c.code && strings.Contains(i.Msg, c.wants) {
					found = true
				}
			}
			if !found {
				t.Errorf("no %s mentioning %q; got %v", c.code, c.wants, issues)
			}
		})
	}
}

// A valid list is silent, and a plausible Nav2 stack is a valid list.
func TestLifecycleValidListIsSilent(t *testing.T) {
	_, issues := Validate(managerApp(lifecycleDecl(
		[]string{"map_server", "amcl", "controller_server", "planner_server", "bt_navigator"}, "30s")))
	for _, i := range issues {
		if strings.HasPrefix(i.Code, "CND06") {
			t.Errorf("unexpected %s: %s", i.Code, i.Msg)
		}
	}
}

// Managing one of this application's own nodes is a warning: the runtime
// already brings those up in graph order, so either it is redundant or the name
// collides with the node that was meant.
func TestLifecycleManagingOurOwnNodeWarns(t *testing.T) {
	app := managerApp(lifecycleDecl([]string{"amcl", "navigator"}, ""),
		&scan.Node{Name: "navigator", StructName: "Navigator", Methods: map[string]scan.MethodSig{}})

	_, issues := Validate(app)
	var found bool
	for _, i := range issues {
		if i.Code == "CND061" {
			found = true
			if i.Severity != Warning {
				t.Errorf("CND061 severity = %v, want a warning", i.Severity)
			}
			if !strings.Contains(i.Msg, "navigator") {
				t.Errorf("CND061 does not name the node: %s", i.Msg)
			}
		}
	}
	if !found {
		t.Errorf("no CND061 for managing our own node; got %v", issues)
	}
}

package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"strings"

	"conductor.dev/conductor"
	"conductor.dev/conductor/internal/urdf"
)

// runGroups derives the planning groups from a robot's SRDF.
//
// It is the frames command's twin, for the other half of a robot description:
// the URDF says what the robot is, the SRDF says what parts of it a motion
// planner is asked about. A MoveIt-driving application names those groups and
// their configurations as strings, with the joint values copied in beside them
// — the same duplication frames.json had before it was derived.
func runGroups(args []string) error {
	fs := flag.NewFlagSet("groups", flag.ExitOnError)
	from := fs.String("from", "", "robot semantics to derive the groups from (a plain SRDF)")
	out := fs.String("o", "", "write groups.json here (default: stdout)")
	if err := fs.Parse(args); err != nil {
		return err
	}
	if *from == "" {
		return fmt.Errorf("groups: -from <robot.srdf> is required")
	}

	semantics, err := urdf.LoadSemantics(*from)
	if err != nil {
		return err
	}
	if semantics == nil {
		return fmt.Errorf("groups: %s does not exist", *from)
	}
	groups := urdf.Groups(semantics)

	doc, err := json.MarshalIndent(struct {
		Groups []conductor.PlanningGroup `json:"groups"`
	}{groups}, "", "  ")
	if err != nil {
		return err
	}
	doc = append(doc, '\n')

	if *out == "" {
		fmt.Print(string(doc))
		reportGroups(os.Stderr, semantics, groups)
		return nil
	}
	if err := os.MkdirAll(filepath.Dir(*out), 0o755); err != nil {
		return err
	}
	if err := os.WriteFile(*out, doc, 0o644); err != nil {
		return err
	}
	fmt.Printf("wrote %s from %s\n", *out, semantics.Path)
	reportGroups(os.Stdout, semantics, groups)
	return nil
}

func reportGroups(w *os.File, semantics *urdf.Semantics, groups []conductor.PlanningGroup) {
	fmt.Fprintf(w, "\n%s: %d planning group(s)\n", semantics.Name, len(groups))
	for _, g := range groups {
		what := fmt.Sprintf("%d joint(s)", len(g.Joints))
		if g.TipLink != "" {
			what = fmt.Sprintf("chain %s -> %s", g.BaseLink, g.TipLink)
		}
		if len(g.Subgroups) > 0 {
			what = "subgroups " + strings.Join(g.Subgroups, ", ")
		}
		states := make([]string, len(g.States))
		for i, s := range g.States {
			states[i] = s.Name
		}
		line := fmt.Sprintf("  %-16s %s", g.Name, what)
		if len(states) > 0 {
			line += fmt.Sprintf("; states: %s", strings.Join(states, ", "))
		}
		fmt.Fprintln(w, line)
	}
}

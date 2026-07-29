package graph

import (
	"fmt"
	"sort"
	"strconv"
	"time"

	"conductor.dev/conductor"
	"conductor.dev/conductor/internal/scan"
)

// A mission is a state machine written as declarations, so unlike a behaviour
// tree in an XML file it can be checked: every transition names a step that
// exists, every step is reachable from the start, and every step has a
// handler. The same machine is what gen draws in mission.dot.

// Machine is one node's mission, resolved.
type Machine struct {
	Node  string
	Name  string
	Start string
	Steps []MachineStep
	Pos   string
}

// MachineStep is one step and its outgoing transitions.
type MachineStep struct {
	Name      string
	Next      string
	Fail      string
	Gotos     []string // targets of literal Task.Goto calls in the handler
	Timeout   string
	Retry     string
	Backoff   string
	Reachable bool
	Pos       string
}

// Targets lists every step this one can lead to, tags and Gotos together.
func (s MachineStep) Targets() []string {
	var out []string
	for _, t := range append([]string{s.Next, s.Fail}, s.Gotos...) {
		if t == "" {
			continue
		}
		out = appendOnce(out, t)
	}
	return out
}

// Machines resolves every node's declared mission.
func Machines(app *scan.App) []Machine {
	var out []Machine
	for _, n := range app.Nodes {
		if len(n.Missions) == 0 {
			continue
		}
		m := n.Missions[0]
		machine := Machine{
			Node:  n.Name,
			Name:  m.Name,
			Start: m.Start,
			Pos:   fmt.Sprintf("%s:%d", m.File, m.Line),
		}
		for _, s := range n.Steps {
			next := s.Next
			if next == "" {
				next = conductor.StepDone
			}
			machine.Steps = append(machine.Steps, MachineStep{
				Name:    s.Name,
				Next:    next,
				Fail:    s.Fail,
				Gotos:   gotoTargets(n, s),
				Timeout: s.Timeout,
				Retry:   s.Retry,
				Backoff: s.Backoff,
				Pos:     fmt.Sprintf("%s:%d", s.File, s.Line),
			})
		}
		markReachable(&machine)
		out = append(out, machine)
	}
	sort.Slice(out, func(i, j int) bool { return out[i].Node < out[j].Node })
	return out
}

// gotoTargets returns the literal Goto targets written in a step's handler.
// A Goto elsewhere in the node is not a transition of this step, so the
// enclosing method has to be the step's own handler.
func gotoTargets(n *scan.Node, s scan.Step) []string {
	var out []string
	for _, c := range n.Calls {
		if c.Method == "Goto" && c.In == "On"+s.Field && len(c.Args) == 1 {
			out = appendOnce(out, c.Args[0])
		}
	}
	return out
}

// markReachable walks the machine from its start step.
func markReachable(m *Machine) {
	index := map[string]int{}
	for i, s := range m.Steps {
		index[s.Name] = i
	}
	seen := map[string]bool{}
	var walk func(name string)
	walk = func(name string) {
		i, ok := index[name]
		if !ok || seen[name] {
			return
		}
		seen[name] = true
		m.Steps[i].Reachable = true
		for _, t := range m.Steps[i].Targets() {
			walk(t)
		}
	}
	walk(m.Start)
}

// validateMissions checks the declared task machines.
func validateMissions(add func(Severity, string, string, string, ...any), app *scan.App) {
	for _, n := range app.Nodes {
		switch {
		case len(n.Missions) > 1:
			m := n.Missions[1]
			add(Error, "CND041", fmt.Sprintf("%s:%d", m.File, m.Line),
				"node %s declares %d missions; a node runs one mission", n.Name, len(n.Missions))
			continue
		case len(n.Missions) == 0 && len(n.Steps) > 0:
			s := n.Steps[0]
			add(Error, "CND041", fmt.Sprintf("%s:%d", s.File, s.Line),
				"node %s declares step %s but no conductor.Mission field to run it", n.Name, s.Field)
			continue
		case len(n.Missions) == 0:
			continue
		}

		m := n.Missions[0]
		pos := fmt.Sprintf("%s:%d", m.File, m.Line)
		declared := map[string]bool{}
		for _, s := range n.Steps {
			declared[s.Name] = true
		}
		if len(n.Steps) == 0 {
			add(Error, "CND041", pos, "node %s: mission %s declares no steps", n.Name, m.Name)
			continue
		}
		switch {
		case m.Start == "":
			add(Error, "CND041", pos, "node %s: mission %s is missing a start tag (e.g. start:%q)", n.Name, m.Name, n.Steps[0].Name)
		case !declared[m.Start]:
			add(Error, "CND041", pos, "node %s: mission %s starts at %q, which is not a step of this node (declared: %s)",
				n.Name, m.Name, m.Start, list(n.Steps))
		}

		for _, s := range n.Steps {
			spos := fmt.Sprintf("%s:%d", s.File, s.Line)
			checkHandler(add, n, s.Field, scan.MethodSig{Params: 1, Results: 1}, spos)
			for _, t := range []struct{ tag, target string }{{"next", s.Next}, {"fail", s.Fail}} {
				if t.target == "" || terminalStep(t.target) {
					continue
				}
				if !declared[t.target] {
					add(Error, "CND040", spos, "node %s: step %s has %s:%q, which is not a step of this node (declared: %s, or %s/%s to end the mission)",
						n.Name, s.Name, t.tag, t.target, list(n.Steps), conductor.StepDone, conductor.StepFailed)
				}
			}
			for _, c := range n.Calls {
				if c.Method != "Goto" || c.In != "On"+s.Field || len(c.Args) != 1 {
					continue
				}
				if !declared[c.Args[0]] && !terminalStep(c.Args[0]) {
					add(Error, "CND040", fmt.Sprintf("%s:%d", c.File, c.Line),
						"node %s: step %s calls Goto(%q), which is not a step of this node (declared: %s)",
						n.Name, s.Name, c.Args[0], list(n.Steps))
				}
			}
			if s.Timeout != "" {
				if d, err := time.ParseDuration(s.Timeout); err != nil || d <= 0 {
					add(Error, "CND043", spos, "node %s: step %s has an invalid timeout %q", n.Name, s.Name, s.Timeout)
				}
			}
			if s.Backoff != "" {
				if d, err := time.ParseDuration(s.Backoff); err != nil || d < 0 {
					add(Error, "CND043", spos, "node %s: step %s has an invalid backoff %q", n.Name, s.Name, s.Backoff)
				}
			}
			if s.Retry != "" {
				if _, err := strconv.Atoi(s.Retry); err != nil {
					add(Error, "CND043", spos, "node %s: step %s has an invalid retry count %q", n.Name, s.Name, s.Retry)
				}
			}
		}
	}

	// Reachability is a property of the whole machine, so it comes after the
	// per-step checks — and only when the machine resolved cleanly enough to
	// walk.
	for _, machine := range Machines(app) {
		for _, s := range machine.Steps {
			if !s.Reachable {
				add(Warning, "CND042", s.Pos, "node %s: step %s is unreachable from %s (nothing transitions to it)",
					machine.Node, s.Name, machine.Start)
			}
		}
	}
}

func terminalStep(name string) bool {
	return name == conductor.StepDone || name == conductor.StepFailed
}

func list(steps []scan.Step) string {
	names := make([]string, len(steps))
	for i, s := range steps {
		names[i] = s.Name
	}
	sort.Strings(names)
	out := ""
	for i, n := range names {
		if i > 0 {
			out += ", "
		}
		out += n
	}
	return out
}

func appendOnce(list []string, v string) []string {
	for _, x := range list {
		if x == v {
			return list
		}
	}
	return append(list, v)
}

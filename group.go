package conductor

import (
	"encoding/json"
	"fmt"
	"os"
	"reflect"
	"sort"
	"strings"
)

// Group is a planning group, declared rather than spelled.
//
// A MoveIt-driving application names its groups and their configurations as
// strings — `request.group_name = "panda_arm"`, and then six joint angles
// copied out of the SRDF into the code beside them. Both are the problem this
// framework exists to remove: a name nothing checks, and numbers duplicated out
// of a file that already holds them.
//
//	//conductor:node
//	type Commander struct {
//	    Arm  conductor.Group `group:"panda_arm"`
//	    Hand conductor.Group `group:"hand"`
//	}
//
//	func (c *Commander) OnPick(t *conductor.Task) error {
//	    ready, err := c.Arm.State("ready")   // joint names and values, from the SRDF
//	    ...
//	}
//
// `conductor check` resolves both against the robot's SRDF: a group that is not
// declared there, or a `State` written with a string literal that the group has
// no configuration for, is a build error rather than a planning request the
// move_group rejects at runtime.
//
// Tags: group (required) — the planning group's name in the SRDF.
type Group struct {
	name  string
	group PlanningGroup
	found bool
}

// PlanningGroup is one group of the robot's semantics: what a motion planning
// request names, and the configurations the SRDF defines for it.
type PlanningGroup struct {
	Name string `json:"name"`

	// How the group is composed, as the SRDF declared it: a chain between two
	// links (how an arm is usually written), an explicit set of links or
	// joints, or other groups. An application that asks "which joints am I
	// planning for?" should not have to read the SRDF again to find out.
	BaseLink  string   `json:"base_link,omitempty"`
	TipLink   string   `json:"tip_link,omitempty"`
	Links     []string `json:"links,omitempty"`
	Joints    []string `json:"joints,omitempty"`
	Subgroups []string `json:"subgroups,omitempty"`

	States []NamedState `json:"states,omitempty"`
}

// NamedState is a named joint configuration — the SRDF's group_state.
type NamedState struct {
	Name       string    `json:"name"`
	JointNames []string  `json:"joints,omitempty"`
	Positions  []float64 `json:"positions,omitempty"`
}

// Semantics is the robot's declared planning groups, as the runtime holds them.
type Semantics struct {
	Path   string
	Groups []PlanningGroup
}

// Group returns a planning group by name.
func (s *Semantics) Group(name string) (PlanningGroup, bool) {
	if s == nil {
		return PlanningGroup{}, false
	}
	for _, g := range s.Groups {
		if g.Name == name {
			return g, true
		}
	}
	return PlanningGroup{}, false
}

// Names lists the declared groups, sorted.
func (s *Semantics) Names() []string {
	if s == nil {
		return nil
	}
	out := make([]string, len(s.Groups))
	for i, g := range s.Groups {
		out[i] = g.Name
	}
	sort.Strings(out)
	return out
}

// LoadSemantics reads groups.json — the planning groups derived from an SRDF by
// `conductor groups -from robot.srdf`. A missing file is not an error: an
// application that declares no groups needs none.
//
// The runtime reads the derived file rather than the SRDF itself for the same
// reason it reads frames.json rather than a URDF: one loader, one format, and
// the robot description stays a build-time input.
func LoadSemantics(path string) (*Semantics, error) {
	b, err := os.ReadFile(path)
	if err != nil {
		if os.IsNotExist(err) {
			return nil, nil
		}
		return nil, err
	}
	var file struct {
		Groups []PlanningGroup `json:"groups"`
	}
	if err := json.Unmarshal(b, &file); err != nil {
		return nil, fmt.Errorf("%s: %w", path, err)
	}
	for _, g := range file.Groups {
		if g.Name == "" {
			return nil, fmt.Errorf("%s: every group needs a name", path)
		}
	}
	return &Semantics{Path: path, Groups: file.Groups}, nil
}

// Name is the group's name, which is what a planning request carries.
func (g *Group) Name() string { return g.name }

// Joints are the joints of the group, when the SRDF listed them.
func (g *Group) Joints() []string { return g.group.Joints }

// Links are the links of the group, when the SRDF listed them.
func (g *Group) Links() []string { return g.group.Links }

// Chain is the group's base and tip link, empty unless it was declared as a
// chain — which is how an arm is usually declared.
func (g *Group) Chain() (base, tip string) { return g.group.BaseLink, g.group.TipLink }

// State returns a named configuration of this group: the joint names and the
// values the SRDF gives them. This is the call that replaces six numbers copied
// into the application.
func (g *Group) State(name string) (NamedState, error) {
	if !g.found {
		return NamedState{}, fmt.Errorf("conductor: planning group %q is not in the declared semantics", g.name)
	}
	for _, s := range g.group.States {
		if s.Name == name {
			return s, nil
		}
	}
	return NamedState{}, fmt.Errorf("conductor: planning group %q has no state %q (it has: %s)",
		g.name, name, strings.Join(g.StateNames(), ", "))
}

// StateNames lists the group's named configurations.
func (g *Group) StateNames() []string {
	out := make([]string, len(g.group.States))
	for i, s := range g.group.States {
		out[i] = s.Name
	}
	return out
}

func (g *Group) bind(rt *runtimeState, nr *nodeRuntime, field reflect.StructField, ownerPtr reflect.Value) error {
	name := field.Tag.Get("group")
	if name == "" {
		return fmt.Errorf(`missing group tag (e.g. group:"panda_arm")`)
	}
	g.name = name
	if rt.semantics == nil {
		return fmt.Errorf("planning group %q is declared but no groups file was loaded "+
			"(derive one with `conductor groups -from robot.srdf`, and pass -groups)", name)
	}
	group, ok := rt.semantics.Group(name)
	if !ok {
		return fmt.Errorf("planning group %q is not in %s (it declares: %s)",
			name, rt.semantics.Path, strings.Join(rt.semantics.Names(), ", "))
	}
	g.group, g.found = group, true

	rt.recordEndpoint(Endpoint{
		Node: nr.name, Kind: EndpointGroup, Field: field.Name, Name: name,
		Type: "moveit_msgs/msg/MotionPlanRequest.group_name",
		Rate: fmt.Sprintf("%d joint(s), %d named state(s)", len(group.Joints)+chainJoints(group), len(group.States)),
	})
	return nil
}

// chainJoints reports 1 when the group was declared as a chain, so the summary
// says something rather than "0 joints" for an arm.
func chainJoints(g PlanningGroup) int {
	if g.BaseLink != "" || g.TipLink != "" {
		return 1
	}
	return 0
}

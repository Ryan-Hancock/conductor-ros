package urdf

import (
	"encoding/xml"
	"fmt"
	"os"
	"sort"
	"strconv"
	"strings"

	conductor "conductor.dev/conductor"
)

// An SRDF is the other half of a robot description: the URDF says what the
// robot is, the SRDF says what parts of it are worth planning for. Its
// **groups** name the kinematic chains a motion planner is asked about
// ("panda_arm", "hand"), and its **group states** are named joint
// configurations ("ready", "open") — both of which a MoveIt-driving
// application refers to by string today, with the joint values copied into the
// code beside them.
//
// That is the same problem frames had: a name nothing checks and numbers
// duplicated out of a file that already holds them. So the SRDF is read, and
// `conductor.Group` makes a planning group a declaration.

// Semantics is the parsed subset of an SRDF.
type Semantics struct {
	Name   string
	Groups []Group
	Path   string
}

// Group is a planning group: what a MoveGroup goal names, and the named
// configurations defined for it.
type Group struct {
	Name string

	// Chain, Links, Joints and Subgroups are how the group is composed. They
	// are kept because an application that asks "which joints am I planning
	// for?" should not have to parse this again.
	BaseLink, TipLink string
	Links             []string
	Joints            []string
	Subgroups         []string

	// States are the named configurations, in declaration order.
	States []GroupState
}

// GroupState is a named joint configuration, the SRDF's answer to hard-coding
// six joint angles in application code.
type GroupState struct {
	Name       string
	JointNames []string
	Positions  []float64
}

// State returns a named configuration of this group.
func (g Group) State(name string) (GroupState, bool) {
	for _, s := range g.States {
		if s.Name == name {
			return s, true
		}
	}
	return GroupState{}, false
}

// StateNames lists the named configurations, for the error message that
// matters when one is misspelled.
func (g Group) StateNames() []string {
	out := make([]string, len(g.States))
	for i, s := range g.States {
		out[i] = s.Name
	}
	return out
}

// Group returns a planning group by name.
func (s *Semantics) Group(name string) (Group, bool) {
	if s == nil {
		return Group{}, false
	}
	for _, g := range s.Groups {
		if g.Name == name {
			return g, true
		}
	}
	return Group{}, false
}

// GroupNames lists the declared groups, sorted.
func (s *Semantics) GroupNames() []string {
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

type xmlSemantics struct {
	XMLName xml.Name        `xml:"robot"`
	Name    string          `xml:"name,attr"`
	Groups  []xmlGroup      `xml:"group"`
	States  []xmlGroupState `xml:"group_state"`
}

type xmlGroup struct {
	Name      string        `xml:"name,attr"`
	Chains    []xmlChain    `xml:"chain"`
	Links     []xmlNamed    `xml:"link"`
	Joints    []xmlNamed    `xml:"joint"`
	Subgroups []xmlSubgroup `xml:"group"`
}

type xmlChain struct {
	BaseLink string `xml:"base_link,attr"`
	TipLink  string `xml:"tip_link,attr"`
}

type xmlNamed struct {
	Name string `xml:"name,attr"`
}

type xmlSubgroup struct {
	Name string `xml:"name,attr"`
}

type xmlGroupState struct {
	Name   string          `xml:"name,attr"`
	Group  string          `xml:"group,attr"`
	Joints []xmlStateJoint `xml:"joint"`
}

type xmlStateJoint struct {
	Name  string `xml:"name,attr"`
	Value string `xml:"value,attr"`
}

// LoadSemantics reads an SRDF file. A missing file is not an error: an
// application that declares no planning groups needs none.
func LoadSemantics(path string) (*Semantics, error) {
	b, err := os.ReadFile(path)
	if err != nil {
		if os.IsNotExist(err) {
			return nil, nil
		}
		return nil, err
	}
	s, err := ParseSemantics(b)
	if err != nil {
		return nil, fmt.Errorf("%s: %w", path, err)
	}
	s.Path = path
	return s, nil
}

// ParseSemantics reads an SRDF document. Like a URDF it is refused while it is
// still xacro, for the same reason: a `${...}` where a joint value belongs
// would become a silent zero, and a wrong joint value is a robot moving
// somewhere unexpected.
func ParseSemantics(doc []byte) (*Semantics, error) {
	if marker, ok := unexpandedXacro(doc); ok {
		return nil, fmt.Errorf("this is a xacro document, not a plain SRDF (found %s): expand it first, "+
			"e.g. `xacro panda.srdf.xacro > panda.srdf`", marker)
	}

	var x xmlSemantics
	if err := xml.Unmarshal(doc, &x); err != nil {
		return nil, fmt.Errorf("not an SRDF: %w", err)
	}
	if x.XMLName.Local != "robot" {
		return nil, fmt.Errorf("not an SRDF: root element is <%s>, want <robot>", x.XMLName.Local)
	}

	s := &Semantics{Name: x.Name}
	byName := map[string]int{}
	for _, g := range x.Groups {
		if g.Name == "" {
			return nil, fmt.Errorf("a <group> has no name")
		}
		if _, dup := byName[g.Name]; dup {
			return nil, fmt.Errorf("group %q is declared twice", g.Name)
		}
		group := Group{Name: g.Name}
		if len(g.Chains) > 0 {
			group.BaseLink, group.TipLink = g.Chains[0].BaseLink, g.Chains[0].TipLink
		}
		for _, l := range g.Links {
			group.Links = append(group.Links, l.Name)
		}
		for _, j := range g.Joints {
			group.Joints = append(group.Joints, j.Name)
		}
		for _, sub := range g.Subgroups {
			group.Subgroups = append(group.Subgroups, sub.Name)
		}
		byName[g.Name] = len(s.Groups)
		s.Groups = append(s.Groups, group)
	}
	if len(s.Groups) == 0 {
		return nil, fmt.Errorf("robot %q declares no planning groups", s.Name)
	}

	for _, st := range x.States {
		i, ok := byName[st.Group]
		if !ok {
			return nil, fmt.Errorf("group state %q names group %q, which is not declared", st.Name, st.Group)
		}
		state := GroupState{Name: st.Name}
		for _, j := range st.Joints {
			v, err := strconv.ParseFloat(strings.TrimSpace(j.Value), 64)
			if err != nil {
				return nil, fmt.Errorf("group state %q, joint %q: value %q: %w", st.Name, j.Name, j.Value, err)
			}
			state.JointNames = append(state.JointNames, j.Name)
			state.Positions = append(state.Positions, v)
		}
		s.Groups[i].States = append(s.Groups[i].States, state)
	}
	return s, nil
}

// Groups converts the parsed semantics into the runtime's own type, which is
// what a conductor.Group field is bound against — the same split the frames
// work uses, so the toolchain and the runtime read one file through one loader.
func Groups(s *Semantics) []conductor.PlanningGroup {
	if s == nil {
		return nil
	}
	out := make([]conductor.PlanningGroup, 0, len(s.Groups))
	for _, g := range s.Groups {
		pg := conductor.PlanningGroup{
			Name: g.Name, BaseLink: g.BaseLink, TipLink: g.TipLink,
			Links: g.Links, Joints: g.Joints, Subgroups: g.Subgroups,
		}
		for _, st := range g.States {
			pg.States = append(pg.States, conductor.NamedState{
				Name: st.Name, JointNames: st.JointNames, Positions: st.Positions,
			})
		}
		out = append(out, pg)
	}
	return out
}

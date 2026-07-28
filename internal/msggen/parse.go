// Package msggen generates Go message types from ROS 2 .msg definitions:
// structs wired for conductor's CDR codec, //ros:type directives for the
// static toolchain, and RegisterMessage calls carrying REP-2011 RIHS01 type
// hashes computed by this package — so custom interfaces need no ROS
// installation at build time.
package msggen

import (
	"fmt"
	"strconv"
	"strings"
)

// Message is a parsed .msg definition, or one side of a .srv definition.
type Message struct {
	Pkg       string // ROS package, e.g. "std_msgs"
	Name      string // type name, e.g. "String" or "SetBool_Request"
	Kind      string // interface namespace: "msg" (default) or "srv"
	Fields    []Field
	Constants []Constant
}

// FullName returns the fully qualified interface name ("std_msgs/msg/String").
func (m *Message) FullName() string {
	kind := m.Kind
	if kind == "" {
		kind = "msg"
	}
	return m.Pkg + "/" + kind + "/" + m.Name
}

type Field struct {
	Name    string
	Type    FieldType
	Default string // raw default-value text; informational only (not hashed)
}

type Constant struct {
	Type  string // builtin type name as written
	Name  string
	Value string // raw value text
}

type ContainerKind int

const (
	Scalar ContainerKind = iota
	Array
	BoundedSeq
	UnboundedSeq
)

// FieldType is a normalized .msg field type: either a builtin (Builtin set)
// or a reference to another message (Nested set to "pkg/msg/Name"), possibly
// wrapped in an array/sequence container.
type FieldType struct {
	Builtin     string // normalized builtin name ("bool", "uint8", ..., "string")
	Nested      string // fully qualified nested type name, if not a builtin
	Kind        ContainerKind
	Capacity    int // array size or sequence bound
	StrCapacity int // bounded-string capacity
}

// builtinNames are the .msg builtins after normalization (char -> uint8).
var builtinNames = map[string]bool{
	"bool": true, "byte": true,
	"int8": true, "uint8": true, "int16": true, "uint16": true,
	"int32": true, "uint32": true, "int64": true, "uint64": true,
	"float32": true, "float64": true,
	"string": true, "wstring": true,
}

// ParseMessage parses .msg source. Messages without fields receive the
// structure_needs_at_least_one_member placeholder, mirroring rosidl (it is
// part of the wire format and the type hash).
func ParseMessage(pkg, name string, src []byte) (*Message, error) {
	m := &Message{Pkg: pkg, Name: name}
	for lineNo, raw := range strings.Split(string(src), "\n") {
		line := strings.TrimSpace(stripComment(raw))
		if line == "" {
			continue
		}
		if err := parseLine(m, line); err != nil {
			return nil, fmt.Errorf("%s/%s.msg:%d: %w", pkg, name, lineNo+1, err)
		}
	}
	if len(m.Fields) == 0 {
		m.Fields = append(m.Fields, Field{
			Name: "structure_needs_at_least_one_member",
			Type: FieldType{Builtin: "uint8"},
		})
	}
	return m, nil
}

// Service is a parsed .srv definition (Kind "srv") or a service synthesized
// for an action (Kind "action"). Event is the <Name>_Event message rosidl
// synthesizes for service introspection; it is part of the service's type
// description (and therefore its hash) even though conductor does not
// publish events.
type Service struct {
	Pkg      string
	Name     string
	Kind     string // interface namespace: "srv" (default) or "action"
	Request  *Message
	Response *Message
	Event    *Message
}

// FullName returns the fully qualified service name ("std_srvs/srv/SetBool").
func (s *Service) FullName() string {
	kind := s.Kind
	if kind == "" {
		kind = "srv"
	}
	return s.Pkg + "/" + kind + "/" + s.Name
}

// newService assembles a service from parsed request/response messages,
// synthesizing the _Event message.
func newService(pkg, name, kind string, req, res *Message) *Service {
	req.Kind, res.Kind = kind, kind
	svc := &Service{Pkg: pkg, Name: name, Kind: kind, Request: req, Response: res}
	svc.Event = &Message{Pkg: pkg, Name: name + "_Event", Kind: kind, Fields: []Field{
		{Name: "info", Type: FieldType{Nested: "service_msgs/msg/ServiceEventInfo"}},
		{Name: "request", Type: FieldType{Nested: req.FullName(), Kind: BoundedSeq, Capacity: 1}},
		{Name: "response", Type: FieldType{Nested: res.FullName(), Kind: BoundedSeq, Capacity: 1}},
	}}
	return svc
}

// splitSections splits interface source on "---" separator lines, requiring
// exactly want sections.
func splitSections(src []byte, want int) ([][]byte, error) {
	sections := make([][]string, 1, want)
	for _, line := range strings.Split(string(src), "\n") {
		if strings.TrimSpace(line) == "---" {
			sections = append(sections, nil)
			continue
		}
		sections[len(sections)-1] = append(sections[len(sections)-1], line)
	}
	if len(sections) != want {
		return nil, fmt.Errorf("want %d sections separated by ---, got %d", want, len(sections))
	}
	out := make([][]byte, want)
	for i, s := range sections {
		out[i] = []byte(strings.Join(s, "\n"))
	}
	return out, nil
}

// ParseService parses .srv source: request fields, a "---" separator, then
// response fields.
func ParseService(pkg, name string, src []byte) (*Service, error) {
	sections, err := splitSections(src, 2)
	if err != nil {
		return nil, fmt.Errorf("%s/%s.srv: %w", pkg, name, err)
	}
	req, err := ParseMessage(pkg, name+"_Request", sections[0])
	if err != nil {
		return nil, err
	}
	res, err := ParseMessage(pkg, name+"_Response", sections[1])
	if err != nil {
		return nil, err
	}
	return newService(pkg, name, "srv", req, res), nil
}

// ActionDef is a parsed .action definition plus everything rosidl
// synthesizes from it: the _SendGoal/_GetResult services and the
// _FeedbackMessage wrapper.
type ActionDef struct {
	Pkg, Name       string
	Goal            *Message
	Result          *Message
	Feedback        *Message
	FeedbackMessage *Message
	SendGoal        *Service
	GetResult       *Service
}

// FullName returns the fully qualified action name
// ("example_interfaces/action/Fibonacci").
func (a *ActionDef) FullName() string { return a.Pkg + "/action/" + a.Name }

// ParseAction parses .action source: goal, result, and feedback sections.
func ParseAction(pkg, name string, src []byte) (*ActionDef, error) {
	sections, err := splitSections(src, 3)
	if err != nil {
		return nil, fmt.Errorf("%s/%s.action: %w", pkg, name, err)
	}
	a := &ActionDef{Pkg: pkg, Name: name}
	for i, part := range []struct {
		suffix string
		dst    **Message
	}{{"_Goal", &a.Goal}, {"_Result", &a.Result}, {"_Feedback", &a.Feedback}} {
		m, err := ParseMessage(pkg, name+part.suffix, sections[i])
		if err != nil {
			return nil, err
		}
		m.Kind = "action"
		*part.dst = m
	}

	uuid := FieldType{Nested: "unique_identifier_msgs/msg/UUID"}
	a.FeedbackMessage = &Message{Pkg: pkg, Name: name + "_FeedbackMessage", Kind: "action", Fields: []Field{
		{Name: "goal_id", Type: uuid},
		{Name: "feedback", Type: FieldType{Nested: a.Feedback.FullName()}},
	}}
	a.SendGoal = newService(pkg, name+"_SendGoal", "action",
		&Message{Pkg: pkg, Name: name + "_SendGoal_Request", Fields: []Field{
			{Name: "goal_id", Type: uuid},
			{Name: "goal", Type: FieldType{Nested: a.Goal.FullName()}},
		}},
		&Message{Pkg: pkg, Name: name + "_SendGoal_Response", Fields: []Field{
			{Name: "accepted", Type: FieldType{Builtin: "bool"}},
			{Name: "stamp", Type: FieldType{Nested: "builtin_interfaces/msg/Time"}},
		}})
	a.GetResult = newService(pkg, name+"_GetResult", "action",
		&Message{Pkg: pkg, Name: name + "_GetResult_Request", Fields: []Field{
			{Name: "goal_id", Type: uuid},
		}},
		&Message{Pkg: pkg, Name: name + "_GetResult_Response", Fields: []Field{
			{Name: "status", Type: FieldType{Builtin: "int8"}},
			{Name: "result", Type: FieldType{Nested: a.Result.FullName()}},
		}})
	return a, nil
}

// stripComment removes a trailing # comment, respecting quoted strings in
// default values.
func stripComment(line string) string {
	var quote byte
	for i := 0; i < len(line); i++ {
		c := line[i]
		switch {
		case quote != 0:
			if c == quote {
				quote = 0
			}
		case c == '"' || c == '\'':
			quote = c
		case c == '#':
			return line[:i]
		}
	}
	return line
}

func parseLine(m *Message, line string) error {
	typeTok, rest, ok := strings.Cut(line, " ")
	rest = strings.TrimSpace(rest)
	if !ok || rest == "" {
		return fmt.Errorf("expected \"type name\", got %q", line)
	}

	// A constant is "TYPE NAME=VALUE" (spaces around '=' allowed); a field
	// default never follows its name directly with '='.
	nameEnd := strings.IndexAny(rest, " \t=")
	name, tail := rest, ""
	if nameEnd >= 0 {
		name, tail = rest[:nameEnd], strings.TrimSpace(rest[nameEnd:])
	}
	if strings.HasPrefix(tail, "=") {
		if !builtinNames[normalizeBuiltin(typeTok)] {
			return fmt.Errorf("constant %s must have a builtin type, got %q", name, typeTok)
		}
		m.Constants = append(m.Constants, Constant{
			Type:  normalizeBuiltin(typeTok),
			Name:  name,
			Value: strings.TrimSpace(strings.TrimPrefix(tail, "=")),
		})
		return nil
	}

	ft, err := parseTypeToken(typeTok, m.Pkg)
	if err != nil {
		return err
	}
	m.Fields = append(m.Fields, Field{Name: name, Type: ft, Default: tail})
	return nil
}

func normalizeBuiltin(s string) string {
	if s == "char" {
		return "uint8" // rosidl maps .msg char to uint8
	}
	return s
}

func parseTypeToken(tok, pkg string) (FieldType, error) {
	var ft FieldType
	base := tok

	if i := strings.Index(tok, "["); i >= 0 {
		if !strings.HasSuffix(tok, "]") {
			return ft, fmt.Errorf("malformed array type %q", tok)
		}
		base = tok[:i]
		inner := tok[i+1 : len(tok)-1]
		switch {
		case inner == "":
			ft.Kind = UnboundedSeq
		case strings.HasPrefix(inner, "<="):
			n, err := strconv.Atoi(inner[2:])
			if err != nil || n <= 0 {
				return ft, fmt.Errorf("invalid sequence bound in %q", tok)
			}
			ft.Kind, ft.Capacity = BoundedSeq, n
		default:
			n, err := strconv.Atoi(inner)
			if err != nil || n <= 0 {
				return ft, fmt.Errorf("invalid array size in %q", tok)
			}
			ft.Kind, ft.Capacity = Array, n
		}
	}

	if i := strings.Index(base, "<="); i >= 0 {
		n, err := strconv.Atoi(base[i+2:])
		if err != nil || n <= 0 {
			return ft, fmt.Errorf("invalid string bound in %q", tok)
		}
		ft.StrCapacity = n
		base = base[:i]
		if base != "string" && base != "wstring" {
			return ft, fmt.Errorf("bound <=N is only valid on string types, got %q", tok)
		}
	}

	base = normalizeBuiltin(base)
	switch {
	case base == "wstring":
		return ft, fmt.Errorf("wstring is not supported")
	case builtinNames[base]:
		ft.Builtin = base
	case base == "Header":
		ft.Nested = "std_msgs/msg/Header"
	case strings.Contains(base, "/"):
		parts := strings.Split(base, "/")
		switch len(parts) {
		case 2:
			ft.Nested = parts[0] + "/msg/" + parts[1]
		case 3:
			ft.Nested = base
		default:
			return ft, fmt.Errorf("malformed type reference %q", tok)
		}
	default:
		ft.Nested = pkg + "/msg/" + base
	}
	return ft, nil
}

package msggen

import (
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
)

// Resolver locates and parses message definitions by fully qualified name,
// from explicitly registered local .msg files first, then from ROS-style
// share trees (<prefix>/<pkg>/msg/<Name>.msg).
type Resolver struct {
	prefixes []string
	local    map[string]string // full name -> file path (.msg, .srv, .action)
	cache    map[string]*Message
	svcCache map[string]*Service
	actCache map[string]*ActionDef
}

// NewResolver creates a resolver searching the given share directories.
func NewResolver(sharePrefixes []string) *Resolver {
	return &Resolver{
		prefixes: sharePrefixes,
		local:    map[string]string{},
		cache:    map[string]*Message{},
		svcCache: map[string]*Service{},
		actCache: map[string]*ActionDef{},
	}
}

// SharePrefixesFromEnv derives share directories from an AMENT_PREFIX_PATH
// value ("/opt/ros/x:/other" -> ".../share").
func SharePrefixesFromEnv(amentPrefixPath string) []string {
	var out []string
	for _, p := range strings.Split(amentPrefixPath, ":") {
		if p != "" {
			out = append(out, filepath.Join(p, "share"))
		}
	}
	return out
}

// AddLocal registers a local .msg or .srv file under rosPkg; its type name
// is the file basename. Returns the fully qualified name.
func (r *Resolver) AddLocal(rosPkg, path string) (string, error) {
	base := filepath.Base(path)
	switch {
	case strings.HasSuffix(base, ".msg"):
		full := rosPkg + "/msg/" + strings.TrimSuffix(base, ".msg")
		r.local[full] = path
		return full, nil
	case strings.HasSuffix(base, ".srv"):
		full := rosPkg + "/srv/" + strings.TrimSuffix(base, ".srv")
		r.local[full] = path
		return full, nil
	case strings.HasSuffix(base, ".action"):
		full := rosPkg + "/action/" + strings.TrimSuffix(base, ".action")
		r.local[full] = path
		return full, nil
	default:
		return "", fmt.Errorf("%s: not a .msg, .srv, or .action file", path)
	}
}

// IsServiceName reports whether a fully qualified name refers to a service
// itself (pkg/srv/Name) rather than one of its request/response/event
// sub-messages.
func IsServiceName(fullName string) bool {
	parts := strings.Split(fullName, "/")
	if len(parts) != 3 || parts[1] != "srv" {
		return false
	}
	for _, suffix := range []string{"_Request", "_Response", "_Event"} {
		if strings.HasSuffix(parts[2], suffix) {
			return false
		}
	}
	return true
}

// actionSuffixes are the names rosidl derives from an action, longest-match
// first so suffix stripping finds the action's base name.
var actionSuffixes = []string{
	"_SendGoal_Request", "_SendGoal_Response", "_SendGoal_Event", "_SendGoal",
	"_GetResult_Request", "_GetResult_Response", "_GetResult_Event", "_GetResult",
	"_FeedbackMessage", "_Feedback", "_Goal", "_Result",
}

// IsActionName reports whether a fully qualified name refers to an action
// itself (pkg/action/Name) rather than one of its derived types.
func IsActionName(fullName string) bool {
	parts := strings.Split(fullName, "/")
	if len(parts) != 3 || parts[1] != "action" {
		return false
	}
	for _, suffix := range actionSuffixes {
		if strings.HasSuffix(parts[2], suffix) {
			return false
		}
	}
	return true
}

func actionBase(name string) string {
	for _, suffix := range actionSuffixes {
		if strings.HasSuffix(name, suffix) {
			return strings.TrimSuffix(name, suffix)
		}
	}
	return name
}

// ResolveAction parses the definition of a fully qualified action name and
// caches every derived message and service.
func (r *Resolver) ResolveAction(fullName string) (*ActionDef, error) {
	if a, ok := r.actCache[fullName]; ok {
		return a, nil
	}
	parts := strings.Split(fullName, "/")
	if len(parts) != 3 || parts[1] != "action" {
		return nil, fmt.Errorf("invalid action type name %q (want pkg/action/Name)", fullName)
	}
	path, ok := r.local[fullName]
	if !ok {
		for _, prefix := range r.prefixes {
			candidate := filepath.Join(prefix, parts[0], "action", parts[2]+".action")
			if _, err := os.Stat(candidate); err == nil {
				path = candidate
				break
			}
		}
	}
	if path == "" {
		return nil, fmt.Errorf("cannot find %s.action for %s (searched local files and %d share prefix(es))",
			parts[2], fullName, len(r.prefixes))
	}
	src, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	a, err := ParseAction(parts[0], parts[2], src)
	if err != nil {
		return nil, err
	}
	r.actCache[fullName] = a
	for _, m := range []*Message{a.Goal, a.Result, a.Feedback, a.FeedbackMessage} {
		r.cache[m.FullName()] = m
	}
	for _, s := range []*Service{a.SendGoal, a.GetResult} {
		r.svcCache[s.FullName()] = s
		r.cache[s.Request.FullName()] = s.Request
		r.cache[s.Response.FullName()] = s.Response
		r.cache[s.Event.FullName()] = s.Event
	}
	return a, nil
}

// ResolveService parses the definition of a fully qualified service name.
func (r *Resolver) ResolveService(fullName string) (*Service, error) {
	if s, ok := r.svcCache[fullName]; ok {
		return s, nil
	}
	parts := strings.Split(fullName, "/")
	if len(parts) != 3 || parts[1] != "srv" {
		return nil, fmt.Errorf("invalid service type name %q (want pkg/srv/Name)", fullName)
	}
	path, ok := r.local[fullName]
	if !ok {
		for _, prefix := range r.prefixes {
			candidate := filepath.Join(prefix, parts[0], "srv", parts[2]+".srv")
			if _, err := os.Stat(candidate); err == nil {
				path = candidate
				break
			}
		}
	}
	if path == "" {
		return nil, fmt.Errorf("cannot find %s.srv for %s (searched local files and %d share prefix(es))",
			parts[2], fullName, len(r.prefixes))
	}
	src, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	s, err := ParseService(parts[0], parts[2], src)
	if err != nil {
		return nil, err
	}
	r.svcCache[fullName] = s
	r.cache[s.Request.FullName()] = s.Request
	r.cache[s.Response.FullName()] = s.Response
	r.cache[s.Event.FullName()] = s.Event
	return s, nil
}

// Resolve parses the definition of a fully qualified message type name,
// including service sub-messages (pkg/srv/Name_Request etc.), which are
// resolved by parsing their parent .srv file.
func (r *Resolver) Resolve(fullName string) (*Message, error) {
	if m, ok := r.cache[fullName]; ok {
		return m, nil
	}
	parts := strings.Split(fullName, "/")
	if len(parts) == 3 && parts[1] == "srv" {
		base := parts[2]
		for _, suffix := range []string{"_Request", "_Response", "_Event"} {
			base = strings.TrimSuffix(base, suffix)
		}
		if _, err := r.ResolveService(parts[0] + "/srv/" + base); err != nil {
			return nil, err
		}
		m, ok := r.cache[fullName]
		if !ok {
			return nil, fmt.Errorf("%s is not a request/response/event of service %s", fullName, base)
		}
		return m, nil
	}
	if len(parts) == 3 && parts[1] == "action" {
		if _, err := r.ResolveAction(parts[0] + "/action/" + actionBase(parts[2])); err != nil {
			return nil, err
		}
		m, ok := r.cache[fullName]
		if !ok {
			return nil, fmt.Errorf("%s is not a derived type of action %s", fullName, actionBase(parts[2]))
		}
		return m, nil
	}
	if len(parts) != 3 || parts[1] != "msg" {
		return nil, fmt.Errorf("invalid message type name %q (want pkg/msg/Name)", fullName)
	}
	path, ok := r.local[fullName]
	if !ok {
		for _, prefix := range r.prefixes {
			candidate := filepath.Join(prefix, parts[0], "msg", parts[2]+".msg")
			if _, err := os.Stat(candidate); err == nil {
				path = candidate
				break
			}
		}
	}
	if path == "" {
		return nil, fmt.Errorf("cannot find %s.msg for %s (searched local files and %d share prefix(es))",
			parts[2], fullName, len(r.prefixes))
	}
	src, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	m, err := ParseMessage(parts[0], parts[2], src)
	if err != nil {
		return nil, err
	}
	r.cache[fullName] = m
	return m, nil
}

// Describe resolves fullName — a message, a service sub-message, a service,
// or an action — and all transitively referenced types into a hashable
// TypeDescription.
func (r *Resolver) Describe(fullName string) (TypeDescription, error) {
	switch {
	case IsActionName(fullName):
		return r.describeAction(fullName)
	case IsServiceName(fullName) || isActionServiceName(fullName):
		svc, err := r.serviceByName(fullName)
		if err != nil {
			return TypeDescription{}, err
		}
		refs := map[string]Individual{}
		if err := r.addServiceRefs(svc, refs); err != nil {
			return TypeDescription{}, err
		}
		delete(refs, svc.FullName())
		return assemble(serviceIndividual(svc), refs), nil
	}
	root, err := r.Resolve(fullName)
	if err != nil {
		return TypeDescription{}, err
	}
	refs := map[string]Individual{}
	if err := r.collectRefs(root, refs); err != nil {
		return TypeDescription{}, err
	}
	return assemble(individualOf(root), refs), nil
}

// isActionServiceName matches an action's derived services
// (pkg/action/Name_SendGoal, pkg/action/Name_GetResult).
func isActionServiceName(fullName string) bool {
	parts := strings.Split(fullName, "/")
	return len(parts) == 3 && parts[1] == "action" &&
		(strings.HasSuffix(parts[2], "_SendGoal") || strings.HasSuffix(parts[2], "_GetResult"))
}

func (r *Resolver) serviceByName(fullName string) (*Service, error) {
	parts := strings.Split(fullName, "/")
	if parts[1] == "srv" {
		return r.ResolveService(fullName)
	}
	if _, err := r.ResolveAction(parts[0] + "/action/" + actionBase(parts[2])); err != nil {
		return nil, err
	}
	svc, ok := r.svcCache[fullName]
	if !ok {
		return nil, fmt.Errorf("%s is not a service of action %s", fullName, actionBase(parts[2]))
	}
	return svc, nil
}

// serviceIndividual is the service-level description rosidl hashes: three
// nested members (request/response/event messages).
func serviceIndividual(svc *Service) Individual {
	return Individual{TypeName: svc.FullName(), Fields: []DescField{
		{Name: "request_message", TypeID: idNested, NestedTypeName: svc.Request.FullName()},
		{Name: "response_message", TypeID: idNested, NestedTypeName: svc.Response.FullName()},
		{Name: "event_message", TypeID: idNested, NestedTypeName: svc.Event.FullName()},
	}}
}

// addServiceRefs records the service's own individual plus its sub-messages
// and their transitive references.
func (r *Resolver) addServiceRefs(svc *Service, refs map[string]Individual) error {
	refs[svc.FullName()] = serviceIndividual(svc)
	for _, m := range []*Message{svc.Request, svc.Response, svc.Event} {
		refs[m.FullName()] = individualOf(m)
		if err := r.collectRefs(m, refs); err != nil {
			return err
		}
	}
	return nil
}

// describeAction builds the action-level description rosidl hashes: six
// members (goal/result/feedback messages, the two derived services, and the
// feedback wrapper message), with everything they reference.
func (r *Resolver) describeAction(fullName string) (TypeDescription, error) {
	a, err := r.ResolveAction(fullName)
	if err != nil {
		return TypeDescription{}, err
	}
	ind := Individual{TypeName: fullName, Fields: []DescField{
		{Name: "goal", TypeID: idNested, NestedTypeName: a.Goal.FullName()},
		{Name: "result", TypeID: idNested, NestedTypeName: a.Result.FullName()},
		{Name: "feedback", TypeID: idNested, NestedTypeName: a.Feedback.FullName()},
		{Name: "send_goal_service", TypeID: idNested, NestedTypeName: a.SendGoal.FullName()},
		{Name: "get_result_service", TypeID: idNested, NestedTypeName: a.GetResult.FullName()},
		{Name: "feedback_message", TypeID: idNested, NestedTypeName: a.FeedbackMessage.FullName()},
	}}
	refs := map[string]Individual{}
	for _, m := range []*Message{a.Goal, a.Result, a.Feedback, a.FeedbackMessage} {
		refs[m.FullName()] = individualOf(m)
		if err := r.collectRefs(m, refs); err != nil {
			return TypeDescription{}, err
		}
	}
	for _, svc := range []*Service{a.SendGoal, a.GetResult} {
		if err := r.addServiceRefs(svc, refs); err != nil {
			return TypeDescription{}, err
		}
	}
	return assemble(ind, refs), nil
}

func assemble(ind Individual, refs map[string]Individual) TypeDescription {
	td := TypeDescription{Individual: ind}
	names := make([]string, 0, len(refs))
	for name := range refs {
		names = append(names, name)
	}
	sort.Strings(names)
	for _, name := range names {
		td.Referenced = append(td.Referenced, refs[name])
	}
	return td
}

func (r *Resolver) collectRefs(m *Message, refs map[string]Individual) error {
	for _, f := range m.Fields {
		name := f.Type.Nested
		if name == "" {
			continue
		}
		if _, done := refs[name]; done {
			continue
		}
		ref, err := r.Resolve(name)
		if err != nil {
			return fmt.Errorf("resolving %s (referenced by %s): %w", name, m.FullName(), err)
		}
		refs[name] = individualOf(ref)
		if err := r.collectRefs(ref, refs); err != nil {
			return err
		}
	}
	return nil
}

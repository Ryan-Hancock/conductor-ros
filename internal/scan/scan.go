// Package scan discovers conductor nodes, message types, and external
// interfaces by parsing Go source. It is purely syntactic — no type checking
// or build — so it works on any tree under a go.mod: it looks for
// //conductor:node directives on struct types, conductor.Sub/Pub/Param/Timer
// fields, //ros:type directives on message structs, and an optional
// conductor.json declaring external ROS nodes' topics.
package scan

import (
	"encoding/json"
	"fmt"
	"go/ast"
	"go/parser"
	"go/token"
	"go/types"
	"io/fs"
	"os"
	"path/filepath"
	"reflect"
	"sort"
	"strconv"
	"strings"
	"unicode"

	"conductor.dev/conductor"
)

type App struct {
	Name       string
	Dir        string
	ModuleRoot string
	Nodes      []*Node
	// Messages maps a Go type reference as written in source (package-qualified,
	// e.g. "msgs.Twist") to its ROS interface name ("geometry_msgs/msg/Twist"),
	// collected from //ros:type directives across the module.
	Messages map[string]string
	// Stamped records which of those types carry a std_msgs/Header, which is
	// what a frame tag needs to stamp.
	Stamped   map[string]bool
	Externals []External

	// Frames is the declared transform tree (frames.json, or whatever the
	// resolved environment names instead); nil when the app declares none.
	Frames     *conductor.FrameTree
	FramesFile string

	// Environments are declared in environments.json (see environments.go).
	// Env is the one this App has been resolved for, nil until Resolve.
	Environments map[string]*Environment
	EnvNames     []string
	DefaultEnv   string
	Env          *Environment
	// Robot is the machine this App has been resolved for, when the
	// environment declares a fleet; nil otherwise.
	Robot *Robot
}

type Node struct {
	StructName    string
	Name          string // snake_case node name
	File          string
	Line          int
	Subs          []Endpoint
	Pubs          []Endpoint
	Services      []ServiceEndpoint // servers (Svc fields)
	Clients       []ServiceEndpoint // clients (Client fields)
	Actions       []ActionEndpoint  // action servers (Action fields)
	ActionClients []ActionEndpoint  // action clients (ActionClient fields)
	Params        []Param
	Timers        []Timer
	Missions      []Mission // at most one; more than one is a validation error
	Steps         []Step
	TF            *TFDecl
	// Methods maps method name to its signature shape for handler checks.
	Methods map[string]MethodSig
	// Calls are conductor calls made with string literals inside this node's
	// methods — Task.Goto("recharge"), TF.Lookup("base_link", "laser") — so
	// the checker can hold code to the same declarations the tags are held to.
	Calls []Call
}

// Mission declares a node's task state machine (a conductor.Mission field).
type Mission struct {
	Field string
	Name  string // snake_case field name
	Start string
	File  string
	Line  int
}

// Step is one state of a node's mission (a conductor.Step field).
type Step struct {
	Field   string
	Name    string // snake_case field name: what next/fail/Goto refer to
	Next    string
	Fail    string
	Timeout string
	Retry   string
	Backoff string
	File    string
	Line    int
}

// Call is a call to a conductor method with string-literal arguments.
type Call struct {
	Method string   // "Goto", "Lookup"
	Recv   string   // receiver expression as written, e.g. "t" or "p.TF"
	In     string   // enclosing method name
	Args   []string // literal arguments
	File   string
	Line   int
}

// TFDecl is a node's conductor.TF field: its access to the declared frames.
type TFDecl struct {
	Field string
	File  string
	Line  int
}

type MethodSig struct {
	Params  int
	Results int
}

type ServiceEndpoint struct {
	Field   string
	Service string
	ReqType string // Go type reference as written, package-qualified
	ResType string
	File    string
	Line    int
}

type ActionEndpoint struct {
	Field        string
	Action       string
	GoalType     string
	FeedbackType string
	ResultType   string
	File         string
	Line         int
}

type Endpoint struct {
	Field  string
	Topic  string
	QoS    string
	Frame  string // frame tag: the TF frame this endpoint's messages are in
	GoType string
	File   string
	Line   int
}

type Param struct {
	Field   string
	Name    string
	Default string
	GoType  string
}

type Timer struct {
	Field string
	Rate  string
	File  string
	Line  int
}

// External declares a topic or service endpoint provided by a node outside
// this application (an existing ROS 2 node), so graph validation can account
// for it. Roles publisher/subscriber declare a topic endpoint; server/client
// declare a service endpoint (Topic then names the service).
type External struct {
	Topic string `json:"topic"`
	Type  string `json:"type"` // ROS interface name
	Role  string `json:"role"` // "publisher", "subscriber", "server", or "client"
	// QoS is omitted when empty so that generated blocks (conductor
	// externals) do not write `"qos": ""` onto services and actions, which
	// have no profile.
	QoS string `json:"qos,omitempty"`
}

type config struct {
	App       string     `json:"app"`
	Externals []External `json:"externals"`
}

// ScanApp scans the application rooted at dir. Message types are collected
// from the entire module containing dir; nodes are collected from dir itself.
func ScanApp(dir string) (*App, error) {
	abs, err := filepath.Abs(dir)
	if err != nil {
		return nil, err
	}
	if fi, err := os.Stat(abs); err != nil || !fi.IsDir() {
		return nil, fmt.Errorf("not a directory: %s", dir)
	}
	root, err := findModuleRoot(abs)
	if err != nil {
		return nil, err
	}
	app := &App{Dir: abs, ModuleRoot: root, Messages: map[string]string{}, Stamped: map[string]bool{}}

	if b, err := os.ReadFile(filepath.Join(abs, "conductor.json")); err == nil {
		var c config
		if err := json.Unmarshal(b, &c); err != nil {
			return nil, fmt.Errorf("conductor.json: %w", err)
		}
		app.Name = c.App
		app.Externals = c.Externals
	}
	if app.Name == "" {
		app.Name = filepath.Base(abs)
	}
	if err := loadEnvironments(abs, app); err != nil {
		return nil, err
	}
	if err := loadFrames(abs, "frames.json", app); err != nil {
		return nil, err
	}

	err = walkGoFiles(root, func(path string, f *ast.File, fset *token.FileSet) {
		collectMessages(f, app.Messages, app.Stamped)
	})
	if err != nil {
		return nil, err
	}

	var nodes []*Node
	methodsByType := map[string]map[string]MethodSig{}
	callsByType := map[string][]Call{}
	err = walkGoFiles(abs, func(path string, f *ast.File, fset *token.FileSet) {
		collectMessages(f, app.Messages, app.Stamped)
		collectNodes(path, f, fset, &nodes)
		collectMethods(path, f, fset, methodsByType, callsByType)
	})
	if err != nil {
		return nil, err
	}
	for _, n := range nodes {
		n.Methods = methodsByType[n.StructName]
		if n.Methods == nil {
			n.Methods = map[string]MethodSig{}
		}
		n.Calls = callsByType[n.StructName]
	}
	sort.Slice(nodes, func(i, j int) bool { return nodes[i].Name < nodes[j].Name })
	app.Nodes = nodes
	return app, nil
}

var skipDirs = map[string]bool{"gen": true, "testdata": true, "vendor": true, "node_modules": true}

func walkGoFiles(root string, visit func(path string, f *ast.File, fset *token.FileSet)) error {
	fset := token.NewFileSet()
	return filepath.WalkDir(root, func(path string, d fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if d.IsDir() {
			if path != root && (skipDirs[d.Name()] || strings.HasPrefix(d.Name(), ".")) {
				return filepath.SkipDir
			}
			return nil
		}
		if !strings.HasSuffix(path, ".go") || strings.HasSuffix(path, "_test.go") {
			return nil
		}
		f, perr := parser.ParseFile(fset, path, nil, parser.ParseComments|parser.SkipObjectResolution)
		if perr != nil {
			return perr
		}
		visit(path, f, fset)
		return nil
	})
}

func findModuleRoot(dir string) (string, error) {
	for d := dir; ; {
		if _, err := os.Stat(filepath.Join(d, "go.mod")); err == nil {
			return d, nil
		}
		parent := filepath.Dir(d)
		if parent == d {
			return "", fmt.Errorf("no go.mod found above %s", dir)
		}
		d = parent
	}
}

func hasDirective(doc *ast.CommentGroup, directive string) bool {
	if doc == nil {
		return false
	}
	for _, c := range doc.List {
		if strings.TrimSpace(c.Text) == directive {
			return true
		}
	}
	return false
}

func directiveValue(doc *ast.CommentGroup, prefix string) string {
	if doc == nil {
		return ""
	}
	for _, c := range doc.List {
		t := strings.TrimSpace(c.Text)
		if strings.HasPrefix(t, prefix+" ") {
			return strings.TrimSpace(strings.TrimPrefix(t, prefix+" "))
		}
	}
	return ""
}

func collectMessages(f *ast.File, out map[string]string, stamped map[string]bool) {
	pkg := f.Name.Name
	for _, decl := range f.Decls {
		gd, ok := decl.(*ast.GenDecl)
		if !ok || gd.Tok != token.TYPE {
			continue
		}
		for _, spec := range gd.Specs {
			ts, ok := spec.(*ast.TypeSpec)
			if !ok {
				continue
			}
			rosName := directiveValue(ts.Doc, "//ros:type")
			if rosName == "" && len(gd.Specs) == 1 {
				rosName = directiveValue(gd.Doc, "//ros:type")
			}
			if rosName != "" {
				out[pkg+"."+ts.Name.Name] = rosName
				stamped[pkg+"."+ts.Name.Name] = hasHeaderField(ts.Type)
			}
		}
	}
}

// hasHeaderField reports whether a message struct carries a Header, which is
// what a frame tag needs somewhere to put the frame id.
func hasHeaderField(e ast.Expr) bool {
	st, ok := e.(*ast.StructType)
	if !ok {
		return false
	}
	for _, f := range st.Fields.List {
		for _, name := range f.Names {
			if name.Name == "Header" {
				return true
			}
		}
	}
	return false
}

func collectNodes(path string, f *ast.File, fset *token.FileSet, out *[]*Node) {
	pkg := f.Name.Name
	for _, decl := range f.Decls {
		gd, ok := decl.(*ast.GenDecl)
		if !ok || gd.Tok != token.TYPE {
			continue
		}
		for _, spec := range gd.Specs {
			ts, ok := spec.(*ast.TypeSpec)
			if !ok {
				continue
			}
			if !hasDirective(gd.Doc, "//conductor:node") && !hasDirective(ts.Doc, "//conductor:node") {
				continue
			}
			st, ok := ts.Type.(*ast.StructType)
			if !ok {
				continue
			}
			n := &Node{
				StructName: ts.Name.Name,
				Name:       SnakeCase(ts.Name.Name),
				File:       path,
				Line:       fset.Position(ts.Pos()).Line,
			}
			for _, field := range st.Fields.List {
				addField(n, pkg, field, path, fset)
			}
			*out = append(*out, n)
		}
	}
}

func addField(n *Node, pkg string, field *ast.Field, path string, fset *token.FileSet) {
	if len(field.Names) == 0 {
		return
	}
	fname := field.Names[0].Name
	line := fset.Position(field.Pos()).Line
	tag := parseTag(field.Tag)

	kind, args := conductorField(field.Type)
	switch kind {
	case "Sub", "Pub":
		ep := Endpoint{
			Field:  fname,
			Topic:  tag.Get("topic"),
			QoS:    tag.Get("qos"),
			Frame:  tag.Get("frame"),
			GoType: qualify(pkg, args[0]),
			File:   path,
			Line:   line,
		}
		if kind == "Sub" {
			n.Subs = append(n.Subs, ep)
		} else {
			n.Pubs = append(n.Pubs, ep)
		}
	case "Svc", "Client":
		ep := ServiceEndpoint{
			Field:   fname,
			Service: tag.Get("service"),
			ReqType: qualify(pkg, args[0]),
			ResType: qualify(pkg, args[1]),
			File:    path,
			Line:    line,
		}
		if kind == "Svc" {
			n.Services = append(n.Services, ep)
		} else {
			n.Clients = append(n.Clients, ep)
		}
	case "Action", "ActionClient":
		ep := ActionEndpoint{
			Field:        fname,
			Action:       tag.Get("action"),
			GoalType:     qualify(pkg, args[0]),
			FeedbackType: qualify(pkg, args[1]),
			ResultType:   qualify(pkg, args[2]),
			File:         path,
			Line:         line,
		}
		if kind == "Action" {
			n.Actions = append(n.Actions, ep)
		} else {
			n.ActionClients = append(n.ActionClients, ep)
		}
	case "Param":
		name := tag.Get("name")
		if name == "" {
			name = SnakeCase(fname)
		}
		n.Params = append(n.Params, Param{Field: fname, Name: name, Default: tag.Get("default"), GoType: args[0]})
	case "Timer":
		n.Timers = append(n.Timers, Timer{Field: fname, Rate: tag.Get("rate"), File: path, Line: line})
	case "Mission":
		n.Missions = append(n.Missions, Mission{
			Field: fname, Name: SnakeCase(fname), Start: tag.Get("start"), File: path, Line: line,
		})
	case "Step":
		n.Steps = append(n.Steps, Step{
			Field:   fname,
			Name:    SnakeCase(fname),
			Next:    tag.Get("next"),
			Fail:    tag.Get("fail"),
			Timeout: tag.Get("timeout"),
			Retry:   tag.Get("retry"),
			Backoff: tag.Get("backoff"),
			File:    path,
			Line:    line,
		})
	case "TF":
		n.TF = &TFDecl{Field: fname, File: path, Line: line}
	}
}

// conductorField reports whether e is a conductor framework field type,
// returning its kind ("Sub", "Pub", "Svc", "Client", "Param", "Timer") and
// type arguments. Matching is syntactic on the "conductor" package qualifier.
func conductorField(e ast.Expr) (kind string, args []string) {
	switch x := e.(type) {
	case *ast.IndexExpr: // one type parameter: Sub[T], Pub[T], Param[T]
		sel, ok := x.X.(*ast.SelectorExpr)
		if !ok {
			return "", nil
		}
		id, ok := sel.X.(*ast.Ident)
		if !ok || id.Name != "conductor" {
			return "", nil
		}
		switch sel.Sel.Name {
		case "Sub", "Pub", "Param":
			return sel.Sel.Name, []string{types.ExprString(x.Index)}
		}
	case *ast.IndexListExpr: // Svc[Req, Res], Client[Req, Res], Action[G, F, R]
		sel, ok := x.X.(*ast.SelectorExpr)
		if !ok {
			return "", nil
		}
		id, ok := sel.X.(*ast.Ident)
		if !ok || id.Name != "conductor" {
			return "", nil
		}
		switch {
		case len(x.Indices) == 2 && (sel.Sel.Name == "Svc" || sel.Sel.Name == "Client"):
			return sel.Sel.Name, []string{types.ExprString(x.Indices[0]), types.ExprString(x.Indices[1])}
		case len(x.Indices) == 3 && (sel.Sel.Name == "Action" || sel.Sel.Name == "ActionClient"):
			return sel.Sel.Name, []string{types.ExprString(x.Indices[0]), types.ExprString(x.Indices[1]), types.ExprString(x.Indices[2])}
		}
	case *ast.SelectorExpr: // no type parameters: Timer, Mission, Step, TF
		id, ok := x.X.(*ast.Ident)
		if !ok || id.Name != "conductor" {
			return "", nil
		}
		switch x.Sel.Name {
		case "Timer", "Mission", "Step", "TF":
			return x.Sel.Name, []string{""}
		}
	}
	return "", nil
}

// qualify makes a message type reference package-qualified so it matches the
// keys produced by collectMessages: bare identifiers refer to the node's own
// package.
func qualify(pkg, typ string) string {
	if typ == "" || strings.Contains(typ, ".") {
		return typ
	}
	return pkg + "." + typ
}

func parseTag(lit *ast.BasicLit) reflect.StructTag {
	if lit == nil {
		return ""
	}
	return reflect.StructTag(strings.Trim(lit.Value, "`"))
}

func collectMethods(path string, f *ast.File, fset *token.FileSet, out map[string]map[string]MethodSig, calls map[string][]Call) {
	for _, decl := range f.Decls {
		fd, ok := decl.(*ast.FuncDecl)
		if !ok || fd.Recv == nil || len(fd.Recv.List) == 0 {
			continue
		}
		recv := receiverTypeName(fd.Recv.List[0].Type)
		if recv == "" {
			continue
		}
		if out[recv] == nil {
			out[recv] = map[string]MethodSig{}
		}
		out[recv][fd.Name.Name] = MethodSig{
			Params:  fd.Type.Params.NumFields(),
			Results: fd.Type.Results.NumFields(),
		}
		calls[recv] = append(calls[recv], literalCalls(path, fd, fset)...)
	}
}

// literalCalls finds the conductor calls in a method body whose arguments are
// all string literals — Goto("recharge"), Lookup("base_link", "laser") — so
// the checker can resolve them against the declarations. Anything computed is
// simply not checked, which is the honest limit of a syntactic scanner.
func literalCalls(path string, fd *ast.FuncDecl, fset *token.FileSet) []Call {
	var out []Call
	ast.Inspect(fd.Body, func(n ast.Node) bool {
		call, ok := n.(*ast.CallExpr)
		if !ok {
			return true
		}
		sel, ok := call.Fun.(*ast.SelectorExpr)
		if !ok {
			return true
		}
		switch sel.Sel.Name {
		case "Goto", "Lookup":
		default:
			return true
		}
		args := make([]string, 0, len(call.Args))
		for _, a := range call.Args {
			lit, ok := a.(*ast.BasicLit)
			if !ok || lit.Kind != token.STRING {
				return true
			}
			s, err := strconv.Unquote(lit.Value)
			if err != nil {
				return true
			}
			args = append(args, s)
		}
		out = append(out, Call{
			Method: sel.Sel.Name,
			Recv:   types.ExprString(sel.X),
			In:     fd.Name.Name,
			Args:   args,
			File:   path,
			Line:   fset.Position(call.Pos()).Line,
		})
		return true
	})
	return out
}

func receiverTypeName(e ast.Expr) string {
	if se, ok := e.(*ast.StarExpr); ok {
		e = se.X
	}
	if id, ok := e.(*ast.Ident); ok {
		return id.Name
	}
	return ""
}

// SnakeCase converts a Go struct name to its node name (Navigator ->
// "navigator", SafetyMonitor -> "safety_monitor"); it mirrors the runtime's
// naming so static and runtime views agree.
func SnakeCase(s string) string {
	var b strings.Builder
	for i, r := range s {
		if unicode.IsUpper(r) {
			if i > 0 {
				b.WriteByte('_')
			}
			b.WriteRune(unicode.ToLower(r))
		} else {
			b.WriteRune(r)
		}
	}
	return b.String()
}

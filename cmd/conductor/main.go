// Command conductor is the Conductor CLI: it scans an application for
// //conductor:node declarations, validates the resulting topic graph, and
// generates launch and parameter artifacts.
//
// Usage:
//
//	conductor check [dir]   scan and validate the application graph
//	conductor graph [dir]   print the topic graph as Graphviz dot
//	conductor build [dir]   check, compile the app, and write gen/ artifacts
//	conductor test [dir]    check the graph, then run the app's Go tests
package main

import (
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"sort"
	"strings"

	"conductor.dev/conductor/internal/gen"
	"conductor.dev/conductor/internal/graph"
	"conductor.dev/conductor/internal/scan"
)

func main() {
	if len(os.Args) < 2 {
		usage()
		os.Exit(2)
	}
	dir := "."
	if len(os.Args) > 2 {
		dir = os.Args[2]
	}
	var err error
	switch os.Args[1] {
	case "check":
		err = runCheck(dir)
	case "graph":
		err = runGraph(dir)
	case "build":
		err = runBuild(dir)
	case "test":
		err = runTest(os.Args[2:])
	case "msggen":
		err = runMsggen(os.Args[2:])
	default:
		usage()
		os.Exit(2)
	}
	if err != nil {
		fmt.Fprintln(os.Stderr, "conductor:", err)
		os.Exit(1)
	}
}

func usage() {
	fmt.Fprint(os.Stderr, `usage:
  conductor check [dir]   scan and validate the application graph
  conductor graph [dir]   print the topic graph as Graphviz dot
  conductor build [dir]   check, compile the app, and write gen/ artifacts
  conductor test [dir] [go test flags...]
                          validate the graph, then run the application's Go
                          tests (see the conductortest package)
  conductor msggen -out <dir> [-pkg <gopkg>] [-ros-pkg <pkg>] <target...>
                          generate Go message types (with computed RIHS01
                          hashes) from .msg definitions; targets are ROS
                          interface names (geometry_msgs/msg/Twist, resolved
                          via $AMENT_PREFIX_PATH) or local .msg files/dirs
                          (which require -ros-pkg)
`)
}

func runCheck(dir string) error {
	_, _, err := check(dir, true)
	return err
}

func check(dir string, report bool) (*scan.App, *graph.Graph, error) {
	app, err := scan.ScanApp(dir)
	if err != nil {
		return nil, nil, err
	}
	g, issues := graph.Validate(app)
	if report {
		printReport(app, g, issues)
	}
	if graph.Errors(issues) {
		return app, g, fmt.Errorf("graph has errors")
	}
	return app, g, nil
}

func printReport(app *scan.App, g *graph.Graph, issues []graph.Issue) {
	fmt.Printf("app %s — %d node(s), %d topic(s), %d external interface(s)\n\n",
		app.Name, len(app.Nodes), len(g.Topics), len(app.Externals))

	fmt.Println("nodes:")
	for _, n := range app.Nodes {
		fmt.Printf("  %-16s %s (%s)\n", n.Name, n.StructName, relPos(app, fmt.Sprintf("%s:%d", n.File, n.Line)))
		for _, tm := range n.Timers {
			fmt.Printf("    tick  every %s\n", tm.Rate)
		}
		for _, s := range n.Subs {
			fmt.Printf("    sub   %-14s %-32s [%s]\n", s.Topic, rosOrGo(app, s.GoType), qosName(s.QoS))
		}
		for _, p := range n.Pubs {
			fmt.Printf("    pub   %-14s %-32s [%s]\n", p.Topic, rosOrGo(app, p.GoType), qosName(p.QoS))
		}
		for _, s := range n.Services {
			fmt.Printf("    svc   %-14s (%s) -> (%s)\n", s.Service, s.ReqType, s.ResType)
		}
		for _, c := range n.Clients {
			fmt.Printf("    call  %-14s (%s) -> (%s)\n", c.Service, c.ReqType, c.ResType)
		}
		for _, a := range n.Actions {
			fmt.Printf("    act   %-14s goal %s, feedback %s, result %s\n", a.Action, a.GoalType, a.FeedbackType, a.ResultType)
		}
		for _, a := range n.ActionClients {
			fmt.Printf("    send  %-14s goal %s, feedback %s, result %s\n", a.Action, a.GoalType, a.FeedbackType, a.ResultType)
		}
		for _, p := range n.Params {
			def := p.Default
			if def == "" {
				def = "(no default)"
			}
			fmt.Printf("    param %s %s = %s\n", p.Name, p.GoType, def)
		}
	}

	fmt.Println("\ntopics:")
	for _, t := range g.Topics {
		fmt.Printf("  %-14s %-32s %s -> %s\n", t.Name, orDash(t.RosType()), endpointNames(t.Pubs), endpointNames(t.Subs))
	}

	if len(g.Services) > 0 {
		fmt.Println("\nservices:")
		for _, s := range g.Services {
			fmt.Printf("  %-14s %-32s %s <- %s\n", s.Name, orDash(svcRosType(s)), svcNames(s.Servers), svcNames(s.Clients))
		}
	}

	if len(g.Actions) > 0 {
		fmt.Println("\nactions:")
		for _, a := range g.Actions {
			rosType := ""
			for _, ep := range append(append([]graph.SvcEndpoint{}, a.Servers...), a.Clients...) {
				if ep.RosType != "" {
					rosType = ep.RosType
					break
				}
			}
			fmt.Printf("  %-14s %-32s %s <- %s\n", a.Name, orDash(rosType), svcNames(a.Servers), svcNames(a.Clients))
		}
	}

	order, cycles := g.BringupOrder()
	if len(order) > 1 {
		fmt.Printf("\nbringup order (derived from the graph):\n  %s\n", strings.Join(order, " -> "))
		if len(cycles) > 0 {
			fmt.Printf("  note: %s form a dependency cycle and start in declaration order\n", strings.Join(cycles, ", "))
		}
	}

	nErr, nWarn := 0, 0
	if len(issues) > 0 {
		fmt.Println("\nissues:")
		for _, i := range issues {
			if i.Severity == graph.Error {
				nErr++
			} else {
				nWarn++
			}
			pos := ""
			if i.Pos != "" {
				pos = " (" + relPos(app, i.Pos) + ")"
			}
			fmt.Printf("  %-7s %s: %s%s\n", i.Severity, i.Code, i.Msg, pos)
		}
	}
	if nErr == 0 {
		fmt.Printf("\n✓ graph valid: 0 errors, %d warning(s)\n", nWarn)
	} else {
		fmt.Printf("\n✗ graph invalid: %d error(s), %d warning(s)\n", nErr, nWarn)
	}
}

func runGraph(dir string) error {
	_, g, err := check(dir, false)
	if err != nil {
		return err
	}
	fmt.Print(gen.Dot(g))
	return nil
}

func runBuild(dir string) error {
	app, g, err := check(dir, true)
	if err != nil {
		return err
	}

	rel, err := filepath.Rel(app.ModuleRoot, app.Dir)
	if err != nil {
		return err
	}
	binPath := filepath.Join("gen", "bin", app.Name)
	binAbs := filepath.Join(app.Dir, binPath)
	if err := os.MkdirAll(filepath.Dir(binAbs), 0o755); err != nil {
		return err
	}
	cmd := exec.Command("go", "build", "-o", binAbs, "./"+filepath.ToSlash(rel))
	cmd.Dir = app.ModuleRoot
	cmd.Stdout, cmd.Stderr = os.Stdout, os.Stderr
	if err := cmd.Run(); err != nil {
		return fmt.Errorf("go build: %w", err)
	}

	written, err := gen.WriteAll(g, binPath)
	if err != nil {
		return err
	}
	written = append(written, binAbs)
	sort.Strings(written)
	fmt.Println("\nwrote:")
	for _, w := range written {
		fmt.Println("  " + relPos(app, w))
	}
	return nil
}

// runTest validates the graph before running the tests: a wiring mistake
// makes every behavioural test fail for the same uninformative reason, so it
// is worth reporting first, in its own vocabulary.
func runTest(args []string) error {
	dir := "."
	if len(args) > 0 && !strings.HasPrefix(args[0], "-") {
		dir, args = args[0], args[1:]
	}
	app, _, err := check(dir, true)
	if err != nil {
		return err
	}
	rel, err := filepath.Rel(app.ModuleRoot, app.Dir)
	if err != nil {
		return err
	}
	pattern := "./..."
	if rel != "." {
		pattern = "./" + filepath.ToSlash(rel) + "/..."
	}
	fmt.Println()
	cmd := exec.Command("go", append(append([]string{"test"}, args...), pattern)...)
	cmd.Dir = app.ModuleRoot
	cmd.Stdout, cmd.Stderr = os.Stdout, os.Stderr
	if err := cmd.Run(); err != nil {
		return fmt.Errorf("go test: %w", err)
	}
	return nil
}

func rosOrGo(app *scan.App, goType string) string {
	if r, ok := app.Messages[goType]; ok {
		return r
	}
	return goType
}

func qosName(tag string) string {
	if tag == "" {
		return "reliable"
	}
	return tag
}

func orDash(s string) string {
	if s == "" {
		return "-"
	}
	return s
}

func endpointNames(eps []graph.Endpoint) string {
	if len(eps) == 0 {
		return "(none)"
	}
	names := make([]string, len(eps))
	for i, e := range eps {
		names[i] = e.Node
	}
	return strings.Join(names, ",")
}

func svcNames(eps []graph.SvcEndpoint) string {
	if len(eps) == 0 {
		return "(none)"
	}
	names := make([]string, len(eps))
	for i, e := range eps {
		names[i] = e.Node
	}
	return strings.Join(names, ",")
}

func svcRosType(s *graph.Service) string {
	for _, ep := range append(append([]graph.SvcEndpoint{}, s.Servers...), s.Clients...) {
		if ep.RosType != "" {
			return ep.RosType
		}
	}
	return ""
}

func relPos(app *scan.App, pos string) string {
	if rel, err := filepath.Rel(app.ModuleRoot, pos); err == nil && !strings.HasPrefix(rel, "..") {
		return rel
	}
	return pos
}

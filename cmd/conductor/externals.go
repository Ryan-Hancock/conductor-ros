package main

import (
	"flag"
	"fmt"
	"os"
	"sort"
	"strings"
	"time"

	"conductor.dev/conductor/internal/discover"
	"conductor.dev/conductor/internal/scan"
)

// runExternals asks a running system what it offers, and compares that with
// what the application says about it.
//
// conductor.json's externals block is the one thing the checker takes entirely
// on trust, and every entry in it was transcribed by hand from someone else's
// source. The Nav2 example needed two details that are only discoverable that
// way — cmd_vel is TwistStamped now, amcl_pose is latched — and getting either
// wrong produces silence rather than an error. A live graph knows both.
func runExternals(args []string) error {
	dir, args := splitDir(args)
	fs := flag.NewFlagSet("externals", flag.ExitOnError)
	env := fs.String("env", "", "environment to compare against (see environments.json)")
	robot := fs.String("robot", "", "compare as one robot of the environment's fleet")
	endpoint := fs.String("endpoint", "", "zenoh router endpoint (default: the environment's)")
	domain := fs.Int("domain", -1, "ROS domain id (default: the environment's, else 0)")
	timeout := fs.Duration("timeout", 3*time.Second, "how long to let the graph answer")
	write := fs.Bool("write", false, "update conductor.json's externals block in place")
	checkOnly := fs.Bool("check", false, "exit non-zero if the graph disagrees with what is declared")
	all := fs.Bool("all", false, "include interfaces this application does not use")
	infra := fs.Bool("infra", false, "include parameter and lifecycle services")
	raw := fs.Bool("raw", false, "print the discovered graph instead of externals")
	if err := fs.Parse(args); err != nil {
		return err
	}

	app, err := resolveRobot(dir, *env, *robot)
	if err != nil {
		return err
	}
	g, err := discover.Query(endpointFor(app, *endpoint), domainFor(app, *domain), *timeout)
	if err != nil {
		return err
	}
	if *raw {
		printGraph(g)
		return nil
	}

	report := discover.Externals(app, g, discover.Options{All: *all, Infrastructure: *infra})
	printExternals(report)

	// -check is the form for a script: the declarations are meant to describe
	// the system, so a difference is a failure rather than a suggestion.
	if *checkOnly && report.Changed() {
		return fmt.Errorf("conductor.json does not match the live graph (see above); run with -write to update it")
	}
	if *write {
		if !report.Changed() {
			fmt.Println("\nconductor.json is already what the graph says; nothing written.")
			return nil
		}
		if err := discover.Write(app, report.Externals); err != nil {
			return err
		}
		fmt.Printf("\nwrote %d external(s) to %s/conductor.json — run `conductor check` next.\n",
			len(report.Externals), app.Dir)
	}
	return nil
}

// endpointFor prefers the flag, then the environment: the graph to read is
// normally the one this environment runs on.
func endpointFor(app *scan.App, flagValue string) string {
	if flagValue != "" {
		return flagValue
	}
	if app.Env != nil && app.Env.Endpoint != "" {
		return app.Env.Endpoint
	}
	return "tcp/127.0.0.1:7447"
}

func domainFor(app *scan.App, flagValue int) int {
	if flagValue >= 0 {
		return flagValue
	}
	if app.Env != nil && app.Env.Domain != nil {
		return *app.Env.Domain
	}
	return 0
}

// printGraph is the -raw view: every node and endpoint as discovered, which is
// what to look at when a derivation seems wrong.
func printGraph(g *discover.Graph) {
	nodes := g.Nodes()
	fmt.Printf("graph on domain %d via %s — %d node(s), %d endpoint(s)\n\n",
		g.Domain, g.Endpoint, len(nodes), len(g.Entities)-len(nodes))
	if len(nodes) > 0 {
		fmt.Printf("nodes:\n  %s\n\n", strings.Join(nodes, "\n  "))
	}

	interfaces := g.Interfaces()
	fmt.Println("interfaces:")
	for _, i := range interfaces {
		qos := ""
		if i.Kind == discover.KindTopic {
			qos = "  [" + orUnnamed(i.QoS) + "]"
		}
		infra := ""
		if i.Infrastructure {
			infra = "  (infrastructure)"
		}
		fmt.Printf("  %-7s %-42s %s%s%s\n", i.Kind, i.Name, i.Type, qos, infra)
		if len(i.Publishers) > 0 {
			fmt.Printf("            publishers:  %s\n", strings.Join(i.Publishers, ", "))
		}
		if len(i.Subscribers) > 0 {
			fmt.Printf("            subscribers: %s\n", strings.Join(i.Subscribers, ", "))
		}
		if len(i.Servers) > 0 {
			fmt.Printf("            servers:     %s\n", strings.Join(i.Servers, ", "))
		}
		if len(i.Clients) > 0 {
			fmt.Printf("            clients:     %s\n", strings.Join(i.Clients, ", "))
		}
	}
	printUnreadable(g)
}

func printExternals(r *discover.Report) {
	env := ""
	if r.Env != "" {
		env = fmt.Sprintf(" [env %s]", r.Env)
	}
	fmt.Printf("app %s%s — read %d node(s) from the graph on domain %d via %s\n",
		r.App, env, len(r.Graph.Nodes()), r.Graph.Domain, r.Graph.Endpoint)
	if len(r.Ours) > 0 {
		fmt.Printf("this application's own nodes on the graph (excluded): %s\n", strings.Join(r.Ours, ", "))
	}
	fmt.Println()

	if len(r.Findings) == 0 {
		fmt.Println("✓ every declared external matches the graph, and the graph adds nothing new.")
	} else {
		byKind := map[discover.FindingKind][]discover.Finding{}
		for _, f := range r.Findings {
			byKind[f.Kind] = append(byKind[f.Kind], f)
		}
		for _, kind := range []discover.FindingKind{
			discover.FindingMismatch, discover.FindingConflict,
			discover.FindingMissing, discover.FindingAbsent,
		} {
			for _, f := range byKind[kind] {
				where := f.Topic
				if f.Role != "" {
					where += " (" + f.Role + ")"
				}
				fmt.Printf("  %-9s %-40s %s\n", kind, where, f.Message)
			}
		}
	}

	fmt.Print("\nexternals as the graph describes them:\n\n")
	block, err := discover.Render(r.Externals)
	if err != nil {
		fmt.Fprintln(os.Stderr, "conductor:", err)
		return
	}
	fmt.Println(indent(string(block), "  "))
	printUnreadable(r.Graph)
}

func printUnreadable(g *discover.Graph) {
	if len(g.Unreadable) == 0 {
		return
	}
	// Sorted so the same graph reports the same way twice.
	sort.Slice(g.Unreadable, func(a, b int) bool { return g.Unreadable[a].Token < g.Unreadable[b].Token })
	fmt.Printf("\n%d liveliness token(s) this build could not read:\n", len(g.Unreadable))
	for _, u := range g.Unreadable {
		fmt.Printf("  %s\n    %s\n", u.Token, u.Err)
	}
}

func indent(s, with string) string {
	lines := strings.Split(strings.TrimRight(s, "\n"), "\n")
	for i, l := range lines {
		lines[i] = with + l
	}
	return strings.Join(lines, "\n")
}

func orUnnamed(qos string) string {
	if qos == "" {
		return "no conductor profile"
	}
	return qos
}

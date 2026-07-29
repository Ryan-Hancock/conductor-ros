package run

import (
	"fmt"
	"io"
	"net"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"strconv"
	"strings"
	"syscall"
	"time"

	"conductor.dev/conductor"
	"conductor.dev/conductor/internal/gen"
	"conductor.dev/conductor/internal/scan"
)

// A zenoh deployment is one process per node. Locally the in-process bus is
// the default, so every boundary that exists on the robot is absent on the
// desk — which is exactly where "works here, silent in the field" comes from.
// `-split` runs the layout the units run: one process per node, in bringup
// order, on the dashboard ports the deployment would assign, with the fleet
// view over them.

// defaultDashboardBase is where a development run serves its portal when the
// environment does not say. It binds loopback: a development default that
// opened a port to the network would be a poor one.
const defaultDashboardBase = "127.0.0.1:4000"

// defaultFleetAddr is where the aggregated view goes in a split run. It is
// well clear of the per-node ports so a large application does not collide
// with it.
const defaultFleetAddr = "127.0.0.1:4500"

// runSplit builds the application once and runs it once per node.
func (s *Session) runSplit() error {
	if transport(s.app) == "inproc" {
		return fmt.Errorf("-split needs a transport that leaves the process: %s runs on the "+
			"in-process bus, so its nodes would be started as %d applications that cannot hear each other "+
			"(this is the same rule `conductor deploy` applies when it makes one unit instead of many)",
			envName(s.app), len(s.graph.App.Nodes))
	}
	order, _ := s.graph.BringupOrder()
	if s.opts.Node != "" {
		order = []string{s.opts.Node}
	}
	if len(order) == 0 {
		return fmt.Errorf("no nodes to run")
	}

	bin, err := s.build()
	if err != nil {
		return err
	}

	base := s.dashboardBase()
	dep := gen.Deployment{App: s.app.Name, Dashboard: base, Metrics: metricsBase(s.app)}
	flags := Flags(s.app)

	fmt.Fprintf(s.out, "conductor: running %s%s as %d processes\n", s.app.Name, envSuffix(s.app), len(order))
	var peers []conductor.Peer
	for i, node := range order {
		args := append([]string{"-node", node}, withoutDashboard(flags)...)
		if addr := dep.DashboardAddr(i); addr != "" && base != "off" {
			// A development run records spans: the fleet view stitches
			// them into chains across these processes, which is the thing a
			// split layout makes possible and a single one cannot show.
			args = append(args, "-dashboard", addr, "-dashboard-traces", "300")
			peers = append(peers, conductor.Peer{Name: node, URL: "http://" + addr})
		}
		if addr := dep.MetricsAddr(i); addr != "" {
			args = append(args, "-metrics-addr", addr)
		}
		args = append(args, s.opts.Args...)

		if err := s.startNode(bin, node, args); err != nil {
			return err
		}
	}

	if len(peers) > 0 {
		s.serveFleet(peers)
	}
	return s.awaitNodes()
}

// build compiles the application once, the way a release does, instead of
// letting `go run` do it per process.
func (s *Session) build() (string, error) {
	rel, err := filepath.Rel(s.app.ModuleRoot, s.app.Dir)
	if err != nil {
		return "", err
	}
	bin := filepath.Join(os.TempDir(), fmt.Sprintf("conductor-%s-%d", s.app.Name, os.Getpid()))
	args := []string{"build", "-o", bin}
	if tags := buildTags(s.app); tags != "" {
		args = append(args, "-tags", tags)
	}
	args = append(args, "./"+filepath.ToSlash(rel))

	fmt.Fprintf(s.out, "conductor: go %s\n", strings.Join(args, " "))
	cmd := exec.Command("go", args...)
	cmd.Dir = s.app.ModuleRoot
	cmd.Stdout, cmd.Stderr = s.out, os.Stderr
	if err := cmd.Run(); err != nil {
		return "", fmt.Errorf("go build: %w", err)
	}
	s.binary = bin
	return bin, nil
}

// startNode runs one node, with its output labelled so eight interleaved
// processes stay readable.
func (s *Session) startNode(bin, node string, args []string) error {
	cmd := exec.Command(bin, args...)
	cmd.Dir = s.app.Dir
	cmd.SysProcAttr = &syscall.SysProcAttr{Setpgid: true, Pdeathsig: syscall.SIGTERM}
	p := &process{spec: scan.Process{Name: node}, cmd: cmd, log: newTail(40), done: make(chan struct{})}
	sink := io.MultiWriter(p.log, prefixed(s.out, node))
	cmd.Stdout, cmd.Stderr = sink, sink

	if err := cmd.Start(); err != nil {
		return fmt.Errorf("starting %s: %w", node, err)
	}
	s.started = append(s.started, p)
	s.nodes = append(s.nodes, p)
	go func() { p.err = cmd.Wait(); close(p.done) }()
	return nil
}

// serveFleet puts the aggregated view over the processes just started —
// which is the view that matches this layout, since each process can only
// honestly report its own quarter of the graph.
func (s *Session) serveFleet(peers []conductor.Peer) {
	addr := s.fleetAddr()
	srv, err := conductor.ServeFleet(addr, peers, 2*time.Second, conductor.FleetOptions{
		External: s.app.ExternalTopics(),
		Traces:   2000,
	})
	if err != nil {
		fmt.Fprintf(s.out, "conductor: fleet view unavailable: %v\n", err)
		return
	}
	s.fleet = srv
	url := "http://" + addr + "/"
	fmt.Fprintf(s.out, "conductor: fleet view on %s\n", url)
	s.openBrowser(url)
}

// awaitNodes waits for the first node to exit, or for an interrupt. A node
// that dies takes the run with it: a graph missing a quarter of itself is not
// something to keep running quietly.
func (s *Session) awaitNodes() error {
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM, syscall.SIGHUP)
	defer signal.Stop(sig)

	cases := make(chan *process, len(s.nodes))
	for _, p := range s.nodes {
		go func(p *process) { <-p.done; cases <- p }(p)
	}

	select {
	case <-sig:
		fmt.Fprintf(s.out, "\nconductor: stopping\n")
		s.signalNodes(syscall.SIGINT)
		s.waitNodes(5 * time.Second)
		return nil
	case p := <-cases:
		if p.err != nil {
			fmt.Fprintf(s.out, "\nconductor: %s exited: %v\n", p.spec.Name, p.err)
		} else {
			fmt.Fprintf(s.out, "\nconductor: %s finished\n", p.spec.Name)
		}
		s.signalNodes(syscall.SIGINT)
		s.waitNodes(5 * time.Second)
		return p.err
	}
}

func (s *Session) signalNodes(sig syscall.Signal) {
	for _, p := range s.nodes {
		if p.cmd.Process != nil {
			syscall.Kill(-p.cmd.Process.Pid, sig)
		}
	}
}

func (s *Session) waitNodes(grace time.Duration) {
	deadline := time.After(grace)
	for _, p := range s.nodes {
		select {
		case <-p.done:
		case <-deadline:
			s.signalNodes(syscall.SIGKILL)
			return
		}
	}
}

// dashboardBase is where the per-node dashboards start, before the offset
// each node's position adds.
func (s *Session) dashboardBase() string {
	switch {
	case s.opts.Dashboard != "":
		return s.opts.Dashboard
	case s.app.Env != nil && s.app.Env.Dashboard != "":
		return s.app.Env.Dashboard
	}
	return defaultDashboardBase
}

func (s *Session) fleetAddr() string {
	// Keep the fleet view clear of the per-node ports.
	host, port, err := net.SplitHostPort(s.dashboardBase())
	if err != nil {
		return defaultFleetAddr
	}
	n, err := strconv.Atoi(port)
	if err != nil {
		return defaultFleetAddr
	}
	if host == "" {
		host = "127.0.0.1"
	}
	return net.JoinHostPort(host, strconv.Itoa(n+500))
}

func metricsBase(app *scan.App) string {
	if app.Env == nil {
		return ""
	}
	return app.Env.Metrics
}

func transport(app *scan.App) string {
	if app.Env == nil || app.Env.Transport == "" {
		return "inproc"
	}
	return app.Env.Transport
}

func envName(app *scan.App) string {
	if app.Env == nil {
		return "this application"
	}
	return "environment " + app.Env.Name()
}

// withoutDashboard drops a dashboard flag the environment implied, because a
// split run assigns one port per node instead.
func withoutDashboard(flags []string) []string {
	out := make([]string, 0, len(flags))
	for i := 0; i < len(flags); i++ {
		if flags[i] == "-dashboard" || flags[i] == "-metrics-addr" {
			i++
			continue
		}
		out = append(out, flags[i])
	}
	return out
}

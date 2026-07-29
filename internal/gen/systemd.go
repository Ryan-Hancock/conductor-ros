package gen

import (
	"fmt"
	"net"
	"path"
	"sort"
	"strconv"
	"strings"

	"conductor.dev/conductor/internal/graph"
)

// Deployment describes where a release is installed and how its processes are
// started. It is the systemd projection of the same graph the launch file and
// the runtime are built from.
type Deployment struct {
	App     string
	Env     string // environment name, empty if the app declares none
	Version string
	Prefix  string // install root, e.g. /opt/conductor
	Scope   string // "system" or "user"
	Flags   []string
	Environ map[string]string

	// Metrics and Dashboard are the environment's metrics and dashboard
	// addresses. They are kept apart from Flags because a per-node
	// deployment is several processes: they cannot share one port, so each
	// unit gets the base port plus its position in the bringup order.
	Metrics   string
	Dashboard string

	// SingleProcess runs the whole application as one unit instead of one
	// per node. It is required by the in-process transport, whose bus does
	// not leave the process, and it is a reasonable choice generally: one
	// binary, one unit, one journal.
	SingleProcess bool
}

// AppDir, ReleaseDir and CurrentDir are the layout every release shares:
//
//	<prefix>/<app>/releases/<version>/   this release
//	<prefix>/<app>/current -> releases/<version>
//
// Units reference `current`, so a rollback is a symlink swap and a restart.
func (d Deployment) AppDir() string     { return path.Join(d.Prefix, d.App) }
func (d Deployment) ReleaseDir() string { return path.Join(d.AppDir(), "releases", d.Version) }
func (d Deployment) CurrentDir() string { return path.Join(d.AppDir(), "current") }

// UnitName is the systemd unit for one node, and TargetName the unit that
// groups them: `systemctl restart patrol.target` restarts the application
// whether it is one unit or one per node.
func (d Deployment) UnitName(node string) string { return fmt.Sprintf("%s-%s.service", d.App, node) }
func (d Deployment) TargetName() string          { return d.App + ".target" }

// SystemdUnits renders one service unit per node plus the application target,
// keyed by file name.
//
// The ordering in the units is not hand-written: After= comes from the same
// dependency edges as the bringup order, so a node starts after the nodes it
// subscribes to or calls. Edges that would close a cycle are dropped (systemd
// breaks ordering cycles arbitrarily, and the runtime's lifecycle already
// copes with peers that are not up yet).
func SystemdUnits(g *graph.Graph, d Deployment) map[string]string {
	order, cycles := g.BringupOrder()
	if d.SingleProcess {
		// One process runs every node; the runtime brings them up in the
		// same order the units would have encoded.
		name := d.App + ".service"
		return map[string]string{
			name:           d.serviceUnit("", nil, false, 0),
			d.TargetName(): d.targetUnit([]string{name}, order),
		}
	}
	rank := map[string]int{}
	for i, n := range order {
		rank[n] = i
	}
	deps := g.Dependencies()
	inCycle := map[string]bool{}
	for _, n := range cycles {
		inCycle[n] = true
	}

	units := map[string]string{}
	var services []string
	for i, node := range order {
		var after []string
		for _, dep := range deps[node] {
			if rank[dep] < rank[node] {
				after = append(after, d.UnitName(dep))
			}
		}
		sort.Strings(after)
		units[d.UnitName(node)] = d.serviceUnit(node, after, inCycle[node], i)
		services = append(services, d.UnitName(node))
	}
	units[d.TargetName()] = d.targetUnit(services, order)
	return units
}

func (d Deployment) serviceUnit(node string, after []string, cycle bool, index int) string {
	var b strings.Builder
	fmt.Fprintf(&b, "# %s\n", header)
	if cycle {
		b.WriteString("# This node is part of a dependency cycle in the graph; systemd cannot\n" +
			"# order it, and the runtime's lifecycle handles the wait instead.\n")
	}
	b.WriteString("[Unit]\n")
	if node == "" {
		fmt.Fprintf(&b, "Description=%s (conductor%s)\n", d.App, d.envSuffix())
	} else {
		fmt.Fprintf(&b, "Description=%s / %s (conductor%s)\n", d.App, node, d.envSuffix())
	}
	fmt.Fprintf(&b, "PartOf=%s\n", d.TargetName())
	ordering := append([]string{"network-online.target"}, after...)
	fmt.Fprintf(&b, "After=%s\n", strings.Join(ordering, " "))
	if len(after) > 0 {
		// Ordering only: a peer restarting must not take this node down,
		// because the lifecycle is what recovers from a missing provider.
		fmt.Fprintf(&b, "Wants=%s\n", strings.Join(after, " "))
	}

	b.WriteString("\n[Service]\n")
	fmt.Fprintf(&b, "Type=simple\n")
	fmt.Fprintf(&b, "WorkingDirectory=%s\n", d.CurrentDir())
	fmt.Fprintf(&b, "ExecStart=%s\n", d.execStart(node, index))
	for _, k := range sortedKeys(d.Environ) {
		fmt.Fprintf(&b, "Environment=%s=%s\n", k, d.Environ[k])
	}
	b.WriteString("Restart=on-failure\nRestartSec=2\n")
	// SIGINT is what the runtime installs a handler for, and the lifecycle
	// teardown it triggers is worth waiting for before SIGKILL.
	b.WriteString("KillSignal=SIGINT\nTimeoutStopSec=10\n")
	b.WriteString("StandardOutput=journal\nStandardError=journal\n")

	fmt.Fprintf(&b, "\n[Install]\nWantedBy=%s\n", d.TargetName())
	return b.String()
}

func (d Deployment) execStart(node string, index int) string {
	args := []string{path.Join(d.CurrentDir(), "bin", d.App)}
	if node != "" {
		args = append(args, "-node", node)
	}
	args = append(args, d.Flags...)
	if addr := d.MetricsAddr(index); addr != "" {
		args = append(args, "-metrics-addr", addr)
	}
	if addr := d.DashboardAddr(index); addr != "" {
		args = append(args, "-dashboard", addr)
	}
	return strings.Join(args, " ")
}

// MetricsAddr offsets the environment's metrics port by a node's position, so
// the nodes of a per-node deployment do not fight over one port. Scraping
// still needs one target per node, which is what a multi-process application
// looks like to Prometheus either way.
func (d Deployment) MetricsAddr(index int) string { return offsetPort(d.Metrics, index) }

// DashboardAddr does the same for the per-process dashboard. The ports are
// assigned by the same rule the fleet view resolves them by, so
// `conductor dashboard -env robot` finds the processes this deployment
// created without being told where they are.
func (d Deployment) DashboardAddr(index int) string { return offsetPort(d.Dashboard, index) }

func offsetPort(addr string, index int) string {
	if addr == "" {
		return ""
	}
	host, port, err := net.SplitHostPort(addr)
	if err != nil {
		return addr
	}
	n, err := strconv.Atoi(port)
	if err != nil {
		return addr
	}
	return net.JoinHostPort(host, strconv.Itoa(n+index))
}

func (d Deployment) targetUnit(services, order []string) string {
	var b strings.Builder
	fmt.Fprintf(&b, "# %s\n", header)
	fmt.Fprintf(&b, "# Bringup order from the application graph: %s\n", strings.Join(order, " -> "))
	b.WriteString("[Unit]\n")
	fmt.Fprintf(&b, "Description=%s (conductor application%s)\n", d.App, d.envSuffix())
	fmt.Fprintf(&b, "Wants=%s\n", strings.Join(services, " "))
	fmt.Fprintf(&b, "After=%s\n", strings.Join(services, " "))
	wantedBy := "multi-user.target"
	if d.Scope == "user" {
		wantedBy = "default.target"
	}
	fmt.Fprintf(&b, "\n[Install]\nWantedBy=%s\n", wantedBy)
	return b.String()
}

func (d Deployment) envSuffix() string {
	if d.Env == "" {
		return ""
	}
	return ", env " + d.Env
}

func sortedKeys(m map[string]string) []string {
	out := make([]string, 0, len(m))
	for k := range m {
		out = append(out, k)
	}
	sort.Strings(out)
	return out
}

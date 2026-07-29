package deploy

import (
	"fmt"
	"net"
	"strings"

	"conductor.dev/conductor"
	"conductor.dev/conductor/internal/gen"
	"conductor.dev/conductor/internal/scan"
)

// Where a deployment's dashboards are is not a thing to write down twice.
// The units are generated with a dashboard port per node — the base port from
// the environment plus the node's position in the bringup order — so the
// fleet view can resolve the same addresses from the same declarations rather
// than being handed a list that drifts.

// FleetPeers returns the dashboard endpoints of every process of every robot
// an environment runs on, robot by robot. An environment with no robots
// declared is one machine, which is the same loop with one iteration.
func FleetPeers(app *scan.App, order []string) ([]conductor.Peer, error) {
	robots := app.Robots()
	if len(robots) == 0 {
		return Peers(app, order), nil
	}
	var out []conductor.Peer
	for _, r := range robots {
		on, err := app.ForRobot(r)
		if err != nil {
			return nil, err
		}
		out = append(out, Peers(on, order)...)
	}
	return out, nil
}

// Peers returns the dashboard endpoints of an environment's processes, in
// bringup order. It is empty when the environment declares no dashboard
// address, because then there is nothing serving one.
func Peers(app *scan.App, order []string) []conductor.Peer {
	if app.Env == nil || app.Env.Dashboard == "" {
		return nil
	}
	dep := gen.Deployment{
		App:           app.Name,
		Dashboard:     app.Env.Dashboard,
		SingleProcess: singleProcess(app),
	}
	host := peerHost(app)
	// A robot is one ROS graph, and its name is what the fleet view labels
	// its nodes with; without a fleet the machine's own name does that job.
	robot, label := "", host
	if app.Robot != nil {
		robot, label = app.Robot.Name, app.Robot.Name
	}

	if dep.SingleProcess {
		// One unit runs every node, so there is one dashboard: the whole
		// application, on the base port.
		return []conductor.Peer{{
			Name:  app.Name,
			Host:  label,
			Robot: robot,
			URL:   peerURL(host, dep.DashboardAddr(0)),
		}}
	}
	peers := make([]conductor.Peer, 0, len(order))
	for i, node := range order {
		peers = append(peers, conductor.Peer{
			Name:  node,
			Host:  label,
			Robot: robot,
			URL:   peerURL(host, dep.DashboardAddr(i)),
		})
	}
	return peers
}

// peerHost is the machine the environment deploys to, as a name to dial and
// to label rows with. An environment with no deploy host runs here.
func peerHost(app *scan.App) string {
	if app.Env == nil || app.Env.Deploy == nil {
		return ""
	}
	host := app.Env.Deploy.Host
	if host == "" || host == "local" {
		return ""
	}
	if i := strings.Index(host, "@"); i >= 0 {
		host = host[i+1:] // ssh's user@host is not a URL host
	}
	return host
}

// peerURL turns a bind address into a URL to fetch. A unit that binds
// 0.0.0.0 or nothing at all is reachable at the deploy host; one that binds
// loopback is only reachable from the robot itself, and saying so with the
// address it actually bound is more useful than silently dialling elsewhere.
func peerURL(host, addr string) string {
	h, port, err := net.SplitHostPort(addr)
	if err != nil {
		return "http://" + addr
	}
	switch {
	case h == "" || h == "0.0.0.0" || h == "::":
		if host == "" {
			host = "127.0.0.1"
		}
	default:
		host = h
	}
	return fmt.Sprintf("http://%s", net.JoinHostPort(host, port))
}

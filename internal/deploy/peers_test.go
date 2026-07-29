package deploy

import (
	"net"
	"strings"
	"testing"

	"conductor.dev/conductor"

	"conductor.dev/conductor/internal/gen"
	"conductor.dev/conductor/internal/scan"
)

func envApp(env *scan.Environment) *scan.App {
	app := &scan.App{Name: "patrol"}
	app.Env = env
	return app
}

// The fleet view resolves the ports the units were generated with, because
// both come from the same rule.
func TestPeersFollowTheUnitPorts(t *testing.T) {
	env := &scan.Environment{
		Transport: "zenoh",
		Dashboard: ":4000",
		Deploy:    &scan.DeployConfig{Host: "pi@patrol-1"},
	}
	order := []string{"localizer", "navigator", "safety_monitor"}
	peers := Peers(envApp(env), order)
	if len(peers) != len(order) {
		t.Fatalf("%d peers, want one per node: %+v", len(peers), peers)
	}

	dep := gen.Deployment{App: "patrol", Dashboard: env.Dashboard}
	want := []string{"http://patrol-1:4000", "http://patrol-1:4001", "http://patrol-1:4002"}
	for i, p := range peers {
		if p.Name != order[i] {
			t.Errorf("peer %d is named %q, want %q", i, p.Name, order[i])
		}
		if p.URL != want[i] {
			t.Errorf("peer %s is at %s, want %s", p.Name, p.URL, want[i])
		}
		// The same offset the unit's ExecStart carries.
		if _, port, err := net.SplitHostPort(dep.DashboardAddr(i)); err != nil || !strings.HasSuffix(p.URL, ":"+port) {
			t.Errorf("peer %s is not on the unit's port %s (%v)", p.Name, port, err)
		}
	}
}

// The in-process transport deploys as one unit, so there is one dashboard:
// the whole application on the base port.
func TestPeersForASingleProcessDeployment(t *testing.T) {
	env := &scan.Environment{Transport: "inproc", Dashboard: "127.0.0.1:4000"}
	peers := Peers(envApp(env), []string{"localizer", "navigator"})
	if len(peers) != 1 {
		t.Fatalf("%d peers, want 1 for a single-process deployment: %+v", len(peers), peers)
	}
	if peers[0].Name != "patrol" || peers[0].URL != "http://127.0.0.1:4000" {
		t.Fatalf("peer = %+v", peers[0])
	}
}

// An environment with no dashboard address has nothing serving one, and
// saying so is better than inventing an address.
func TestPeersNeedADashboardAddress(t *testing.T) {
	if peers := Peers(envApp(&scan.Environment{Transport: "zenoh"}), []string{"a"}); peers != nil {
		t.Fatalf("peers = %+v, want none", peers)
	}
	if peers := Peers(&scan.App{Name: "patrol"}, []string{"a"}); peers != nil {
		t.Fatalf("peers without an environment = %+v, want none", peers)
	}
}

// A bind address that says nothing about the host is reached at the deploy
// host; one that names a host is taken at its word.
func TestPeerURLs(t *testing.T) {
	cases := []struct{ host, addr, want string }{
		{"robot-1", ":4000", "http://robot-1:4000"},
		{"robot-1", "0.0.0.0:4000", "http://robot-1:4000"},
		{"", ":4000", "http://127.0.0.1:4000"},
		{"robot-1", "10.0.0.5:4000", "http://10.0.0.5:4000"},
	}
	for _, c := range cases {
		if got := peerURL(c.host, c.addr); got != c.want {
			t.Errorf("peerURL(%q, %q) = %q, want %q", c.host, c.addr, got, c.want)
		}
	}
}

// ssh's user@host is not a URL host.
func TestPeerHostDropsTheSSHUser(t *testing.T) {
	for _, c := range [][2]string{{"pi@patrol-1", "patrol-1"}, {"patrol-1", "patrol-1"}, {"local", ""}, {"", ""}} {
		app := envApp(&scan.Environment{Deploy: &scan.DeployConfig{Host: c[0]}})
		if got := peerHost(app); got != c[1] {
			t.Errorf("peerHost(%q) = %q, want %q", c[0], got, c[1])
		}
	}
}

// A fleet's peers are every process of every robot, labelled by the machine
// they run on and reached at whatever address that machine declares.
func TestFleetPeers(t *testing.T) {
	app := &scan.App{Name: "patrol"}
	app.Env = &scan.Environment{
		Transport: "zenoh",
		Dashboard: ":4000",
		Deploy:    &scan.DeployConfig{Host: "pi@spare"},
		Robots: []*scan.Robot{
			{Name: "patrol-1", Host: "pi@patrol-1"},
			{Name: "patrol-2", Host: "pi@patrol-2", Dashboard: ":4100"},
		},
	}

	peers, err := FleetPeers(app, []string{"localizer", "navigator"})
	if err != nil {
		t.Fatal(err)
	}
	want := []conductor.Peer{
		{Name: "localizer", Host: "patrol-1", Robot: "patrol-1", URL: "http://patrol-1:4000"},
		{Name: "navigator", Host: "patrol-1", Robot: "patrol-1", URL: "http://patrol-1:4001"},
		{Name: "localizer", Host: "patrol-2", Robot: "patrol-2", URL: "http://patrol-2:4100"},
		{Name: "navigator", Host: "patrol-2", Robot: "patrol-2", URL: "http://patrol-2:4101"},
	}
	if len(peers) != len(want) {
		t.Fatalf("%d peers, want %d: %+v", len(peers), len(want), peers)
	}
	for i := range want {
		if peers[i] != want[i] {
			t.Errorf("peer %d = %+v, want %+v", i, peers[i], want[i])
		}
	}
}

// An environment with no robots is one machine, which is the same call with
// one iteration.
func TestFleetPeersWithoutAFleet(t *testing.T) {
	app := envApp(&scan.Environment{Transport: "zenoh", Dashboard: ":4000",
		Deploy: &scan.DeployConfig{Host: "pi@patrol-1"}})
	peers, err := FleetPeers(app, []string{"localizer"})
	if err != nil {
		t.Fatal(err)
	}
	if len(peers) != 1 || peers[0].Robot != "" || peers[0].URL != "http://patrol-1:4000" {
		t.Fatalf("peers = %+v", peers)
	}
}

// The gate's question: everything answering, every node Active.
func TestHealthProblems(t *testing.T) {
	healthy := conductor.FleetState{Processes: []conductor.ProcessView{{
		Label: "patrol-1/navigator", OK: true,
		Nodes: []conductor.NodeView{{Name: "navigator", State: "active"}},
	}}}
	if got := healthProblems(healthy); len(got) != 0 {
		t.Fatalf("healthy robot reported %v", got)
	}

	sick := conductor.FleetState{Processes: []conductor.ProcessView{
		{Label: "patrol-1/localizer", OK: false, Err: "connection refused"},
		{Label: "patrol-1/navigator", OK: true,
			Nodes: []conductor.NodeView{{Name: "navigator", State: "inactive"}}},
		{Label: "patrol-1/patroller", OK: true},
	}}
	got := strings.Join(healthProblems(sick), "; ")
	for _, want := range []string{"localizer is not answering", "navigator is inactive", "patroller reports no nodes"} {
		if !strings.Contains(got, want) {
			t.Errorf("problems %q missing %q", got, want)
		}
	}
}

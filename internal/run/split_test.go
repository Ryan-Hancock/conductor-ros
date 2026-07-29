package run

import (
	"strings"
	"testing"

	"conductor.dev/conductor/internal/graph"
	"conductor.dev/conductor/internal/scan"
)

// splitApp is a three-node chain: localizer -> navigator -> driver, which is
// also the bringup order a split run starts them in.
func splitApp(transport string) (*scan.App, *graph.Graph) {
	app := &scan.App{
		Name: "rover", Dir: "/src/rover", ModuleRoot: "/src",
		Messages: map[string]string{"msgs.Twist": "geometry_msgs/msg/Twist", "msgs.Pose": "geometry_msgs/msg/Pose"},
		Stamped:  map[string]bool{},
		Nodes: []*scan.Node{
			{
				Name: "localizer", StructName: "Localizer",
				Pubs:    []scan.Endpoint{{Field: "Pose", Topic: "pose", QoS: "reliable", GoType: "msgs.Pose"}},
				Methods: map[string]scan.MethodSig{},
			},
			{
				Name: "navigator", StructName: "Navigator",
				Subs:    []scan.Endpoint{{Field: "Pose", Topic: "pose", QoS: "reliable", GoType: "msgs.Pose"}},
				Pubs:    []scan.Endpoint{{Field: "Cmd", Topic: "cmd_vel", QoS: "reliable", GoType: "msgs.Twist"}},
				Methods: map[string]scan.MethodSig{"OnPose": {Params: 1}},
			},
			{
				Name: "driver", StructName: "Driver",
				Subs:    []scan.Endpoint{{Field: "Cmd", Topic: "cmd_vel", QoS: "reliable", GoType: "msgs.Twist"}},
				Methods: map[string]scan.MethodSig{"OnCmd": {Params: 1}},
			},
		},
	}
	app.Env = &scan.Environment{Transport: transport, Endpoint: "tcp/127.0.0.1:7447"}
	g, _ := graph.Validate(app)
	return app, g
}

// Splitting an in-process application would start nodes that cannot hear each
// other. That is the same mistake `conductor deploy` refuses by making one
// unit instead of many, and it is refused here for the same reason.
func TestSplitRefusesTheInProcessTransport(t *testing.T) {
	app, g := splitApp("inproc")
	var out strings.Builder
	s := &Session{app: app, graph: g, out: &out, opts: Options{Split: true}}

	err := s.runSplit()
	if err == nil {
		t.Fatal("a split inproc run was accepted")
	}
	for _, want := range []string{"in-process bus", "cannot hear each other", "conductor deploy"} {
		if !strings.Contains(err.Error(), want) {
			t.Errorf("error %q does not explain %q", err, want)
		}
	}
}

// The ports a split run assigns are the ports the units would have, because
// both come from the same rule — which is what lets the fleet view find them.
func TestSplitPortsFollowTheDeploymentRule(t *testing.T) {
	app, _ := splitApp("zenoh")
	app.Env.Dashboard = ":4000"
	app.Env.Metrics = ":9090"
	s := &Session{app: app, out: &strings.Builder{}, opts: Options{Split: true}}

	if got := s.dashboardBase(); got != ":4000" {
		t.Fatalf("dashboard base = %q, want the environment's", got)
	}
	// The fleet view sits clear of the per-node ports.
	if got := s.fleetAddr(); got != "127.0.0.1:4500" {
		t.Fatalf("fleet address = %q", got)
	}

	// An environment that says nothing still gets a development default, on
	// loopback rather than on every interface.
	plain := &Session{app: &scan.App{Name: "rover", Env: &scan.Environment{}}, out: &strings.Builder{}}
	if got := plain.dashboardBase(); got != defaultDashboardBase {
		t.Fatalf("default base = %q, want %q", got, defaultDashboardBase)
	}
	if !strings.HasPrefix(plain.dashboardBase(), "127.0.0.1") {
		t.Errorf("the development default binds beyond loopback: %q", plain.dashboardBase())
	}

	// An explicit flag wins over the environment.
	chosen := &Session{app: app, out: &strings.Builder{}, opts: Options{Dashboard: "127.0.0.1:6000"}}
	if got := chosen.dashboardBase(); got != "127.0.0.1:6000" {
		t.Fatalf("flag did not win: %q", got)
	}
}

// A split run assigns its own per-node addresses, so an address the
// environment implied for a single process is dropped rather than shared by
// four of them.
func TestSplitDropsTheSharedAddresses(t *testing.T) {
	flags := []string{"-transport", "zenoh", "-dashboard", ":4000", "-metrics-addr", ":9090", "-trace"}
	got := strings.Join(withoutDashboard(flags), " ")
	if got != "-transport zenoh -trace" {
		t.Fatalf("flags = %q, want the shared addresses removed", got)
	}
}

// Whether to open a browser is decided by looking around, and an explicit
// answer always wins — the same command runs over ssh and in CI.
func TestBrowserPreference(t *testing.T) {
	t.Setenv("CI", "1")
	if desktop() {
		t.Error("CI is not a desktop session")
	}
	t.Setenv("CI", "")
	t.Setenv("SSH_CONNECTION", "10.0.0.1 22 10.0.0.2 22")
	if desktop() {
		t.Error("an ssh session is not a desktop session")
	}
}

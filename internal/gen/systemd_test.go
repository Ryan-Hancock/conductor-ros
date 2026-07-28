package gen

import (
	"strings"
	"testing"

	"conductor.dev/conductor/internal/graph"
	"conductor.dev/conductor/internal/scan"
)

// chainApp is localizer -> navigator -> driver: each node consumes what the
// one before it publishes, which is the ordering the units must encode.
func chainApp() *graph.Graph {
	app := &scan.App{
		Name:     "rover",
		Messages: map[string]string{"msgs.Twist": "geometry_msgs/msg/Twist", "msgs.Pose": "geometry_msgs/msg/Pose"},
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
	g, _ := graph.Validate(app)
	return g
}

func testDeployment() Deployment {
	return Deployment{
		App: "rover", Env: "robot", Version: "20260101-000000",
		Prefix: "/opt/conductor", Scope: "system",
		Flags: []string{"-transport", "zenoh"},
	}
}

func TestSystemdOrderingFollowsTheGraph(t *testing.T) {
	units := SystemdUnits(chainApp(), testDeployment())

	for _, name := range []string{"rover-localizer.service", "rover-navigator.service", "rover-driver.service", "rover.target"} {
		if _, ok := units[name]; !ok {
			t.Fatalf("missing unit %s (have %v)", name, keys(units))
		}
	}
	// navigator subscribes to what localizer publishes.
	nav := units["rover-navigator.service"]
	if !strings.Contains(nav, "After=network-online.target rover-localizer.service") {
		t.Errorf("navigator is not ordered after localizer:\n%s", nav)
	}
	// The first node has nothing to wait for.
	if strings.Contains(units["rover-localizer.service"], "Wants=rover-") {
		t.Errorf("localizer should not depend on another node:\n%s", units["rover-localizer.service"])
	}
	// Ordering, not binding: a restarting provider must not stop consumers.
	if strings.Contains(nav, "Requires=") || strings.Contains(nav, "BindsTo=") {
		t.Errorf("unit ordering must not be a hard requirement:\n%s", nav)
	}
	if !strings.Contains(units["rover-driver.service"], "After=network-online.target rover-navigator.service") {
		t.Errorf("driver is not ordered after navigator:\n%s", units["rover-driver.service"])
	}
}

func TestSystemdExecStartCarriesNodeAndFlags(t *testing.T) {
	units := SystemdUnits(chainApp(), testDeployment())
	want := "ExecStart=/opt/conductor/rover/current/bin/rover -node navigator -transport zenoh"
	if !strings.Contains(units["rover-navigator.service"], want) {
		t.Errorf("ExecStart is wrong:\n%s", units["rover-navigator.service"])
	}
}

func TestSystemdSingleProcess(t *testing.T) {
	d := testDeployment()
	d.SingleProcess = true
	units := SystemdUnits(chainApp(), d)

	if len(units) != 2 {
		t.Fatalf("single-process deployment produced %d units, want 2: %v", len(units), keys(units))
	}
	svc := units["rover.service"]
	if strings.Contains(svc, "-node ") {
		t.Errorf("the single-process unit must run every node:\n%s", svc)
	}
	if !strings.Contains(units["rover.target"], "Wants=rover.service") {
		t.Errorf("target does not want the service:\n%s", units["rover.target"])
	}
}

// Several processes cannot share one metrics port, so each unit gets the
// base port offset by its position in the bringup order.
func TestSystemdMetricsPortsDoNotCollide(t *testing.T) {
	d := testDeployment()
	d.Metrics = ":9090"
	units := SystemdUnits(chainApp(), d)

	want := map[string]string{
		"rover-localizer.service": "-metrics-addr :9090",
		"rover-navigator.service": "-metrics-addr :9091",
		"rover-driver.service":    "-metrics-addr :9092",
	}
	for unit, addr := range want {
		if !strings.Contains(units[unit], addr) {
			t.Errorf("%s does not expose %s:\n%s", unit, addr, units[unit])
		}
	}

	d.SingleProcess = true
	if !strings.Contains(SystemdUnits(chainApp(), d)["rover.service"], "-metrics-addr :9090") {
		t.Error("a single-process deployment should use the base port unchanged")
	}
}

func TestMetricsAddrWithHost(t *testing.T) {
	d := Deployment{Metrics: "127.0.0.1:9090"}
	if got := d.MetricsAddr(2); got != "127.0.0.1:9092" {
		t.Errorf("MetricsAddr = %q, want 127.0.0.1:9092", got)
	}
	// Anything unparseable is passed through rather than mangled.
	d.Metrics = "unix:/run/metrics.sock"
	if got := d.MetricsAddr(1); got != "unix:/run/metrics.sock" {
		t.Errorf("MetricsAddr = %q, want the address unchanged", got)
	}
}

func TestInstallScriptIsSelfDescribing(t *testing.T) {
	d := testDeployment()
	script := InstallScript(d, []string{"rover-localizer.service", "rover.target"}, 3)

	for _, want := range []string{
		"APP=rover",
		"VERSION=20260101-000000",
		"PREFIX=/opt/conductor",
		"SCOPE=system",
		"KEEP=3",
		"TARGET=rover.target",
		"UNITS=(rover-localizer.service rover.target)",
		"--rollback",
		"removed stale unit", // units a release no longer has are cleaned up
	} {
		if !strings.Contains(script, want) {
			t.Errorf("install.sh is missing %q", want)
		}
	}
}

func TestInstallScriptQuotesAwkwardValues(t *testing.T) {
	d := Deployment{App: "rover", Version: "v1 'x'", Prefix: "/opt/my apps", Scope: "user"}
	script := InstallScript(d, nil, 0)
	if !strings.Contains(script, `PREFIX='/opt/my apps'`) {
		t.Errorf("prefix with a space was not quoted:\n%s", script)
	}
	if !strings.Contains(script, `VERSION='v1 '\''x'\'''`) {
		t.Errorf("version with a quote was not escaped:\n%s", script)
	}
}

func keys(m map[string]string) []string {
	out := make([]string, 0, len(m))
	for k := range m {
		out = append(out, k)
	}
	return out
}

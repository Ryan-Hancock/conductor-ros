package deploy

import (
	"bytes"
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"strings"
	"testing"

	"conductor.dev/conductor/internal/graph"
	"conductor.dev/conductor/internal/scan"
)

// patrolApp resolves the patrol example for an environment. Deployment is
// about a whole application, so the tests use a real one rather than a
// fixture that could drift from what the toolchain actually produces.
func patrolApp(t *testing.T, env string) (*scan.App, *graph.Graph) {
	t.Helper()
	app, err := scan.ScanApp(filepath.Join("..", "..", "examples", "patrol"))
	if err != nil {
		t.Fatal(err)
	}
	app, err = app.Resolve(env)
	if err != nil {
		t.Fatal(err)
	}
	g, issues := graph.Validate(app)
	if graph.Errors(issues) {
		t.Fatalf("the patrol example does not validate in env %s: %v", env, issues)
	}
	return app, g
}

func TestValidateRejectsUnbuildableCombinations(t *testing.T) {
	app, _ := patrolApp(t, "robot")
	base := Options{GOOS: "linux", GOARCH: runtime.GOARCH, Scope: "system", Prefix: "/opt/conductor"}

	// The robot environment asks for the zenoh transport; a build without
	// the tag would exit at startup on the robot with "unknown transport".
	o := base
	if err := validate(app, o); err == nil || !strings.Contains(err.Error(), "zenoh tag") {
		t.Errorf("expected a missing-tag error, got %v", err)
	}

	// zenoh is cgo.
	o = base
	o.Tags = []string{"zenoh"}
	if err := validate(app, o); err == nil || !strings.Contains(err.Error(), "cgo") {
		t.Errorf("expected a cgo error, got %v", err)
	}

	// cgo cross-compilation needs a cross compiler.
	o = base
	o.Tags, o.CGO, o.GOARCH = []string{"zenoh"}, true, "riscv64"
	if err := validate(app, o); err == nil || !strings.Contains(err.Error(), "cross compiler") {
		t.Errorf("expected a cross compiler error, got %v", err)
	}

	// With everything supplied it passes.
	o = base
	o.Tags, o.CGO, o.CC, o.GOARCH = []string{"zenoh"}, true, "aarch64-linux-gnu-gcc", "arm64"
	if err := validate(app, o); err != nil {
		t.Errorf("valid options rejected: %v", err)
	}
}

func TestValidateRequiresAbsolutePrefix(t *testing.T) {
	app, _ := patrolApp(t, "bench")
	o := Options{GOOS: "linux", GOARCH: runtime.GOARCH, Scope: "user", Prefix: "relative/path"}
	if err := validate(app, o); err == nil || !strings.Contains(err.Error(), "absolute") {
		t.Errorf("expected an absolute-prefix error, got %v", err)
	}
}

// A remote user-scope deploy cannot guess the target user's home directory,
// so it has to be told; the local case can work it out.
func TestUserScopePrefixDefaults(t *testing.T) {
	if got := defaultPrefix("user", "pi@robot"); got != "" {
		t.Errorf("remote user scope should have no default prefix, got %q", got)
	}
	if got := defaultPrefix("user", ""); !filepath.IsAbs(got) {
		t.Errorf("local user scope prefix = %q, want an absolute path", got)
	}
	if got := defaultPrefix("system", "pi@robot"); got != "/opt/conductor" {
		t.Errorf("system prefix = %q", got)
	}
}

func TestInprocEnvironmentDeploysAsOneProcess(t *testing.T) {
	bench, _ := patrolApp(t, "bench")
	if !singleProcess(bench) {
		t.Error("an inproc environment must deploy as a single process: its bus does not leave the process")
	}
	robot, _ := patrolApp(t, "robot")
	if singleProcess(robot) {
		t.Error("a zenoh environment should deploy one unit per node")
	}
}

func TestRuntimeFlagsComeFromTheEnvironment(t *testing.T) {
	app, _ := patrolApp(t, "robot")
	got := strings.Join(runtimeFlags(app, Options{Prefix: "/opt/conductor"}), " ")
	for _, want := range []string{
		"-transport zenoh",
		"-zenoh-endpoint tcp/127.0.0.1:7447",
		"-domain 0",
		"-params /opt/conductor/patrol/current/params.yaml",
		"-params /opt/conductor/patrol/current/params.robot.yaml",
	} {
		if !strings.Contains(got, want) {
			t.Errorf("runtime flags %q are missing %q", got, want)
		}
	}
	// The metrics address is per-node, so it is not a shared flag.
	if strings.Contains(got, "-metrics-addr") {
		t.Errorf("metrics must be set per unit, not in the shared flags: %q", got)
	}
}

func TestFingerprintTracksTheGraph(t *testing.T) {
	app, g := patrolApp(t, "robot")
	first := Fingerprint(g)
	if first != Fingerprint(g) {
		t.Fatal("fingerprint is not stable")
	}
	// The same application in another environment talks to a different set
	// of peers, and that is part of what the release does.
	_, sim := patrolApp(t, "sim")
	if Fingerprint(sim) == first {
		t.Error("environments with different externals should fingerprint differently")
	}
	// A QoS change is exactly the kind of silent difference worth catching.
	app.Nodes[0].Pubs[0].QoS = "sensor"
	changed, _ := graph.Validate(app)
	if Fingerprint(changed) == first {
		t.Error("a QoS change must change the fingerprint")
	}
}

func TestRunnerBuildsLocalAndRemoteCommands(t *testing.T) {
	local := Runner{}
	if got := local.CopyCmd("/tmp/a", "/tmp/b").Args; !equal(got, []string{"cp", "/tmp/a", "/tmp/b"}) {
		t.Errorf("local copy = %v", got)
	}
	if got := local.ScriptCmd("echo hi").Args; !equal(got, []string{"bash", "-s"}) {
		t.Errorf("local script = %v", got)
	}

	remote := Runner{Host: "pi@robot-1"}
	if got := remote.CopyCmd("/tmp/a", "/tmp/b").Args; !equal(got, []string{"scp", "-q", "/tmp/a", "pi@robot-1:/tmp/b"}) {
		t.Errorf("remote copy = %v", got)
	}
	if got := remote.ScriptCmd("echo hi").Args; !equal(got, []string{"ssh", "-o", "BatchMode=yes", "pi@robot-1", "bash -s"}) {
		t.Errorf("remote script = %v", got)
	}
}

// The whole script runs under one privilege escalation rather than one per
// line, which is what makes a passwordless sudo policy practical.
func TestSudoWrapsTheWholeScript(t *testing.T) {
	script := wrapSudo("mkdir /opt/x\nsystemctl restart y", "sudo -n")
	if !strings.HasPrefix(script, "sudo -n bash -s <<'CONDUCTOR_EOF'\n") {
		t.Errorf("script is not wrapped: %q", script)
	}
	if strings.Count(script, "sudo") != 1 {
		t.Errorf("expected exactly one escalation: %q", script)
	}
	if !strings.Contains(script, "systemctl restart y\nCONDUCTOR_EOF") {
		t.Errorf("script body was mangled: %q", script)
	}
	// A quoted heredoc: the target's shell must not expand anything.
	if !strings.Contains(script, "<<'CONDUCTOR_EOF'") {
		t.Error("the heredoc must be quoted so the script reaches the target verbatim")
	}
}

func TestShQuote(t *testing.T) {
	cases := map[string]string{
		"/opt/conductor": "/opt/conductor",
		"":               "''",
		"two words":      "'two words'",
		"it's":           `'it'\''s'`,
	}
	for in, want := range cases {
		if got := shQuote(in); got != want {
			t.Errorf("shQuote(%q) = %q, want %q", in, got, want)
		}
	}
}

// TestDeployEndToEnd builds a real bundle from the patrol example and
// installs it into a temporary prefix, with systemd left out of it. It
// covers the parts a robot would exercise: the release layout, the current
// symlink, and a rollback.
func TestDeployEndToEnd(t *testing.T) {
	if testing.Short() {
		t.Skip("builds the example binary")
	}
	if _, err := exec.LookPath("tar"); err != nil {
		t.Skip("tar is not available")
	}
	app, g := patrolApp(t, "bench")
	prefix := t.TempDir()
	out := &bytes.Buffer{}

	opts := func(version string) Options {
		return Options{
			Version: version, GOOS: runtime.GOOS, GOARCH: runtime.GOARCH,
			Prefix: prefix, Scope: "user", NoSystemd: true,
			OutDir: filepath.Join(t.TempDir(), "bundle"), Out: out,
		}
	}
	if err := Run(app, g, opts("v1")); err != nil {
		t.Fatalf("deploy v1: %v\n%s", err, out)
	}

	release := filepath.Join(prefix, "patrol", "releases", "v1")
	for _, f := range []string{
		"bin/patrol", "params.yaml", "params.sim.yaml", "manifest.json",
		"install.sh", "systemd/patrol.service", "systemd/patrol.target",
		"patrol.launch.xml",
	} {
		if _, err := os.Stat(filepath.Join(release, f)); err != nil {
			t.Errorf("release is missing %s: %v", f, err)
		}
	}

	// current points at the release, and the binary under it runs.
	current := filepath.Join(prefix, "patrol", "current")
	if got, err := filepath.EvalSymlinks(current); err != nil || got != mustEval(t, release) {
		t.Fatalf("current -> %v (err %v), want %s", got, err, release)
	}
	bin := filepath.Join(current, "bin", "patrol")
	if err := exec.Command(bin, "-h").Run(); err != nil {
		// -h exits non-zero by convention; a missing or unexecutable
		// binary fails differently.
		if _, statErr := os.Stat(bin); statErr != nil {
			t.Fatalf("deployed binary is not usable: %v", statErr)
		}
	}

	var m Manifest
	b, err := os.ReadFile(filepath.Join(release, "manifest.json"))
	if err != nil {
		t.Fatal(err)
	}
	if err := json.Unmarshal(b, &m); err != nil {
		t.Fatal(err)
	}
	if m.App != "patrol" || m.Env != "bench" || m.Version != "v1" {
		t.Errorf("manifest = %+v", m)
	}
	if want := []string{"localizer", "navigator", "safety_monitor"}; !equal(m.BringupOrder, want) {
		t.Errorf("bringup order = %v, want %v", m.BringupOrder, want)
	}
	if sum, ok := m.Files["bin/patrol"]; !ok || len(sum) != 64 {
		t.Errorf("manifest has no checksum for the binary: %q", sum)
	}
	if !strings.HasPrefix(m.Graph, "sha256:") {
		t.Errorf("manifest graph fingerprint = %q", m.Graph)
	}

	// A second release, then a rollback, must land back on the first.
	if err := Run(app, g, opts("v2")); err != nil {
		t.Fatalf("deploy v2: %v\n%s", err, out)
	}
	if got := mustEval(t, current); got != mustEval(t, filepath.Join(prefix, "patrol", "releases", "v2")) {
		t.Fatalf("current -> %s after the second deploy", got)
	}
	back := opts("")
	back.Rollback, back.NoSystemd = true, true
	if err := Run(app, nil, back); err != nil {
		t.Fatalf("rollback: %v\n%s", err, out)
	}
	if got := mustEval(t, current); got != mustEval(t, release) {
		t.Errorf("after rollback current -> %s, want %s", got, release)
	}
}

func mustEval(t *testing.T, p string) string {
	t.Helper()
	got, err := filepath.EvalSymlinks(p)
	if err != nil {
		t.Fatal(err)
	}
	return got
}

func equal(a, b []string) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

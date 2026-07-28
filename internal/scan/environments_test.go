package scan

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// writeApp lays out a minimal app directory with the given config files.
func writeApp(t *testing.T, files map[string]string) string {
	t.Helper()
	dir := t.TempDir()
	files["go.mod"] = "module example.test\n\ngo 1.24\n"
	for name, content := range files {
		if err := os.WriteFile(filepath.Join(dir, name), []byte(content), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	return dir
}

const baseConfig = `{
  "app": "rover",
  "externals": [
    {"topic": "estop", "type": "std_msgs/msg/Bool", "role": "publisher", "qos": "transient"},
    {"topic": "cmd_vel", "type": "geometry_msgs/msg/Twist", "role": "subscriber", "qos": "reliable"},
    {"topic": "engage_estop", "type": "std_srvs/srv/SetBool", "role": "client"}
  ]
}`

const envConfigJSON = `{
  "default": "sim",
  "environments": {
    "sim": {
      "transport": "inproc",
      "without": ["engage_estop"],
      "externals": [
        {"topic": "estop", "type": "std_msgs/msg/Bool", "role": "publisher", "qos": "reliable"},
        {"topic": "clock", "type": "rosgraph_msgs/msg/Clock", "role": "publisher", "qos": "reliable"}
      ]
    },
    "robot": {
      "transport": "zenoh",
      "params": ["params.robot.yaml"],
      "deploy": {"host": "pi@rover-1", "goarch": "arm64", "tags": ["zenoh"], "cgo": true}
    }
  }
}`

func scanApp(t *testing.T) *App {
	t.Helper()
	dir := writeApp(t, map[string]string{
		"conductor.json":    baseConfig,
		"environments.json": envConfigJSON,
	})
	app, err := ScanApp(dir)
	if err != nil {
		t.Fatal(err)
	}
	return app
}

func externalIndex(app *App) map[string]External {
	m := map[string]External{}
	for _, e := range app.Externals {
		m[e.Topic+"/"+e.Role] = e
	}
	return m
}

func TestResolveDefaultEnvironment(t *testing.T) {
	app := scanApp(t)
	if app.DefaultEnv != "sim" {
		t.Fatalf("DefaultEnv = %q, want sim", app.DefaultEnv)
	}
	got, err := app.Resolve("")
	if err != nil {
		t.Fatal(err)
	}
	if got.Env == nil || got.Env.Name() != "sim" {
		t.Fatalf("Resolve(\"\") did not select the declared default: %+v", got.Env)
	}
}

func TestResolveMergesExternals(t *testing.T) {
	app := scanApp(t)
	sim, err := app.Resolve("sim")
	if err != nil {
		t.Fatal(err)
	}
	ext := externalIndex(sim)

	// Same topic and role: the environment's entry replaces the base one.
	if got := ext["estop/publisher"].QoS; got != "reliable" {
		t.Errorf("estop qos = %q, want the sim override %q", got, "reliable")
	}
	// New in this environment.
	if _, ok := ext["clock/publisher"]; !ok {
		t.Error("sim external clock is missing")
	}
	// Dropped by "without".
	if _, ok := ext["engage_estop/client"]; ok {
		t.Error("engage_estop should be dropped in sim")
	}
	// Untouched base entry survives.
	if _, ok := ext["cmd_vel/subscriber"]; !ok {
		t.Error("base external cmd_vel is missing")
	}

	// Resolving must not mutate the app it came from: the next environment
	// has to start from the same base.
	if len(app.Externals) != 3 {
		t.Errorf("base externals were modified: %d, want 3", len(app.Externals))
	}
	robot, err := app.Resolve("robot")
	if err != nil {
		t.Fatal(err)
	}
	if _, ok := externalIndex(robot)["engage_estop/client"]; !ok {
		t.Error("robot lost the base engage_estop client")
	}
	if _, ok := externalIndex(robot)["clock/publisher"]; ok {
		t.Error("robot picked up sim's clock external")
	}
}

func TestResolveDeployConfig(t *testing.T) {
	app := scanApp(t)
	robot, err := app.Resolve("robot")
	if err != nil {
		t.Fatal(err)
	}
	d := robot.Env.Deploy
	if d == nil {
		t.Fatal("robot has no deploy config")
	}
	if d.Host != "pi@rover-1" || d.GOARCH != "arm64" || !d.CGO {
		t.Errorf("deploy config = %+v", d)
	}
	if len(robot.Env.Params) != 1 || robot.Env.Params[0] != "params.robot.yaml" {
		t.Errorf("params = %v", robot.Env.Params)
	}
}

func TestResolveUnknownEnvironment(t *testing.T) {
	app := scanApp(t)
	_, err := app.Resolve("moon")
	if err == nil {
		t.Fatal("expected an error for an undeclared environment")
	}
	// The message must list what is available; a typo is the common case.
	for _, want := range []string{"moon", "sim", "robot"} {
		if !strings.Contains(err.Error(), want) {
			t.Errorf("error %q does not mention %q", err, want)
		}
	}
}

func TestNoEnvironmentsFile(t *testing.T) {
	dir := writeApp(t, map[string]string{"conductor.json": baseConfig})
	app, err := ScanApp(dir)
	if err != nil {
		t.Fatal(err)
	}
	got, err := app.Resolve("")
	if err != nil {
		t.Fatal(err)
	}
	if got.Env != nil {
		t.Error("an app with no environments.json must resolve to no environment")
	}
	if _, err := app.Resolve("robot"); err == nil {
		t.Error("naming an environment when none are declared should fail")
	}
}

func TestBadDefaultEnvironment(t *testing.T) {
	dir := writeApp(t, map[string]string{
		"conductor.json":    baseConfig,
		"environments.json": `{"default": "nope", "environments": {"sim": {}}}`,
	})
	if _, err := ScanApp(dir); err == nil {
		t.Fatal("expected an error for a default that is not declared")
	}
}

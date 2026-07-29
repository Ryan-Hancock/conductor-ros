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

// Sensor calibration differs from robot to robot, so an environment may name
// its own transform tree — and must not silently fall back to another one.
func TestResolveFramesPerEnvironment(t *testing.T) {
	dir := writeApp(t, map[string]string{
		"conductor.json": baseConfig,
		"environments.json": `{
  "default": "sim",
  "environments": {
    "sim":   {"transport": "inproc"},
    "robot": {"transport": "zenoh", "frames": "frames.robot.json"},
    "spare": {"transport": "zenoh", "frames": "frames.spare.json"}
  }
}`,
		"frames.json":       `{"static":[{"parent":"base_link","child":"laser","xyz":[0.1,0,0.2]}]}`,
		"frames.robot.json": `{"static":[{"parent":"base_link","child":"laser","xyz":[0.118,0,0.191]}]}`,
	})
	app, err := ScanApp(dir)
	if err != nil {
		t.Fatal(err)
	}
	if app.FramesFile != "frames.json" || app.Frames.Transforms[0].XYZ[0] != 0.1 {
		t.Fatalf("base frames = %+v (%s)", app.Frames, app.FramesFile)
	}

	robot, err := app.Resolve("robot")
	if err != nil {
		t.Fatal(err)
	}
	if robot.FramesFile != "frames.robot.json" || robot.Frames.Transforms[0].XYZ[0] != 0.118 {
		t.Fatalf("robot frames = %+v (%s)", robot.Frames, robot.FramesFile)
	}
	// Resolving one environment must not disturb the next.
	if app.Frames.Transforms[0].XYZ[0] != 0.1 {
		t.Fatal("resolving the robot environment mutated the base tree")
	}

	if _, err := app.Resolve("spare"); err == nil || !strings.Contains(err.Error(), "frames.spare.json") {
		t.Fatalf("Resolve(spare) = %v, want it to refuse the missing frames file", err)
	}
}

const fleetConfig = `{
  "environments": {
    "robot": {
      "transport": "zenoh",
      "endpoint": "tcp/127.0.0.1:7447",
      "params": ["params.robot.yaml"],
      "dashboard_addr": ":4000",
      "deploy": {"host": "pi@spare", "goarch": "arm64", "prefix": "/opt/conductor"},
      "robots": [
        {"name": "patrol-1", "host": "pi@patrol-1", "frames": "frames.patrol-1.json"},
        {"name": "patrol-2", "host": "pi@patrol-2", "params": ["params.patrol-2.yaml"],
         "dashboard_addr": ":4100", "endpoint": "tcp/10.0.0.2:7447", "scope": "user", "prefix": "/srv/conductor"}
      ]
    }
  }
}`

func fleetApp(t *testing.T) *App {
	t.Helper()
	dir := writeApp(t, map[string]string{
		"conductor.json":       baseConfig,
		"environments.json":    fleetConfig,
		"frames.json":          `{"static":[{"parent":"base_link","child":"laser","xyz":[0.1,0,0.2]}]}`,
		"frames.patrol-1.json": `{"static":[{"parent":"base_link","child":"laser","xyz":[0.118,0,0.191]}]}`,
		"params.robot.yaml":    "navigator:\n  ros__parameters:\n    max_speed: 1.5\n",
		"params.patrol-2.yaml": "navigator:\n  ros__parameters:\n    max_speed: 0.9\n",
	})
	app, err := ScanApp(dir)
	if err != nil {
		t.Fatal(err)
	}
	return app
}

// A robot is its environment with that machine's overrides applied — and
// resolving one must leave the environment alone for the next.
func TestResolveRobotOverrides(t *testing.T) {
	app := fleetApp(t)

	one, err := app.ResolveRobot("robot", "patrol-1")
	if err != nil {
		t.Fatal(err)
	}
	if one.Robot == nil || one.Robot.Name != "patrol-1" {
		t.Fatalf("robot = %+v", one.Robot)
	}
	if one.Env.Deploy.Host != "pi@patrol-1" {
		t.Errorf("host = %q, want the robot's", one.Env.Deploy.Host)
	}
	if one.Env.Deploy.GOARCH != "arm64" {
		t.Errorf("goarch = %q, want the environment's", one.Env.Deploy.GOARCH)
	}
	if one.FramesFile != "frames.patrol-1.json" || one.Frames.Transforms[0].XYZ[0] != 0.118 {
		t.Errorf("frames = %s %+v, want the robot's calibration", one.FramesFile, one.Frames.Transforms[0].XYZ)
	}

	two, err := app.ResolveRobot("robot", "patrol-2")
	if err != nil {
		t.Fatal(err)
	}
	// Parameters append: a robot tunes what the environment set.
	if got := strings.Join(two.Env.Params, ","); got != "params.robot.yaml,params.patrol-2.yaml" {
		t.Errorf("params = %s, want the environment's then the robot's", got)
	}
	for _, c := range [][3]string{
		{"endpoint", two.Env.Endpoint, "tcp/10.0.0.2:7447"},
		{"dashboard", two.Env.Dashboard, ":4100"},
		{"scope", two.Env.Deploy.Scope, "user"},
		{"prefix", two.Env.Deploy.Prefix, "/srv/conductor"},
	} {
		if c[1] != c[2] {
			t.Errorf("%s = %q, want %q", c[0], c[1], c[2])
		}
	}
	// patrol-2 declares no frames of its own, so it keeps the environment's.
	if two.FramesFile != "frames.json" {
		t.Errorf("frames = %s, want the environment's", two.FramesFile)
	}

	// Resolving robots must not have disturbed each other or the environment.
	if one.Env.Endpoint != "tcp/127.0.0.1:7447" || one.Env.Dashboard != ":4000" {
		t.Errorf("patrol-1 picked up patrol-2's overrides: %+v", one.Env)
	}
	if env := app.Environments["robot"]; env.Deploy.Host != "pi@spare" || len(env.Params) != 1 {
		t.Errorf("the environment was mutated: %+v", env)
	}
}

func TestResolveRobotErrors(t *testing.T) {
	app := fleetApp(t)
	if _, err := app.ResolveRobot("robot", "patrol-9"); err == nil ||
		!strings.Contains(err.Error(), "patrol-1, patrol-2") {
		t.Fatalf("error = %v, want the declared robots listed", err)
	}
	plain := scanApp(t) // an environment with no robots
	if _, err := plain.ResolveRobot("sim", "patrol-1"); err == nil ||
		!strings.Contains(err.Error(), "declares no robots") {
		t.Fatalf("error = %v, want it to say the environment has no fleet", err)
	}
}

// A fleet whose robots cannot be told apart is refused when it is read, not
// halfway through a rollout.
func TestRobotsNeedDistinctNames(t *testing.T) {
	for _, bad := range []string{
		`{"environments": {"robot": {"robots": [{"name": "a"}, {"name": "a"}]}}}`,
		`{"environments": {"robot": {"robots": [{"host": "pi@x"}]}}}`,
	} {
		dir := writeApp(t, map[string]string{"conductor.json": baseConfig, "environments.json": bad})
		if _, err := ScanApp(dir); err == nil {
			t.Errorf("accepted %s", bad)
		}
	}
}

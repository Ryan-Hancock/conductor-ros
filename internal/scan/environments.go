package scan

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
)

// Environments are declared in environments.json beside conductor.json:
//
//	{
//	  "default": "sim",
//	  "environments": {
//	    "sim":   {"transport": "inproc", "params": ["params.sim.yaml"],
//	              "externals": [{"topic": "scan", "type": "sensor_msgs/msg/LaserScan",
//	                             "role": "publisher", "qos": "sensor"}]},
//	    "robot": {"transport": "zenoh", "params": ["params.robot.yaml"],
//	              "deploy": {"host": "pi@robot-1", "goarch": "arm64", "tags": ["zenoh"]}}
//	  }
//	}
//
// The application model stays in Go; what differs between a simulator, a
// bench and a robot is *who else is on the graph* and where the binary runs,
// which is configuration. Keeping it in its own file (rather than sections
// inside conductor.json) follows the Encore split: the app is code, the
// environments it runs in are deployment config.

// Environment is one deployment environment.
type Environment struct {
	// Transport, Endpoint and Domain become the runtime flags baked into the
	// generated units and launch file.
	Transport string `json:"transport"`
	Endpoint  string `json:"endpoint"`
	Domain    *int   `json:"domain"`

	// Params lists parameter files (relative to the app directory) loaded
	// after the generated params.yaml, in order.
	Params []string `json:"params"`

	// Externals are added to the base conductor.json externals; an entry
	// with the same topic and role replaces the base one. Without drops
	// base externals by topic — a driver that exists on the robot but not
	// in simulation, so that `conductor check -env sim` says so.
	Externals []External `json:"externals"`
	Without   []string   `json:"without"`

	// Frames names a transform tree file to use instead of frames.json:
	// sensor calibration differs from robot to robot, which is what an
	// environment is.
	Frames string `json:"frames"`

	// Metrics, Dashboard and Trace default the corresponding runtime flags.
	// A dashboard address makes every unit serve its own portal, which is
	// what `conductor dashboard -env <name>` then aggregates.
	Metrics   string `json:"metrics_addr"`
	Dashboard string `json:"dashboard_addr"`
	Trace     bool   `json:"trace"`

	// Requires are the processes that must be running for this environment
	// to work: the zenoh router, a simulator, the drivers a bench stands in
	// for. conductor.json says what is outside the application; this says
	// who provides it here, which is the part that differs between a
	// simulator and a robot.
	Requires []Process `json:"requires"`

	Deploy *DeployConfig `json:"deploy"`

	// Robots are the machines this environment runs on. One robot (or none
	// at all, which means the deploy host) is the ordinary case; several
	// make the environment a fleet, rolled out one robot at a time.
	Robots []*Robot `json:"robots"`

	name string
}

// Robot is one machine an environment runs on. It inherits everything from
// its environment and overrides only what is genuinely per-machine: where it
// is, how it is calibrated, and what it is tuned to.
//
//	"robot": {
//	  "transport": "zenoh",
//	  "deploy": {"goarch": "arm64", "tags": ["zenoh"], "cgo": true},
//	  "robots": [
//	    {"name": "patrol-1", "host": "pi@patrol-1", "frames": "frames.patrol-1.json"},
//	    {"name": "patrol-2", "host": "pi@patrol-2", "params": ["params.patrol-2.yaml"]}
//	  ]
//	}
//
// Modelling a robot as an environment's instance rather than as a separate
// kind of thing is what keeps one code path: resolving an environment for a
// robot produces an ordinary resolved application, and every command that
// takes -env takes -robot the same way.
type Robot struct {
	Name string `json:"name"`
	Host string `json:"host"` // user@host for ssh; empty deploys to this machine

	// Per-machine overrides. Params are appended after the environment's, so
	// a robot tunes rather than replaces; the rest replace when set.
	Params    []string `json:"params"`
	Frames    string   `json:"frames"`
	Domain    *int     `json:"domain"`
	Endpoint  string   `json:"endpoint"`
	Metrics   string   `json:"metrics_addr"`
	Dashboard string   `json:"dashboard_addr"`
	Prefix    string   `json:"prefix"`
	Scope     string   `json:"scope"`
}

// Process is something `conductor run` starts before the application, and
// stops after it:
//
//	"requires": [
//	  {"name": "router",    "run": "$CONDUCTOR_OVERLAY/lib/rmw_zenoh_cpp/rmw_zenohd",
//	                        "ready": {"endpoint": "tcp/127.0.0.1:7447"}},
//	  {"name": "turtlesim", "run": "ros2 run turtlesim turtlesim_node",
//	                        "ready": {"command": "ros2 topic list | grep -q /turtle1/pose"}}
//	]
//
// Conductor deliberately knows nothing about how ROS is installed: what to
// run is a command, and whether it is up is a condition. What it does know is
// that the condition is worth waiting for — a development bringup written in
// shell says `sleep 4` there, and that is where the flakiness comes from.
type Process struct {
	Name string            `json:"name"`
	Run  string            `json:"run"` // a command line, run through sh
	Dir  string            `json:"dir"` // working directory, relative to the app
	Env  map[string]string `json:"env"`

	Ready   Readiness `json:"ready"`
	Timeout string    `json:"ready_timeout"` // default 30s
}

// Readiness is how to tell that a required process is up. The kinds are
// checked in this order, and a process with none declared is simply started.
type Readiness struct {
	// Endpoint is a zenoh-style endpoint ("tcp/127.0.0.1:7447") or a plain
	// host:port; ready means the port accepts a connection.
	Endpoint string `json:"endpoint"`
	// Command is polled until it exits 0.
	Command string `json:"command"`
	// Delay waits a fixed time — the honest last resort, and the thing every
	// other kind exists to avoid.
	Delay string `json:"delay"`
}

// Declared reports whether any readiness condition was given.
func (r Readiness) Declared() bool {
	return r.Endpoint != "" || r.Command != "" || r.Delay != ""
}

// DeployConfig is where an environment's binary goes and how it is built.
type DeployConfig struct {
	Host   string            `json:"host"`   // user@host for ssh; empty means this machine
	GOOS   string            `json:"goos"`   // default linux
	GOARCH string            `json:"goarch"` // default the host's
	Tags   []string          `json:"tags"`   // build tags (e.g. ["zenoh"])
	CGO    bool              `json:"cgo"`    // required by the zenoh transport
	CC     string            `json:"cc"`     // cross compiler, when CGO is on
	Scope  string            `json:"scope"`  // systemd scope: system (default) or user
	Prefix string            `json:"prefix"` // install root, default /opt/conductor
	Sudo   string            `json:"sudo"`   // privilege prefix for system scope, default "sudo -n"
	Env    map[string]string `json:"env"`    // extra Environment= lines in the units
	Keep   int               `json:"keep"`   // releases to retain, default 5
}

// Name reports the environment's name as declared.
func (e *Environment) Name() string { return e.name }

type envConfig struct {
	Default      string                  `json:"default"`
	Environments map[string]*Environment `json:"environments"`
}

// loadEnvironments reads environments.json if present.
func loadEnvironments(dir string, app *App) error {
	path := filepath.Join(dir, "environments.json")
	b, err := os.ReadFile(path)
	if err != nil {
		return nil // optional file
	}
	var c envConfig
	if err := json.Unmarshal(b, &c); err != nil {
		return fmt.Errorf("environments.json: %w", err)
	}
	app.Environments = map[string]*Environment{}
	for name, env := range c.Environments {
		if env == nil {
			env = &Environment{}
		}
		env.name = name
		app.Environments[name] = env
		app.EnvNames = append(app.EnvNames, name)
	}
	sort.Strings(app.EnvNames)
	app.DefaultEnv = c.Default
	if app.DefaultEnv != "" && app.Environments[app.DefaultEnv] == nil {
		return fmt.Errorf("environments.json: default environment %q is not declared", app.DefaultEnv)
	}
	for _, name := range app.EnvNames {
		if err := checkRobots(app.Environments[name]); err != nil {
			return fmt.Errorf("environments.json: %w", err)
		}
	}
	return nil
}

// checkRobots refuses a fleet that cannot be addressed: a rollout names
// robots one at a time, in logs and on the command line, so they need names,
// and two machines answering to one name is a mistake worth catching here
// rather than halfway through a rollout.
func checkRobots(env *Environment) error {
	seen := map[string]bool{}
	for i, r := range env.Robots {
		if r == nil {
			return fmt.Errorf("environment %q: robot %d is empty", env.name, i)
		}
		if r.Name == "" {
			return fmt.Errorf("environment %q: robot %d has no name", env.name, i)
		}
		if seen[r.Name] {
			return fmt.Errorf("environment %q: two robots are named %q", env.name, r.Name)
		}
		seen[r.Name] = true
	}
	return nil
}

// RobotNames lists the environment's robots in rollout order.
func (e *Environment) RobotNames() []string {
	out := make([]string, 0, len(e.Robots))
	for _, r := range e.Robots {
		out = append(out, r.Name)
	}
	return out
}

// RobotByName finds a declared robot.
func (e *Environment) RobotByName(name string) (*Robot, error) {
	for _, r := range e.Robots {
		if r.Name == name {
			return r, nil
		}
	}
	if len(e.Robots) == 0 {
		return nil, fmt.Errorf("environment %q declares no robots (add a robots list to environments.json)", e.name)
	}
	return nil, fmt.Errorf("unknown robot %q in environment %q (declared: %s)",
		name, e.name, strings.Join(e.RobotNames(), ", "))
}

// Resolve returns a copy of the app as it looks in the named environment:
// externals merged, and Env set so the rest of the toolchain can report
// which environment a result belongs to. An empty name selects the declared
// default, or no environment at all when none is declared.
func (a *App) Resolve(name string) (*App, error) {
	if name == "" {
		name = a.DefaultEnv
	}
	if name == "" {
		return a, nil
	}
	env := a.Environments[name]
	if env == nil {
		if len(a.EnvNames) == 0 {
			return nil, fmt.Errorf("environment %q selected but the app declares none (add environments.json)", name)
		}
		return nil, fmt.Errorf("unknown environment %q (declared: %s)", name, strings.Join(a.EnvNames, ", "))
	}

	out := *a
	out.Env = env
	out.Externals = mergeExternals(a.Externals, env)
	if err := resolveFrames(a, env, &out); err != nil {
		return nil, err
	}
	return &out, nil
}

// ResolveRobot resolves an environment as it looks on one of its robots: the
// environment's configuration with that machine's overrides applied. The
// result is an ordinary resolved application — Env carries the merged
// settings — so deploy, check and the dashboard need no notion of a robot
// beyond choosing which one.
func (a *App) ResolveRobot(envName, robotName string) (*App, error) {
	resolved, err := a.Resolve(envName)
	if err != nil {
		return nil, err
	}
	if robotName == "" {
		return resolved, nil
	}
	if resolved.Env == nil {
		return nil, fmt.Errorf("robot %q selected but no environment is declared", robotName)
	}
	robot, err := resolved.Env.RobotByName(robotName)
	if err != nil {
		return nil, err
	}
	return resolved.ForRobot(robot)
}

// ForRobot applies one robot's overrides to an already-resolved app. Callers
// holding a resolved environment use this directly; ResolveRobot is the same
// thing from names.
func (a *App) ForRobot(robot *Robot) (*App, error) {
	env := *a.Env // a copy: resolving one robot must not disturb the next
	env.Robots = nil
	if robot.Domain != nil {
		env.Domain = robot.Domain
	}
	for _, field := range []struct {
		dst *string
		val string
	}{
		{&env.Endpoint, robot.Endpoint},
		{&env.Metrics, robot.Metrics},
		{&env.Dashboard, robot.Dashboard},
	} {
		if field.val != "" {
			*field.dst = field.val
		}
	}
	// Parameters append: a robot tunes what the environment set rather than
	// starting again, and the later file wins.
	if len(robot.Params) > 0 {
		env.Params = append(append([]string{}, env.Params...), robot.Params...)
	}
	deploy := &DeployConfig{}
	if env.Deploy != nil {
		copied := *env.Deploy
		deploy = &copied
	}
	if robot.Host != "" {
		deploy.Host = robot.Host
	}
	if robot.Prefix != "" {
		deploy.Prefix = robot.Prefix
	}
	if robot.Scope != "" {
		deploy.Scope = robot.Scope
	}
	env.Deploy = deploy

	out := *a
	out.Env = &env
	out.Robot = robot
	if robot.Frames != "" {
		if err := resolveFrames(a, &Environment{name: env.name, Frames: robot.Frames}, &out); err != nil {
			return nil, err
		}
	}
	return &out, nil
}

// Robots returns the environment's robots, or a single unnamed one standing
// for "wherever this environment deploys" — so a caller can loop either way.
func (a *App) Robots() []*Robot {
	if a.Env == nil || len(a.Env.Robots) == 0 {
		return nil
	}
	return a.Env.Robots
}

// ExternalTopics is the set of topics this application expects someone else
// to publish or consume. A running process cannot know it — the declaration
// is static — so it is what a fleet view has to be told before it can tell a
// driver's topic from one nobody is publishing.
func (a *App) ExternalTopics() map[string]bool {
	out := map[string]bool{}
	for _, e := range a.Externals {
		if e.Role == "publisher" || e.Role == "subscriber" {
			out[e.Topic] = true
		}
	}
	return out
}

// mergeExternals overlays an environment's externals on the base set. An
// external is identified by topic and role, so an environment can change the
// type or QoS a peer offers on a topic without repeating the whole list.
func mergeExternals(base []External, env *Environment) []External {
	drop := map[string]bool{}
	for _, t := range env.Without {
		drop[t] = true
	}
	out := make([]External, 0, len(base)+len(env.Externals))
	index := map[string]int{}
	for _, e := range base {
		if drop[e.Topic] {
			continue
		}
		index[e.Topic+"\x00"+e.Role] = len(out)
		out = append(out, e)
	}
	for _, e := range env.Externals {
		if i, ok := index[e.Topic+"\x00"+e.Role]; ok {
			out[i] = e
			continue
		}
		index[e.Topic+"\x00"+e.Role] = len(out)
		out = append(out, e)
	}
	return out
}

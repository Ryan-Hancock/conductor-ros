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

	// Metrics and Trace default the corresponding runtime flags.
	Metrics string `json:"metrics_addr"`
	Trace   bool   `json:"trace"`

	Deploy *DeployConfig `json:"deploy"`

	name string
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
	return nil
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

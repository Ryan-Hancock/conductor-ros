package main

import (
	"flag"
	"fmt"
	"os"
	"strings"
	"time"

	"conductor.dev/conductor/internal/deploy"
	"conductor.dev/conductor/internal/graph"
	"conductor.dev/conductor/internal/scan"
)

// runDeploy builds a release bundle for an environment and installs it on
// that environment's target. Every flag defaults to the environment's deploy
// config in environments.json, so the everyday command is just
// `conductor deploy -env robot`.
func runDeploy(args []string) error {
	dir, args := splitDir(args)
	fs := flag.NewFlagSet("deploy", flag.ExitOnError)
	env := fs.String("env", "", "environment to deploy (see environments.json)")
	robot := fs.String("robot", "", "deploy to one robot of the environment's fleet (default: all of them, in order)")
	noGate := fs.Bool("no-gate", false, "do not wait for each robot's graph to come up before the next")
	gateTimeout := fs.Duration("gate-timeout", 90*time.Second, "how long one robot has to come up before the rollout stops")
	host := fs.String("host", "", `target host (user@host), or "local" for this machine; overrides the environment`)
	goarch := fs.String("goarch", "", "target architecture (default: the environment's, else this machine's)")
	goos := fs.String("goos", "", "target OS (default linux)")
	tags := fs.String("tags", "", "comma-separated build tags")
	cc := fs.String("cc", "", "C compiler for cgo builds; overrides the environment")
	prefix := fs.String("prefix", "", "install root on the target (default /opt/conductor)")
	scope := fs.String("scope", "", "systemd scope: system (default) or user")
	sudo := fs.String("sudo", "", `privilege prefix for system scope (default "sudo -n"; "none" to disable)`)
	version := fs.String("version", "", "release version (default: UTC timestamp)")
	keep := fs.Int("keep", 0, "releases to keep on the target (default 5)")
	out := fs.String("o", "", "directory for the bundle (default <app>/gen/deploy)")
	bundle := fs.Bool("bundle", false, "build the bundle but do not ship it")
	dryRun := fs.Bool("dry-run", false, "print the commands that would run on the target")
	noRestart := fs.Bool("no-restart", false, "install without restarting the application")
	noSystemd := fs.Bool("no-systemd", false, "install files only: no units, no restart")
	rollback := fs.Bool("rollback", false, "switch the target back to its previous release")
	if err := fs.Parse(args); err != nil {
		return err
	}

	// A rollback does not build anything, and the reason for one is often
	// that the source has moved on; validating the graph here would only
	// stand between an operator and a working robot.
	var (
		app *scan.App
		g   *graph.Graph
		err error
	)
	// A fleet is checked and reported per robot inside the rollout, since
	// each robot resolves its own parameters and calibration.
	fleet := false
	if *rollback {
		app, err = resolveRobot(dir, *env, *robot)
	} else if app, err = resolve(dir, *env); err == nil && len(app.Robots()) > 0 {
		fleet = true
	} else if err == nil {
		app, g, err = checkRobot(dir, *env, *robot, true)
	}
	if err != nil {
		return err
	}
	if app.Env == nil && !*rollback {
		return fmt.Errorf("deploy needs an environment: declare one in %s/environments.json and pass -env", dir)
	}

	o := deploy.Options{
		Version:    *version,
		GOOS:       *goos,
		GOARCH:     *goarch,
		CC:         *cc,
		Host:       *host,
		Prefix:     *prefix,
		Scope:      *scope,
		Sudo:       *sudo,
		Keep:       *keep,
		OutDir:     *out,
		BundleOnly: *bundle,
		NoRestart:  *noRestart,
		NoSystemd:  *noSystemd,
		Rollback:   *rollback,
		DryRun:     *dryRun,
		Out:        os.Stdout,
	}
	if *tags != "" {
		o.Tags = strings.Split(*tags, ",")
	}
	if fleet {
		return deploy.RunFleet(app, o, deploy.FleetRollout{
			Robot:       *robot,
			NoGate:      *noGate,
			GateTimeout: *gateTimeout,
			Out:         os.Stdout,
		})
	}
	return deploy.Run(app, g, o)
}

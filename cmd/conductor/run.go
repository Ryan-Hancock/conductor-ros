package main

import (
	"flag"
	"fmt"
	"os"
	"os/exec"

	"conductor.dev/conductor/internal/run"
)

// runRun brings an environment up locally: the processes it needs, then the
// application, then everything down again.
//
// It exists because the shell version of that is in every robotics
// repository, `sleep 4` and all, and everything it does by hand is already
// declared: the transport and its endpoint, the parameter files, the
// calibration, and now the processes the environment depends on.
func runRun(args []string) error {
	dir, args := splitDir(args)
	fs := flag.NewFlagSet("run", flag.ExitOnError)
	env := fs.String("env", "", "environment to run (see environments.json)")
	robot := fs.String("robot", "", "run as one robot of the environment's fleet")
	node := fs.String("node", "", "run only this node")
	verbose := fs.Bool("v", false, "stream the required processes' output as well as the application's")
	skipCheck := fs.Bool("no-check", false, "skip graph validation")
	var with stringList
	fs.Var(&with, "with", "also run this command for the duration (repeatable)")
	if err := fs.Parse(args); err != nil {
		return err
	}

	// A wiring mistake otherwise shows up as a graph that is quietly silent,
	// which is the thing this framework exists not to do.
	if !*skipCheck {
		if _, _, err := checkRobot(dir, *env, *robot, false); err != nil {
			return fmt.Errorf("%w (run with -no-check to start anyway)", err)
		}
	}
	app, err := resolveRobot(dir, *env, *robot)
	if err != nil {
		return err
	}

	err = run.Run(app, run.Options{
		Node:    *node,
		Args:    fs.Args(),
		With:    with,
		Verbose: *verbose,
		Out:     os.Stdout,
	})
	// The application's exit status is the command's: a mission that aborts
	// should fail the script that started it.
	if exit, ok := err.(*exec.ExitError); ok {
		os.Exit(exit.ExitCode())
	}
	return err
}

package scan

import (
	"fmt"
	"path/filepath"

	"conductor.dev/conductor"
)

// The transform tree is declared in frames.json beside conductor.json and
// read by the same loader the runtime uses, so the checker and the robot
// cannot disagree about where the lidar is. An environment may name a
// different file — sensor calibration is per-robot, which is exactly what an
// environment is for.

func loadFrames(dir, name string, app *App) error {
	if name == "" {
		name = "frames.json"
	}
	path := filepath.Join(dir, name)
	tree, err := conductor.LoadFrames(path)
	if err != nil {
		return err
	}
	app.Frames = tree
	if tree != nil {
		app.FramesFile = name
	}
	return nil
}

// resolveFrames applies an environment's frames file, which must exist if it
// is named: a robot that silently falls back to the simulator's calibration
// is the failure this whole mechanism exists to prevent.
func resolveFrames(app *App, env *Environment, out *App) error {
	if env.Frames == "" {
		return nil
	}
	path := filepath.Join(app.Dir, env.Frames)
	tree, err := conductor.LoadFrames(path)
	if err != nil {
		return err
	}
	if tree == nil {
		return fmt.Errorf("environment %q names frames file %s, which does not exist", env.name, path)
	}
	out.Frames, out.FramesFile = tree, env.Frames
	return nil
}

// loadGroups reads the planning groups the same way frames are read: derived
// from the robot's SRDF by `conductor groups`, beside conductor.json, through
// the loader the runtime uses.
func loadGroups(dir, name string, app *App) error {
	if name == "" {
		name = "groups.json"
	}
	path := filepath.Join(dir, name)
	semantics, err := conductor.LoadSemantics(path)
	if err != nil {
		return err
	}
	app.Semantics = semantics
	if semantics != nil {
		app.GroupsFile = name
	}
	return nil
}

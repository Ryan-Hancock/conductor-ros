package deploy

import (
	"context"
	"fmt"
	"io"
	"os"
	"sort"
	"strings"
	"time"

	"conductor.dev/conductor"
	"conductor.dev/conductor/internal/graph"
	"conductor.dev/conductor/internal/scan"
)

// Rolling a release across a fleet is the one deployment operation where
// stopping matters more than finishing: a bad release installed on ten robots
// at once is ten robots to recover by hand. So the rollout is sequential and
// gated — the next robot is only touched once the last one's graph is up,
// and "up" is not a sleep. The runtime already reports whether every node
// reached Active, and the fleet view already reads it, so the gate has
// something real to wait for.

// FleetRollout is how a release moves across an environment's robots.
type FleetRollout struct {
	Robot       string        // roll to this robot only; empty means all of them
	NoGate      bool          // install without waiting for each robot to come up
	GateTimeout time.Duration // how long one robot has to come up (default 90s)
	Out         io.Writer
}

// RunFleet rolls a release across every robot of an environment, in the order
// they are declared, stopping at the first one that does not come up.
func RunFleet(app *scan.App, o Options, roll FleetRollout) error {
	if roll.Out == nil {
		roll.Out = os.Stdout
	}
	if o.Out == nil {
		o.Out = roll.Out
	}
	if roll.GateTimeout <= 0 {
		roll.GateTimeout = 90 * time.Second
	}
	if app.Env == nil {
		return fmt.Errorf("a fleet needs an environment: declare one in environments.json and pass -env")
	}
	robots, err := rolloutRobots(app, roll.Robot)
	if err != nil {
		return err
	}
	if o.Host != "" && len(robots) > 1 {
		return fmt.Errorf("-host names one machine but %s has %d robots; use -robot to pick one",
			app.Env.Name(), len(robots))
	}
	// One release, one version: a fleet whose robots carry different version
	// strings cannot be reasoned about afterwards.
	if o.Version == "" {
		o.Version = version()
	}

	fmt.Fprintf(roll.Out, "rolling %s %s to %d robot(s): %s\n",
		app.Name, o.Version, len(robots), strings.Join(robotNames(robots), ", "))

	for i, robot := range robots {
		on, err := app.ForRobot(robot)
		if err != nil {
			return err
		}
		if o.Host != "" {
			// -host says this robot is somewhere other than where it is
			// declared — a tunnel, an ip, or this machine for a trial run.
			// The gate has to look where the release actually went.
			on.Env.Deploy.Host = o.Host
		}
		g, issues := graph.Validate(on)
		if graph.Errors(issues) {
			// Per-robot configuration can be wrong on its own — a calibration
			// file naming a frame nothing publishes, say — and finding that
			// out here is the point of resolving each robot separately.
			for _, issue := range issues {
				if issue.Severity == graph.Error {
					fmt.Fprintf(roll.Out, "  %s\n", issue)
				}
			}
			return fmt.Errorf("robot %s: graph has errors", robot.Name)
		}

		fmt.Fprintf(roll.Out, "\n[%d/%d] %s\n", i+1, len(robots), describeRobot(robot))
		if err := Run(on, g, o); err != nil {
			return fmt.Errorf("robot %s: %w", robot.Name, err)
		}
		if o.BundleOnly || o.DryRun || roll.NoGate {
			continue
		}

		order, _ := g.BringupOrder()
		peers := Peers(on, order)
		if len(peers) == 0 {
			fmt.Fprintf(roll.Out, "  not gated: %s serves no dashboard, so there is nothing to ask "+
				"(set dashboard_addr to gate the rollout)\n", robot.Name)
			continue
		}
		fmt.Fprintf(roll.Out, "  waiting for %s to come up\n", robot.Name)
		if err := AwaitHealthy(context.Background(), peers, roll.GateTimeout, roll.Out); err != nil {
			return fmt.Errorf("robot %s did not come up, stopping the rollout (%d of %d done): %w",
				robot.Name, i, len(robots), err)
		}
		fmt.Fprintf(roll.Out, "  %s is up\n", robot.Name)
	}

	fmt.Fprintf(roll.Out, "\nrolled %s %s to %d robot(s)\n", app.Name, o.Version, len(robots))
	return nil
}

// rolloutRobots is the machines to touch, in declaration order.
func rolloutRobots(app *scan.App, only string) ([]*scan.Robot, error) {
	robots := app.Robots()
	if len(robots) == 0 {
		return nil, fmt.Errorf("environment %q declares no robots; deploy it with -env alone",
			app.Env.Name())
	}
	if only == "" {
		return robots, nil
	}
	robot, err := app.Env.RobotByName(only)
	if err != nil {
		return nil, err
	}
	return []*scan.Robot{robot}, nil
}

func robotNames(robots []*scan.Robot) []string {
	out := make([]string, len(robots))
	for i, r := range robots {
		out[i] = r.Name
	}
	return out
}

func describeRobot(r *scan.Robot) string {
	where := r.Host
	if where == "" || where == "local" {
		where = "this machine"
	}
	return fmt.Sprintf("%s (%s)", r.Name, where)
}

// AwaitHealthy waits until every process of a robot answers and every node it
// runs reports Active. It is the fleet view used as a gate: the same question
// a person would ask by opening the dashboard, asked by the rollout instead.
func AwaitHealthy(ctx context.Context, peers []conductor.Peer, timeout time.Duration, out io.Writer) error {
	deadline := time.Now().Add(timeout)
	var last []string
	for {
		state := conductor.Fleet(ctx, peers, 2*time.Second)
		problems := healthProblems(state)
		if len(problems) == 0 {
			return nil
		}
		if !sameStrings(problems, last) && out != nil {
			fmt.Fprintf(out, "    %s\n", problems[0])
			last = problems
		}
		if time.Now().After(deadline) {
			return fmt.Errorf("after %s: %s", timeout, strings.Join(problems, "; "))
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-time.After(2 * time.Second):
		}
	}
}

// healthProblems is what stands between a robot and "up".
func healthProblems(s conductor.FleetState) []string {
	var out []string
	for _, p := range s.Processes {
		if !p.OK {
			out = append(out, fmt.Sprintf("%s is not answering (%s)", p.Label, p.Err))
			continue
		}
		for _, n := range p.Nodes {
			if n.State != "active" {
				out = append(out, fmt.Sprintf("%s/%s is %s", p.Label, n.Name, n.State))
			}
		}
		if len(p.Nodes) == 0 {
			out = append(out, fmt.Sprintf("%s reports no nodes", p.Label))
		}
	}
	sort.Strings(out)
	return out
}

func sameStrings(a, b []string) bool {
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

package run

import (
	"fmt"
	"net"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"conductor.dev/conductor/internal/scan"
)

func session(t *testing.T, out *strings.Builder) *Session {
	t.Helper()
	return &Session{
		app:  &scan.App{Name: "test", Dir: t.TempDir()},
		out:  out,
		opts: Options{},
	}
}

// A required process is waited for, not slept through: readiness is a
// condition on the process, and the wait ends the moment it holds.
func TestWaitsForAPortToOpen(t *testing.T) {
	// A listener that appears after a moment, as a router does.
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	addr := ln.Addr().String()
	ln.Close()

	var out strings.Builder
	s := session(t, &out)
	defer s.stop()

	start := time.Now()
	err = s.start(scan.Process{
		Name:  "late",
		Run:   fmt.Sprintf("sleep 0.4; exec nc -l %s", strings.Replace(addr, ":", " ", 1)),
		Ready: scan.Readiness{Endpoint: "tcp/" + addr},
	})
	if err != nil {
		t.Skipf("no usable nc on this machine: %v", err)
	}
	if elapsed := time.Since(start); elapsed > 5*time.Second {
		t.Fatalf("waited %s for a port that opened after 0.4s", elapsed)
	}
	if !strings.Contains(out.String(), "waiting for late") {
		t.Errorf("output does not say what it waited for:\n%s", out.String())
	}
}

// A command probe is polled until it succeeds.
func TestWaitsForACommand(t *testing.T) {
	stamp := filepath.Join(t.TempDir(), "ready")
	var out strings.Builder
	s := session(t, &out)
	defer s.stop()

	if err := s.start(scan.Process{
		Name:  "toucher",
		Run:   fmt.Sprintf("sleep 0.3; touch %s; sleep 30", stamp),
		Ready: scan.Readiness{Command: "test -f " + stamp},
	}); err != nil {
		t.Fatal(err)
	}
	if _, err := os.Stat(stamp); err != nil {
		t.Fatalf("start returned before the condition held: %v", err)
	}
}

// A process that dies before it is ready fails the run, with what it said.
func TestReportsAProcessThatDiesEarly(t *testing.T) {
	var out strings.Builder
	s := session(t, &out)
	defer s.stop()

	err := s.start(scan.Process{
		Name:    "doomed",
		Run:     "echo 'cannot bind port'; exit 3",
		Ready:   scan.Readiness{Command: "false"},
		Timeout: "5s",
	})
	if err == nil {
		t.Fatal("a process that exited was reported as started")
	}
	if !strings.Contains(err.Error(), "doomed exited before it was ready") {
		t.Fatalf("error = %v", err)
	}
	if !strings.Contains(err.Error(), "cannot bind port") {
		t.Fatalf("error does not include what the process said: %v", err)
	}
}

// Readiness that never holds fails with the condition named, so the message
// says what was being waited for rather than "timed out".
func TestReadinessTimeout(t *testing.T) {
	var out strings.Builder
	s := session(t, &out)
	defer s.stop()

	err := s.start(scan.Process{
		Name:    "slow",
		Run:     "sleep 30",
		Ready:   scan.Readiness{Command: "false"},
		Timeout: "300ms",
	})
	if err == nil || !strings.Contains(err.Error(), "was not ready after 300ms") {
		t.Fatalf("error = %v", err)
	}
}

// Stopping kills the process group, so a launcher's children go too — this is
// what `pkill -x name` gets wrong in both directions.
func TestStopKillsTheWholeGroup(t *testing.T) {
	dir := t.TempDir()
	stamp := filepath.Join(dir, "child-alive")
	var out strings.Builder
	s := session(t, &out)

	// A shell that spawns a grandchild, as `ros2 run` does.
	if err := s.start(scan.Process{
		Name:  "launcher",
		Run:   fmt.Sprintf("(while true; do touch %s; sleep 0.1; done) & sleep 30", stamp),
		Ready: scan.Readiness{Command: "test -f " + stamp},
	}); err != nil {
		t.Fatal(err)
	}
	s.stop()

	// The grandchild stops touching the file once it is gone.
	time.Sleep(300 * time.Millisecond)
	os.Remove(stamp)
	time.Sleep(500 * time.Millisecond)
	if _, err := os.Stat(stamp); err == nil {
		t.Fatal("a grandchild of the required process survived teardown")
	}
	if !strings.Contains(out.String(), "stopped launcher") {
		t.Errorf("teardown said nothing:\n%s", out.String())
	}
}

// The flags a local run passes are the ones the environment implies — the
// same set the generated units carry, against the sources.
func TestFlagsComeFromTheEnvironment(t *testing.T) {
	domain := 4
	app := &scan.App{Name: "patrol", Dir: "/src/patrol", FramesFile: "frames.robot.json"}
	app.Env = &scan.Environment{
		Transport: "zenoh",
		Endpoint:  "tcp/10.0.0.2:7447",
		Domain:    &domain,
		Params:    []string{"params.robot.yaml"},
		Metrics:   ":9090",
		Dashboard: ":4000",
		Trace:     true,
	}
	got := strings.Join(Flags(app), " ")
	for _, want := range []string{
		"-transport zenoh",
		"-zenoh-endpoint tcp/10.0.0.2:7447",
		"-domain 4",
		"-params /src/patrol/params.robot.yaml",
		"-frames /src/patrol/frames.robot.json",
		"-metrics-addr :9090",
		"-dashboard :4000",
		"-trace",
	} {
		if !strings.Contains(got, want) {
			t.Errorf("flags %q are missing %q", got, want)
		}
	}

	// An application with no environment runs with what it was given.
	if flags := Flags(&scan.App{Name: "plain"}); len(flags) != 0 {
		t.Errorf("flags without an environment = %v", flags)
	}
}

// The endpoint an environment already declares is what the router is waited
// for on, in either spelling.
func TestDialAddr(t *testing.T) {
	for _, c := range [][2]string{
		{"tcp/127.0.0.1:7447", "127.0.0.1:7447"},
		{"127.0.0.1:7447", "127.0.0.1:7447"},
		{"tcp/[::1]:7447", "[::1]:7447"},
	} {
		if got := dialAddr(c[0]); got != c[1] {
			t.Errorf("dialAddr(%q) = %q, want %q", c[0], got, c[1])
		}
	}
}

// Ad-hoc commands from the command line join the environment's own.
func TestRequiredCombinesDeclaredAndAdHoc(t *testing.T) {
	app := &scan.App{Name: "patrol"}
	app.Env = &scan.Environment{Requires: []scan.Process{{Name: "router", Run: "rmw_zenohd"}}}
	got := required(app, []string{"ros2 run turtlesim turtlesim_node"})
	if len(got) != 2 || got[0].Name != "router" || got[1].Run != "ros2 run turtlesim turtlesim_node" {
		t.Fatalf("required = %+v", got)
	}
}

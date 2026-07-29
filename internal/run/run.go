// Package run brings an environment up locally: the processes it depends on,
// then the application itself, then everything down again.
//
// The shell version of this is familiar — start the router, sleep 2, start
// the simulator, sleep 4, run the app, pkill everything by name — and it is
// wrong in three ways that matter. The sleeps are guesses, so it is flaky on
// a loaded machine. `pkill -x turtlesim_node` kills a colleague's simulator
// as happily as its own. And nothing tears down if the app exits badly.
//
// Conductor already knows the environment: its transport, its parameter
// files, its calibration, and now the processes it needs. So it can start
// them, wait for a condition instead of a duration, run the application with
// the flags that environment implies, and stop what it started — by pid.
package run

import (
	"bufio"
	"fmt"
	"io"
	"net"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"

	"conductor.dev/conductor/internal/scan"
)

// Options is one local run.
type Options struct {
	Node    string   // run only this node, like the -node flag
	Args    []string // extra flags passed through to the application
	With    []string // ad-hoc required commands, on top of the environment's
	Verbose bool     // stream the required processes' output
	Build   bool     // go build once instead of go run (faster restarts)
	Out     io.Writer
}

// Session is a running environment: the processes started for it, in order.
type Session struct {
	opts    Options
	app     *scan.App
	out     io.Writer
	started []*process
}

// Run starts the environment's required processes, runs the application, and
// stops everything on the way out. It returns the application's exit error.
func Run(app *scan.App, o Options) error {
	if o.Out == nil {
		o.Out = os.Stdout
	}
	s := &Session{opts: o, app: app, out: o.Out}

	// Hold SIGPIPE for the whole session, not just while the application is
	// running. Without this, a broken stdout — `conductor run | head` — kills
	// the supervisor mid-teardown and leaves a router behind, which is
	// precisely the failure this command exists to remove.
	pipes := make(chan os.Signal, 1)
	signal.Notify(pipes, syscall.SIGPIPE)
	defer signal.Stop(pipes)

	defer s.stop()

	for _, p := range required(app, o.With) {
		if err := s.start(p); err != nil {
			return err
		}
	}
	return s.runApp()
}

// required is the environment's processes plus any given on the command line.
func required(app *scan.App, with []string) []scan.Process {
	var out []scan.Process
	if app.Env != nil {
		out = append(out, app.Env.Requires...)
	}
	for i, cmd := range with {
		out = append(out, scan.Process{Name: fmt.Sprintf("with%d", i+1), Run: cmd})
	}
	return out
}

// process is one started dependency. A single goroutine waits on it, so both
// readiness and teardown can ask whether it is still alive without racing
// each other for its exit status.
type process struct {
	spec scan.Process
	cmd  *exec.Cmd
	log  *tail

	done chan struct{} // closed once the process has been reaped
	err  error         // its exit error, set before done is closed
}

// gone reports whether the process has exited, and why.
func (p *process) gone() (bool, error) {
	select {
	case <-p.done:
		return true, p.err
	default:
		return false, nil
	}
}

func (s *Session) start(spec scan.Process) error {
	if spec.Run == "" {
		return fmt.Errorf("required process %q has nothing to run", spec.Name)
	}
	name := spec.Name
	if name == "" {
		name = firstWord(spec.Run)
	}

	cmd := exec.Command("sh", "-c", spec.Run)
	cmd.Dir = s.app.Dir
	if spec.Dir != "" {
		cmd.Dir = filepath.Join(s.app.Dir, spec.Dir)
	}
	cmd.Env = os.Environ()
	for k, v := range spec.Env {
		cmd.Env = append(cmd.Env, k+"="+v)
	}
	// Its own process group, so stopping it stops whatever it spawned —
	// `ros2 run` is a launcher, and the node is its child. Pdeathsig is the
	// backstop for the case teardown cannot cover: if this supervisor is
	// killed outright, the kernel still tells its children.
	cmd.SysProcAttr = &syscall.SysProcAttr{Setpgid: true, Pdeathsig: syscall.SIGTERM}

	p := &process{spec: spec, cmd: cmd, log: newTail(40), done: make(chan struct{})}
	sink := io.Writer(p.log)
	if s.opts.Verbose {
		sink = io.MultiWriter(p.log, prefixed(s.out, name))
	}
	cmd.Stdout, cmd.Stderr = sink, sink

	if err := cmd.Start(); err != nil {
		return fmt.Errorf("starting %s: %w", name, err)
	}
	s.started = append(s.started, p)
	go func() { p.err = cmd.Wait(); close(p.done) }()
	fmt.Fprintf(s.out, "conductor: started %s (pid %d)\n", name, cmd.Process.Pid)

	if err := s.await(p, name); err != nil {
		return err
	}
	return nil
}

// await blocks until the process reports ready, it dies, or time runs out.
func (s *Session) await(p *process, name string) error {
	if !p.spec.Ready.Declared() {
		return nil
	}
	timeout := 30 * time.Second
	if p.spec.Timeout != "" {
		d, err := time.ParseDuration(p.spec.Timeout)
		if err != nil {
			return fmt.Errorf("%s: invalid ready_timeout %q", name, p.spec.Timeout)
		}
		timeout = d
	}

	if d := p.spec.Ready.Delay; d != "" {
		wait, err := time.ParseDuration(d)
		if err != nil {
			return fmt.Errorf("%s: invalid ready delay %q", name, d)
		}
		fmt.Fprintf(s.out, "conductor: waiting %s for %s\n", d, name)
		time.Sleep(wait)
		return nil
	}

	what, check := probe(p.spec.Ready)
	fmt.Fprintf(s.out, "conductor: waiting for %s (%s)\n", name, what)
	deadline := time.Now().Add(timeout)
	for {
		if check() {
			return nil
		}
		if exited, err := p.gone(); exited {
			return fmt.Errorf("%s exited before it was ready (%v)\n%s", name, err, p.log.String())
		}
		if time.Now().After(deadline) {
			return fmt.Errorf("%s was not ready after %s (%s)\n%s", name, timeout, what, p.log.String())
		}
		time.Sleep(150 * time.Millisecond)
	}
}

// probe turns a readiness declaration into something to poll, and a phrase
// for the log.
func probe(r scan.Readiness) (string, func() bool) {
	if r.Endpoint != "" {
		addr := dialAddr(r.Endpoint)
		return "listening on " + addr, func() bool {
			conn, err := net.DialTimeout("tcp", addr, time.Second)
			if err != nil {
				return false
			}
			conn.Close()
			return true
		}
	}
	cmd := r.Command
	return cmd, func() bool {
		c := exec.Command("sh", "-c", cmd)
		c.Stdout, c.Stderr = io.Discard, io.Discard
		return c.Run() == nil
	}
}

// dialAddr accepts a zenoh endpoint ("tcp/127.0.0.1:7447") as well as a plain
// host:port, because the endpoint the environment already declares is the
// obvious thing to wait for.
func dialAddr(endpoint string) string {
	addr := endpoint
	if i := strings.Index(addr, "/"); i >= 0 && !strings.Contains(addr[:i], ":") {
		addr = addr[i+1:]
	}
	return addr
}

// runApp runs the application with the flags its environment implies, and
// forwards an interrupt to it so its lifecycle shutdown runs.
func (s *Session) runApp() error {
	rel, err := filepath.Rel(s.app.ModuleRoot, s.app.Dir)
	if err != nil {
		return err
	}
	pkg := "./" + filepath.ToSlash(rel)

	args := []string{"run"}
	if tags := buildTags(s.app); tags != "" {
		args = append(args, "-tags", tags)
	}
	args = append(args, pkg)
	flags := Flags(s.app)
	if s.opts.Node != "" {
		flags = append(flags, "-node", s.opts.Node)
	}
	flags = append(flags, s.opts.Args...)
	args = append(args, flags...)

	fmt.Fprintf(s.out, "conductor: running %s%s\n", s.app.Name, envSuffix(s.app))
	fmt.Fprintf(s.out, "conductor: go %s\n\n", strings.Join(args, " "))

	cmd := exec.Command("go", args...)
	cmd.Dir = s.app.ModuleRoot
	cmd.Stdout, cmd.Stderr, cmd.Stdin = os.Stdout, os.Stderr, os.Stdin
	cmd.SysProcAttr = &syscall.SysProcAttr{Setpgid: true}
	if err := cmd.Start(); err != nil {
		return err
	}

	// Ctrl-C goes to the application, not to us: the runtime installs a
	// handler, and the lifecycle teardown it triggers is the whole reason a
	// conductor app is worth stopping politely.
	//
	// SIGPIPE and SIGHUP are here because of how this gets used: piping the
	// output into `head`, or closing the terminal, would otherwise kill the
	// supervisor before it could stop anything, and leave a router running.
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM, syscall.SIGHUP)
	defer signal.Stop(sig)
	done := make(chan error, 1)
	go func() { done <- cmd.Wait() }()

	interrupted := false
	for {
		select {
		case received := <-sig:
			interrupted = true
			switch received {
			case syscall.SIGHUP:
				// Nobody is reading any more; stop the application and let
				// the deferred teardown do the rest.
				syscall.Kill(-cmd.Process.Pid, syscall.SIGINT)
				select {
				case <-done:
				case <-time.After(3 * time.Second):
					syscall.Kill(-cmd.Process.Pid, syscall.SIGKILL)
				}
				return nil
			default:
				syscall.Kill(-cmd.Process.Pid, received.(syscall.Signal))
			}
		case err := <-done:
			if interrupted {
				// We asked it to stop, and it did: that is success. An
				// application that fails on its own still reports it.
				return nil
			}
			return err
		}
	}
}

// Flags are the runtime flags this environment implies, against the sources
// rather than an installed release. They are the same flags the generated
// units carry, which is the point: what a developer runs locally and what
// systemd runs on the robot differ in paths, not in behaviour.
func Flags(app *scan.App) []string {
	var flags []string
	env := app.Env
	if env == nil {
		return flags
	}
	if env.Transport != "" {
		flags = append(flags, "-transport", env.Transport)
	}
	if env.Endpoint != "" {
		flags = append(flags, "-zenoh-endpoint", env.Endpoint)
	}
	if env.Domain != nil {
		flags = append(flags, "-domain", strconv.Itoa(*env.Domain))
	}
	for _, pf := range env.Params {
		flags = append(flags, "-params", filepath.Join(app.Dir, pf))
	}
	if app.FramesFile != "" {
		flags = append(flags, "-frames", filepath.Join(app.Dir, app.FramesFile))
	}
	if env.Metrics != "" {
		flags = append(flags, "-metrics-addr", env.Metrics)
	}
	if env.Dashboard != "" {
		flags = append(flags, "-dashboard", env.Dashboard)
	}
	if env.Trace {
		flags = append(flags, "-trace")
	}
	return flags
}

func buildTags(app *scan.App) string {
	if app.Env == nil || app.Env.Deploy == nil {
		return ""
	}
	return strings.Join(app.Env.Deploy.Tags, ",")
}

func envSuffix(app *scan.App) string {
	if app.Env == nil {
		return ""
	}
	if app.Robot != nil {
		return fmt.Sprintf(" [env %s, robot %s]", app.Env.Name(), app.Robot.Name)
	}
	return fmt.Sprintf(" [env %s]", app.Env.Name())
}

// stop ends what this session started, youngest first, politely and then not.
// Killing the process group is what makes this different from `pkill` by
// name: it stops the simulator this run started, and not the one somebody
// else is using.
func (s *Session) stop() {
	for i := len(s.started) - 1; i >= 0; i-- {
		p := s.started[i]
		if p.cmd.Process == nil {
			continue
		}
		name := p.spec.Name
		if name == "" {
			name = firstWord(p.spec.Run)
		}
		pgid := p.cmd.Process.Pid
		syscall.Kill(-pgid, syscall.SIGINT)
		select {
		case <-p.done:
		case <-time.After(3 * time.Second):
			syscall.Kill(-pgid, syscall.SIGKILL)
			<-p.done
		}
		fmt.Fprintf(s.out, "conductor: stopped %s\n", name)
	}
}

// tail keeps the last lines of a process's output, so a failure can show what
// it said without the successful case printing anything at all.
type tail struct {
	mu    sync.Mutex
	lines []string
	max   int
	buf   []byte
}

func newTail(max int) *tail { return &tail{max: max} }

func (t *tail) Write(b []byte) (int, error) {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.buf = append(t.buf, b...)
	for {
		i := strings.IndexByte(string(t.buf), '\n')
		if i < 0 {
			break
		}
		t.lines = append(t.lines, string(t.buf[:i]))
		t.buf = t.buf[i+1:]
		if len(t.lines) > t.max {
			t.lines = t.lines[1:]
		}
	}
	return len(b), nil
}

func (t *tail) String() string {
	t.mu.Lock()
	defer t.mu.Unlock()
	var b strings.Builder
	for _, l := range t.lines {
		b.WriteString("    " + l + "\n")
	}
	return b.String()
}

// prefixed labels a process's output with its name.
func prefixed(w io.Writer, name string) io.Writer {
	r, pw := io.Pipe()
	go func() {
		sc := bufio.NewScanner(r)
		for sc.Scan() {
			fmt.Fprintf(w, "[%s] %s\n", name, sc.Text())
		}
	}()
	return pw
}

func firstWord(s string) string {
	if i := strings.IndexAny(s, " \t"); i > 0 {
		return filepath.Base(s[:i])
	}
	return filepath.Base(s)
}

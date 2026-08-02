package conductor

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"reflect"
	"sync"
	"sync/atomic"
	"time"
)

// Task orchestration in ROS 2 is either a hand-rolled state machine of
// booleans and timers, or a behaviour tree in an XML file that no compiler
// ever reads. Conductor takes the same line it takes with topics: the machine
// is a declaration.
//
//	//conductor:node
//	type Courier struct {
//	    Nav  conductor.ActionClient[NavGoal, NavFeedback, NavResult] `action:"navigate_to_pose"`
//	    Trip conductor.Mission `start:"pickup"`
//
//	    Pickup   conductor.Step `next:"transit"`
//	    Transit  conductor.Step `next:"dropoff" fail:"recharge" timeout:"2m"`
//	    Dropoff  conductor.Step `next:"done"`
//	    Recharge conductor.Step `next:"transit" retry:"2"`
//	}
//
//	func (c *Courier) OnTransit(t *conductor.Task) error { ... }
//
// Because the transitions are tags rather than control flow, `conductor
// check` reads them: a next/fail target that is not a step is a build error,
// an unreachable step is a warning, and the machine is drawn in mission.dot
// beside the topic graph. Because the runtime drives it, the current step is
// a metric, a span, and a panel on the dashboard, with no user code.

// MissionStatus is where a mission has got to.
type MissionStatus string

const (
	MissionIdle     MissionStatus = "idle"     // not started: the node is not Active
	MissionRunning  MissionStatus = "running"  // a step is executing
	MissionDone     MissionStatus = "done"     // reached the terminal step "done"
	MissionFailed   MissionStatus = "failed"   // a step failed with nowhere to go
	MissionCanceled MissionStatus = "canceled" // deactivated, or Cancel was called
)

// The two terminal step names. They need no Step field: "done" ends the
// mission successfully, "failed" ends it as a failure.
const (
	StepDone   = "done"
	StepFailed = "failed"
)

// Mission declares a task state machine on a node: the steps are the node's
// Step fields, and this is the handle on the machine that runs them.
//
// The mission starts when the node becomes Active and is canceled when it
// leaves Active, so a mission is subject to the same lifecycle as everything
// else the node does. Re-activating restarts it from the start step.
//
// Tags: start (required) — the step to begin at.
type Mission struct {
	runner *missionRunner
}

// Step declares one step of the owning node's mission. The node must define a
// handler method named On<FieldName> with signature func(*conductor.Task)
// error.
//
// Step handlers run on their own goroutine — not the node's executor —
// because a step is long-running by nature: it sends a navigation goal and
// waits for the result. They must therefore not touch node state that
// executor callbacks also use; Task.Do exists for the cases that need to.
//
// Tags:
//
//	next     step to enter when the handler returns nil (default "done")
//	fail     step to enter when it returns an error (default: the mission fails)
//	timeout  deadline for the step, as a duration; cancels the Task's context
//	retry    times to re-run the step on error before taking fail
//	backoff  delay between retries (default none)
type Step struct {
	def *stepDef
}

// Task is the handle passed to a step handler: the step's context, its
// attempt number, and the way to choose the next step dynamically.
type Task struct {
	ctx     context.Context
	runner  *missionRunner
	step    string
	attempt int
	err     error
}

// Context is canceled when the step's timeout expires, when the mission is
// canceled, or when the node leaves Active. A step that blocks must select on
// it; that is how deactivating a node actually stops the work in flight.
func (t *Task) Context() context.Context { return t.ctx }

// Step returns the name of the running step.
func (t *Task) Step() string { return t.step }

// Attempt returns the 1-based attempt number for this step (see the retry tag).
func (t *Task) Attempt() int { return t.attempt }

// Mission returns the mission's name.
func (t *Task) Mission() string { return t.runner.name }

// Err returns the error that sent the mission here, when this step was
// entered through another step's fail branch — so a recovery step can act on
// what actually went wrong. It is nil on any other path.
func (t *Task) Err() error { return t.err }

// Goto selects the next step explicitly, overriding the next/fail tags:
//
//	if c.battery.Low() {
//	    return t.Goto("recharge")
//	}
//
// The returned value is an error only so that it reads as a return from the
// handler; it does not mean the step failed. Targets written as string
// literals are checked by `conductor check` like the tags are.
func (t *Task) Goto(step string) error { return gotoStep{step: step} }

// Sleep waits for d, or until the step's context is canceled — the form of
// sleep a step should use, since it can be interrupted by a deactivation.
func (t *Task) Sleep(d time.Duration) error {
	select {
	case <-t.runner.clock().After(d):
		return nil
	case <-t.ctx.Done():
		return t.stopped()
	}
}

// stopped is why the step's context ended: the step's own timeout, or the
// mission being canceled. context.Cause carries the distinction that ctx.Err
// flattens into "canceled", and a fail branch wants to know which it was.
func (t *Task) stopped() error {
	if cause := context.Cause(t.ctx); cause != nil {
		return cause
	}
	return t.ctx.Err()
}

// Do runs fn on the node's executor and waits for it to finish, so a step can
// read or write the node state that callbacks own without a mutex. Returns
// the step context's error if the node stops first.
func (t *Task) Do(fn func()) error {
	done := make(chan struct{})
	if !t.runner.node.enqueue(func() { fn(); close(done) }) {
		return fmt.Errorf("node %s is not accepting work", t.runner.node.name)
	}
	select {
	case <-done:
		return nil
	case <-t.ctx.Done():
		return t.stopped()
	case <-t.runner.node.quit:
		return fmt.Errorf("node %s shut down", t.runner.node.name)
	}
}

// gotoStep is the sentinel Task.Goto returns.
type gotoStep struct{ step string }

func (g gotoStep) Error() string { return "goto " + g.step }

// stepDef is one declared step.
type stepDef struct {
	name    string
	field   string
	next    string
	fail    string
	timeout time.Duration
	retry   int
	backoff time.Duration
	handler func(*Task) error
	entries atomic.Uint64
}

func (s *Step) bind(rt *runtimeState, nr *nodeRuntime, field reflect.StructField, ownerPtr reflect.Value) error {
	m := ownerPtr.MethodByName("On" + field.Name)
	if !m.IsValid() {
		return fmt.Errorf("missing handler method On%s", field.Name)
	}
	h, ok := m.Interface().(func(*Task) error)
	if !ok {
		return fmt.Errorf("On%s must have signature func(*conductor.Task) error", field.Name)
	}
	def := &stepDef{
		name:    snakeCase(field.Name),
		field:   field.Name,
		next:    field.Tag.Get("next"),
		fail:    field.Tag.Get("fail"),
		handler: h,
	}
	if v := field.Tag.Get("timeout"); v != "" {
		d, err := time.ParseDuration(v)
		if err != nil || d <= 0 {
			return fmt.Errorf("invalid timeout %q", v)
		}
		def.timeout = d
	}
	if v := field.Tag.Get("backoff"); v != "" {
		d, err := time.ParseDuration(v)
		if err != nil || d < 0 {
			return fmt.Errorf("invalid backoff %q", v)
		}
		def.backoff = d
	}
	if v := field.Tag.Get("retry"); v != "" {
		n, err := parseCount(v)
		if err != nil {
			return fmt.Errorf("invalid retry %q", v)
		}
		def.retry = n
	}
	if def.next == "" {
		def.next = StepDone
	}
	s.def = def
	nr.steps = append(nr.steps, def)
	return nil
}

func parseCount(s string) (int, error) {
	n := 0
	for _, r := range s {
		if r < '0' || r > '9' {
			return 0, fmt.Errorf("not a count")
		}
		n = n*10 + int(r-'0')
	}
	if len(s) == 0 {
		return 0, fmt.Errorf("not a count")
	}
	return n, nil
}

func (m *Mission) bind(rt *runtimeState, nr *nodeRuntime, field reflect.StructField, ownerPtr reflect.Value) error {
	start := field.Tag.Get("start")
	if start == "" {
		return errors.New(`missing start tag (e.g. start:"pickup")`)
	}
	if nr.mission != nil {
		return fmt.Errorf("node %s already declares mission %s; a node runs one mission", nr.name, nr.mission.name)
	}
	r := &missionRunner{
		node:  nr,
		name:  snakeCase(field.Name),
		field: field.Name,
		start: start,
		state: missionState{Status: MissionIdle},
	}
	m.runner = r
	nr.mission = r
	return nil
}

// Status reports where the mission has got to.
func (m *Mission) Status() MissionStatus {
	if m.runner == nil {
		return MissionIdle
	}
	return m.runner.snapshot().Status
}

// Step returns the step currently running, or the last one to run.
func (m *Mission) Step() string {
	if m.runner == nil {
		return ""
	}
	return m.runner.snapshot().Step
}

// Err returns the error that failed the mission, if it failed.
func (m *Mission) Err() error {
	if m.runner == nil {
		return nil
	}
	return m.runner.snapshot().Err
}

// Cancel stops the mission: the running step's context is canceled and no
// further step is entered. Activating the node again restarts it.
func (m *Mission) Cancel() {
	if m.runner != nil {
		m.runner.stop(2 * time.Second)
	}
}

// Restart cancels the mission if it is running and starts it again from the
// start step.
func (m *Mission) Restart() {
	if m.runner != nil {
		m.runner.stop(2 * time.Second)
		m.runner.start_()
	}
}

// Done returns a channel closed when the mission reaches a terminal state. It
// is nil before the mission first starts.
func (m *Mission) Done() <-chan struct{} {
	if m.runner == nil {
		return nil
	}
	m.runner.mu.Lock()
	defer m.runner.mu.Unlock()
	return m.runner.done
}

// missionState is a consistent read of a running mission.
type missionState struct {
	Status  MissionStatus
	Step    string
	Attempt int
	Since   time.Time
	Err     error
}

// missionRunner drives one node's mission on its own goroutine.
type missionRunner struct {
	node  *nodeRuntime
	name  string
	field string
	start string

	steps map[string]*stepDef
	order []*stepDef

	// clocks is where the mission reads time: a step's timeout, its backoff and
	// Task.Sleep are all the robot's time, not the machine's, so a mission runs
	// at the pace of the world it is in.
	clocks Clock

	mu     sync.Mutex
	state  missionState
	cancel context.CancelFunc
	done   chan struct{}

	transitions atomic.Uint64
}

func (r *missionRunner) snapshot() missionState {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.state
}

// wireMission assembles the node's steps into the machine and hangs it off
// the lifecycle. It runs after every field is bound, because the Mission
// field and its Steps are declared in whatever order reads best.
func wireMission(rt *runtimeState, nr *nodeRuntime) error {
	if nr.mission == nil {
		if len(nr.steps) > 0 {
			return fmt.Errorf("node %s declares step %s but no conductor.Mission field", nr.name, nr.steps[0].field)
		}
		return nil
	}
	r := nr.mission
	r.clocks = rt.clock
	r.steps = map[string]*stepDef{}
	for _, def := range nr.steps {
		if _, dup := r.steps[def.name]; dup {
			return fmt.Errorf("mission %s: two steps named %q", r.name, def.name)
		}
		r.steps[def.name] = def
		r.order = append(r.order, def)
	}
	if len(r.order) == 0 {
		return fmt.Errorf("mission %s declares no steps", r.name)
	}
	if _, ok := r.steps[r.start]; !ok {
		return fmt.Errorf("mission %s starts at %q, which is not a declared step", r.name, r.start)
	}
	for _, def := range r.order {
		for _, target := range []string{def.next, def.fail} {
			if target == "" || target == StepDone || target == StepFailed {
				continue
			}
			if _, ok := r.steps[target]; !ok {
				return fmt.Errorf("mission %s: step %s names %q, which is not a declared step", r.name, def.name, target)
			}
		}
	}
	r.state.Step = r.start

	nr.onActive = append(nr.onActive, r.start_)
	nr.onInactive = append(nr.onInactive, func() { r.stop(5 * time.Second) })
	rt.recordEndpoint(Endpoint{
		Node: nr.name, Kind: EndpointMission, Field: r.field, Name: r.name,
		count: countOf(r.transitions.Load),
	})
	return nil
}

// start_ begins the mission, unless it is already running. (Named with a
// trailing underscore because start is the tag and the field.)
func (r *missionRunner) start_() {
	r.mu.Lock()
	if r.state.Status == MissionRunning {
		r.mu.Unlock()
		return
	}
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan struct{})
	r.cancel, r.done = cancel, done
	r.state = missionState{Status: MissionRunning, Step: r.start, Attempt: 1, Since: time.Now()}
	r.mu.Unlock()

	gauge("conductor_mission_running", "node", r.node.name, "mission", r.name).Store(1)
	slog.Info("conductor: mission started", "node", r.node.name, "mission", r.name, "step", r.start)
	go r.run(ctx, done)
}

// stop cancels the mission and waits up to grace for the running step to
// return. A step that ignores its context cannot be stopped, so the wait is
// bounded and says so.
func (r *missionRunner) stop(grace time.Duration) {
	r.mu.Lock()
	cancel, done := r.cancel, r.done
	r.mu.Unlock()
	if cancel == nil {
		return
	}
	cancel()
	if done == nil {
		return
	}
	select {
	case <-done:
	case <-time.After(grace):
		slog.Warn("conductor: mission step did not stop when canceled",
			"node", r.node.name, "mission", r.name, "step", r.snapshot().Step)
	}
}

func (r *missionRunner) run(ctx context.Context, done chan struct{}) {
	defer close(done)

	step, attempt := r.start, 1
	var cause error // the error that took a fail branch into this step
	for {
		if ctx.Err() != nil {
			r.finish(MissionCanceled, nil)
			return
		}
		switch step {
		case StepDone:
			r.finish(MissionDone, nil)
			return
		case StepFailed:
			r.finish(MissionFailed, r.snapshot().Err)
			return
		}
		def, ok := r.steps[step]
		if !ok {
			r.finish(MissionFailed, fmt.Errorf("no step %q in mission %s", step, r.name))
			return
		}

		r.enter(step, attempt)
		err := r.runStep(ctx, def, attempt, cause)
		cause = nil

		var jump gotoStep
		switch {
		case errors.As(err, &jump):
			if _, ok := r.steps[jump.step]; !ok && jump.step != StepDone && jump.step != StepFailed {
				r.finish(MissionFailed, fmt.Errorf("step %s: Goto(%q): no such step", def.name, jump.step))
				return
			}
			step, attempt = jump.step, 1
		case err == nil:
			step, attempt = def.next, 1
		case ctx.Err() != nil:
			r.finish(MissionCanceled, nil)
			return
		case attempt <= def.retry:
			slog.Warn("conductor: mission step failed, retrying",
				"node", r.node.name, "mission", r.name, "step", def.name, "attempt", attempt, "err", err)
			counter("conductor_mission_step_retries_total", "node", r.node.name, "mission", r.name, "step", def.name).Add(1)
			attempt++
			if def.backoff > 0 && !sleepCtx(r.clock(), ctx, def.backoff) {
				r.finish(MissionCanceled, nil)
				return
			}
		case def.fail != "":
			slog.Warn("conductor: mission step failed, taking the fail branch",
				"node", r.node.name, "mission", r.name, "step", def.name, "fail", def.fail, "err", err)
			r.setErr(err)
			step, attempt, cause = def.fail, 1, err
		default:
			r.finish(MissionFailed, fmt.Errorf("step %s: %w", def.name, err))
			return
		}
	}
}

// runStep executes one step with its own context, span and metrics.
func (r *missionRunner) runStep(ctx context.Context, def *stepDef, attempt int, cause error) error {
	// A step's timeout is measured on the robot's clock, not the machine's: a
	// simulation running at a tenth of real speed should give a step the same
	// simulated seconds it would have on the robot. context.WithTimeout cannot
	// do that, so the deadline is a waiter on the clock — cancelling with
	// DeadlineExceeded as the cause, so a step still sees why it was stopped.
	sctx, cancelCause := context.WithCancelCause(ctx)
	cancel := func() { cancelCause(nil) }
	if def.timeout > 0 {
		expired := r.clock().After(def.timeout)
		go func() {
			select {
			case <-expired:
				cancelCause(context.DeadlineExceeded)
			case <-sctx.Done():
			}
		}()
	}
	defer cancel()

	t := &Task{ctx: sctx, runner: r, step: def.name, attempt: attempt, err: cause}
	var span *Span
	if tracingEnabled() {
		// A step roots its own trace: the mission runs off the executor, so
		// there is no callback context to continue.
		span = startSpan(r.node.name, SpanStep, r.name+"/"+def.name, TraceContext{})
	}
	start := time.Now()
	err := def.handler(t)
	// A clock-driven deadline cancels the context, so ctx.Err() is Canceled and
	// the reason lives in the cause. A step that simply returned its context's
	// error should still send "timed out" to the fail branch, which is what it
	// meant and what the tag promised.
	if cause := context.Cause(sctx); errors.Is(cause, context.DeadlineExceeded) && errors.Is(err, context.Canceled) {
		err = cause
	}
	var jump gotoStep
	if errors.As(err, &jump) {
		span.finish(nil)
	} else {
		span.finish(err)
	}
	observe("conductor_mission_step_duration", time.Since(start),
		"node", r.node.name, "mission", r.name, "step", def.name)
	return err
}

func (r *missionRunner) enter(step string, attempt int) {
	def := r.steps[step]
	if def != nil {
		def.entries.Add(1)
	}
	r.transitions.Add(1)
	r.mu.Lock()
	r.state.Step, r.state.Attempt, r.state.Since = step, attempt, time.Now()
	r.mu.Unlock()
	counter("conductor_mission_step_entries_total", "node", r.node.name, "mission", r.name, "step", step).Add(1)
	slog.Debug("conductor: mission step", "node", r.node.name, "mission", r.name, "step", step, "attempt", attempt)
}

// clock is the mission's time source, defaulting to the wall clock for a
// runner built outside Run (tests construct these directly).
func (r *missionRunner) clock() Clock {
	if r.clocks == nil {
		return wallClock{}
	}
	return r.clocks
}

func (r *missionRunner) setErr(err error) {
	r.mu.Lock()
	r.state.Err = err
	r.mu.Unlock()
}

func (r *missionRunner) finish(status MissionStatus, err error) {
	r.mu.Lock()
	r.state.Status = status
	if err != nil {
		r.state.Err = err
	}
	switch status {
	case MissionDone:
		r.state.Step = StepDone
	case MissionFailed:
		r.state.Step = StepFailed
	}
	r.state.Since = time.Now()
	r.mu.Unlock()

	gauge("conductor_mission_running", "node", r.node.name, "mission", r.name).Store(0)
	counter("conductor_mission_completions_total", "node", r.node.name, "mission", r.name, "result", string(status)).Add(1)
	switch status {
	case MissionFailed:
		slog.Error("conductor: mission failed", "node", r.node.name, "mission", r.name, "err", err)
	case MissionDone:
		slog.Info("conductor: mission complete", "node", r.node.name, "mission", r.name)
	default:
		slog.Info("conductor: mission stopped", "node", r.node.name, "mission", r.name, "status", string(status))
	}
}

func sleepCtx(clock Clock, ctx context.Context, d time.Duration) bool {
	select {
	case <-clock.After(d):
		return true
	case <-ctx.Done():
		return false
	}
}

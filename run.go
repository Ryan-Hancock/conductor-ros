package conductor

import (
	"context"
	"flag"
	"fmt"
	"log/slog"
	"net/http"
	"os"
	"os/signal"
	"path/filepath"
	"reflect"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"syscall"
	"time"
	"unicode"
)

type runtimeState struct {
	transport   Transport
	nodes       []*nodeRuntime
	timers      []*timerHandle
	deps        map[string]*nodeDeps
	endpoints   []Endpoint
	paramValues map[string]map[string]string // node name -> parameter -> raw value
}

// binder is implemented by the framework field types (Sub, Pub, Param, Timer);
// Run wires each exported field that implements it.
type binder interface {
	bind(rt *runtimeState, nr *nodeRuntime, field reflect.StructField, ownerPtr reflect.Value) error
}

// Run wires the given nodes (pointers to structs with conductor field
// declarations) and blocks until SIGINT or SIGTERM.
//
// Flags parsed from os.Args:
//
//	-node <name>            run only the named node (used by generated launch files)
//	-transport <name>       message transport: inproc (default) or zenoh
//	-zenoh-endpoint <ep>    zenoh router endpoint (default tcp/127.0.0.1:7447)
//	-domain <id>            ROS domain id (default $ROS_DOMAIN_ID or 0)
//	-lifecycle <mode>       auto (default: configure and activate nodes in
//	                        dependency order at startup) or manual (leave
//	                        them unconfigured for an external orchestrator)
func Run(nodes ...any) {
	fs := flag.NewFlagSet("conductor", flag.ExitOnError)
	only := fs.String("node", "", "run only the named node")
	transportName := fs.String("transport", "inproc", "message transport (inproc, zenoh)")
	endpoint := fs.String("zenoh-endpoint", "tcp/127.0.0.1:7447", "zenoh router endpoint")
	domain := fs.Int("domain", envDomain(), "ROS domain id")
	lifecycleMode := fs.String("lifecycle", "auto", "lifecycle mode (auto, manual)")
	metricsAddr := fs.String("metrics-addr", "", "expose Prometheus metrics on this address (e.g. :9090)")
	traceLog := fs.Bool("trace", false, "log a span for every callback, with W3C trace context")
	dashAddr := fs.String("dashboard", "", "serve the live dashboard on this address (e.g. :4000)")
	dashTraces := fs.Int("dashboard-traces", 0, "record this many recent spans for the dashboard's trace view (implies tracing)")
	var paramFiles stringList
	fs.Var(&paramFiles, "params", "parameter file to load (repeatable; later files win)")
	env := fs.String("env", "", "environment name: also loads params.<env>.yaml next to the last -params file, or ./params.<env>.yaml")
	fs.Parse(os.Args[1:])

	files, err := resolveParamFiles(paramFiles, *env)
	if err != nil {
		slog.Error("conductor: parameter files", "err", err)
		os.Exit(1)
	}
	values, err := LoadParamFiles(files...)
	if err != nil {
		slog.Error("conductor: loading parameters", "err", err)
		os.Exit(1)
	}
	if len(files) > 0 {
		slog.Info("conductor: loaded parameters", "files", strings.Join(files, ","), "env", *env)
	}

	if *traceLog {
		AddExporter(LogExporter{})
	}

	if *lifecycleMode != "auto" && *lifecycleMode != "manual" {
		slog.Error("conductor: invalid -lifecycle mode (want auto or manual)", "mode", *lifecycleMode)
		os.Exit(1)
	}

	a, err := newAppWithParams(*transportName, TransportOptions{Endpoint: *endpoint, Domain: *domain}, *only, values, nodes...)
	if err != nil {
		slog.Error("conductor: startup failed", "err", err)
		os.Exit(1)
	}
	if *lifecycleMode == "auto" {
		if err := a.bringUp(); err != nil {
			slog.Error("conductor: bringup failed", "err", err)
			a.stop()
			os.Exit(1)
		}
	} else {
		slog.Info("conductor: lifecycle manual, nodes are unconfigured until told otherwise")
	}
	if *metricsAddr != "" {
		a.metrics = serveMetrics(*metricsAddr)
	}
	if *dashAddr != "" {
		a.dashboard = serveDashboard(*dashAddr, a, *transportName, *dashTraces)
	}
	a.startStats(2 * time.Second)
	names := make([]string, len(a.rt.nodes))
	for i, nr := range a.rt.nodes {
		names[i] = nr.name
	}
	slog.Info("conductor: running", "transport", *transportName, "nodes", strings.Join(names, ","))

	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM)
	select {
	case s := <-sig:
		slog.Info("conductor: shutting down", "signal", s.String())
	case <-stopCh:
		slog.Info("conductor: shutting down", "reason", "requested by the application")
	}
	a.stop()
	for _, nr := range a.rt.nodes {
		slog.Info("conductor: node summary", "node", nr.name,
			"processed", nr.processed.Load(), "dropped", nr.dropped.Load())
	}
	if err := stopErr.Load(); err != nil {
		slog.Error("conductor: aborted", "err", *err)
		os.Exit(1)
	}
}

var (
	stopOnce sync.Once
	stopCh   = make(chan struct{})
	stopErr  atomic.Pointer[error]
)

// Shutdown asks the running application to stop: the nodes are deactivated
// and cleaned up through their lifecycle hooks, the transport is closed, and
// Run returns. Safe to call from any goroutine, and safe to call twice.
//
// Apps that finish — a mission, a scripted test run — should end this way
// rather than with os.Exit, which would skip that cleanup and leave stale
// entries on the ROS graph until discovery times them out.
func Shutdown() {
	stopOnce.Do(func() { close(stopCh) })
}

// Abort is Shutdown for the failure case: the same clean stop, but err is
// logged and the process exits non-zero.
func Abort(err error) {
	stopOnce.Do(func() {
		stopErr.Store(&err)
		close(stopCh)
	})
}

func envDomain() int {
	if s := os.Getenv("ROS_DOMAIN_ID"); s != "" {
		if n, err := strconv.Atoi(s); err == nil {
			return n
		}
	}
	return 0
}

type app struct {
	rt        *runtimeState
	statsQuit chan struct{}
	statsDone chan struct{}
	metrics   *http.Server
	dashboard *http.Server
}

// stringList collects a repeatable string flag.
type stringList []string

func (s *stringList) String() string { return strings.Join(*s, ",") }

func (s *stringList) Set(v string) error {
	*s = append(*s, v)
	return nil
}

// resolveParamFiles expands -params and -env into the ordered list of files
// to load. An environment adds params.<env>.yaml beside the last explicit
// file (or in the working directory), and it must exist if named.
func resolveParamFiles(explicit []string, env string) ([]string, error) {
	files := append([]string{}, explicit...)
	for _, f := range files {
		if _, err := os.Stat(f); err != nil {
			return nil, err
		}
	}
	if env == "" {
		return files, nil
	}
	dir := "."
	if len(files) > 0 {
		dir = filepath.Dir(files[len(files)-1])
	}
	envFile := filepath.Join(dir, "params."+env+".yaml")
	if _, err := os.Stat(envFile); err != nil {
		return nil, fmt.Errorf("environment %q selected but %s is missing", env, envFile)
	}
	return append(files, envFile), nil
}

// appOptions is everything Run's flags (or a test harness) can vary about
// how an application is wired.
type appOptions struct {
	transport    string
	topts        TransportOptions
	only         string                       // run only this node
	params       map[string]map[string]string // node -> parameter -> raw value
	manualTimers bool                         // do not start tickers; fire timers explicitly
}

func newApp(transportName string, topts TransportOptions, only string, nodeStructs ...any) (*app, error) {
	return newAppOpts(appOptions{transport: transportName, topts: topts, only: only}, nodeStructs...)
}

func newAppWithParams(transportName string, topts TransportOptions, only string, values map[string]map[string]string, nodeStructs ...any) (*app, error) {
	return newAppOpts(appOptions{transport: transportName, topts: topts, only: only, params: values}, nodeStructs...)
}

func newAppOpts(o appOptions, nodeStructs ...any) (*app, error) {
	tr, err := newTransport(o.transport, o.topts)
	if err != nil {
		return nil, err
	}
	rt := &runtimeState{transport: tr, paramValues: o.params}
	for _, ns := range nodeStructs {
		ptr := reflect.ValueOf(ns)
		if ptr.Kind() != reflect.Pointer || ptr.Elem().Kind() != reflect.Struct {
			return nil, fmt.Errorf("nodes must be pointers to structs, got %T", ns)
		}
		v := ptr.Elem()
		t := v.Type()
		name := snakeCase(t.Name())
		if o.only != "" && name != o.only {
			continue
		}
		if err := tr.DeclareNode(name); err != nil {
			return nil, fmt.Errorf("%s: %w", t.Name(), err)
		}
		nr := newNodeRuntime(name)
		hooks, err := findHooks(ptr)
		if err != nil {
			return nil, fmt.Errorf("%s: %w", t.Name(), err)
		}
		nr.lifecycle = newLifecycle(nr, hooks)
		rt.depsFor(name) // ensure every node appears in the dependency graph
		if err := bindFields(rt, nr, ptr); err != nil {
			return nil, fmt.Errorf("%s: %w", t.Name(), err)
		}
		if err := bindLifecycle(rt, nr); err != nil {
			return nil, fmt.Errorf("%s lifecycle: %w", t.Name(), err)
		}
		if err := bindParamServices(rt, nr); err != nil {
			return nil, fmt.Errorf("%s parameters: %w", t.Name(), err)
		}
		rt.nodes = append(rt.nodes, nr)
	}
	if len(rt.nodes) == 0 {
		return nil, fmt.Errorf("no nodes to run (node filter %q matched nothing?)", o.only)
	}
	if err := tr.Start(); err != nil {
		return nil, err
	}

	for _, nr := range rt.nodes {
		go nr.run()
	}
	if !o.manualTimers {
		for _, th := range rt.timers {
			th.start()
		}
	}
	return &app{rt: rt}, nil
}

// bindFields wires every exported field of ptr that declares a conductor
// endpoint (Sub, Pub, Param, Timer, Svc, Client, Action, ActionClient) onto
// the given node.
func bindFields(rt *runtimeState, nr *nodeRuntime, ptr reflect.Value) error {
	v := ptr.Elem()
	t := v.Type()
	for i := 0; i < t.NumField(); i++ {
		f := t.Field(i)
		if !f.IsExported() {
			continue
		}
		b, ok := v.Field(i).Addr().Interface().(binder)
		if !ok {
			continue
		}
		if err := b.bind(rt, nr, f, ptr); err != nil {
			return fmt.Errorf("%s: %w", f.Name, err)
		}
	}
	return nil
}

// bringUp configures and activates every node in dependency order.
func (a *app) bringUp() error {
	names := make([]string, len(a.rt.nodes))
	byName := map[string]*nodeRuntime{}
	for i, nr := range a.rt.nodes {
		names[i] = nr.name
		byName[nr.name] = nr
	}
	order, cycles := BringupOrder(names, a.rt.deps)
	if len(cycles) > 0 {
		slog.Warn("conductor: dependency cycle, bringing these up in declaration order",
			"nodes", strings.Join(cycles, ","))
	}
	slog.Info("conductor: bringup order", "order", strings.Join(order, " -> "))

	for _, name := range order {
		nr := byName[name]
		for _, t := range []Transition{TransitionConfigure, TransitionActivate} {
			if ok, err := nr.lifecycle.transition(t); !ok {
				return fmt.Errorf("node %s: %s: %w", name, t, err)
			}
		}
	}
	return nil
}

// shutDown drives every node to Finalized, in reverse bringup order so
// consumers stop before the things they depend on.
func (a *app) shutDown() {
	names := make([]string, len(a.rt.nodes))
	byName := map[string]*nodeRuntime{}
	for i, nr := range a.rt.nodes {
		names[i] = nr.name
		byName[nr.name] = nr
	}
	order, _ := BringupOrder(names, a.rt.deps)
	for i := len(order) - 1; i >= 0; i-- {
		nr := byName[order[i]]
		if nr.lifecycle.State() == StateActive {
			if ok, err := nr.lifecycle.transition(TransitionDeactivate); !ok {
				slog.Warn("conductor: deactivate failed", "node", nr.name, "err", err)
			}
		}
		if t, ok := shutdownFor(nr.lifecycle.State()); ok {
			if ok, err := nr.lifecycle.transition(t); !ok {
				slog.Warn("conductor: shutdown failed", "node", nr.name, "err", err)
			}
		}
	}
}

func (a *app) startStats(interval time.Duration) {
	a.statsQuit = make(chan struct{})
	a.statsDone = make(chan struct{})
	go func() {
		defer close(a.statsDone)
		t := time.NewTicker(interval)
		defer t.Stop()
		for {
			select {
			case <-a.statsQuit:
				return
			case <-t.C:
				for _, nr := range a.rt.nodes {
					slog.Info("conductor: stats", "node", nr.name,
						"processed", nr.processed.Load(), "dropped", nr.dropped.Load())
				}
			}
		}
	}()
}

func (a *app) stop() {
	if a.statsQuit != nil {
		close(a.statsQuit)
		<-a.statsDone
	}
	// Run lifecycle shutdown hooks while the executors are still draining.
	a.shutDown()
	for _, srv := range []*http.Server{a.metrics, a.dashboard} {
		if srv == nil {
			continue
		}
		ctx, cancel := context.WithTimeout(context.Background(), time.Second)
		srv.Shutdown(ctx)
		cancel()
	}
	for _, th := range a.rt.timers {
		if th.stop == nil {
			continue // never started (manual timers, as in tests)
		}
		close(th.stop)
		<-th.done
	}
	for _, nr := range a.rt.nodes {
		close(nr.quit)
		<-nr.done
	}
	if err := a.rt.transport.Close(); err != nil {
		slog.Warn("conductor: transport close", "err", err)
	}
}

func snakeCase(s string) string {
	var b strings.Builder
	for i, r := range s {
		if unicode.IsUpper(r) {
			if i > 0 {
				b.WriteByte('_')
			}
			b.WriteRune(unicode.ToLower(r))
		} else {
			b.WriteRune(r)
		}
	}
	return b.String()
}

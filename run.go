package conductor

import (
	"flag"
	"fmt"
	"log/slog"
	"os"
	"os/signal"
	"reflect"
	"strconv"
	"strings"
	"syscall"
	"time"
	"unicode"
)

type runtimeState struct {
	transport Transport
	nodes     []*nodeRuntime
	timers    []*timerHandle
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
func Run(nodes ...any) {
	fs := flag.NewFlagSet("conductor", flag.ExitOnError)
	only := fs.String("node", "", "run only the named node")
	transportName := fs.String("transport", "inproc", "message transport (inproc, zenoh)")
	endpoint := fs.String("zenoh-endpoint", "tcp/127.0.0.1:7447", "zenoh router endpoint")
	domain := fs.Int("domain", envDomain(), "ROS domain id")
	fs.Parse(os.Args[1:])

	a, err := newApp(*transportName, TransportOptions{Endpoint: *endpoint, Domain: *domain}, *only, nodes...)
	if err != nil {
		slog.Error("conductor: startup failed", "err", err)
		os.Exit(1)
	}
	a.startStats(2 * time.Second)
	names := make([]string, len(a.rt.nodes))
	for i, nr := range a.rt.nodes {
		names[i] = nr.name
	}
	slog.Info("conductor: running", "transport", *transportName, "nodes", strings.Join(names, ","))

	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM)
	s := <-sig
	slog.Info("conductor: shutting down", "signal", s.String())
	a.stop()
	for _, nr := range a.rt.nodes {
		slog.Info("conductor: node summary", "node", nr.name,
			"processed", nr.processed.Load(), "dropped", nr.dropped.Load())
	}
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
}

func newApp(transportName string, topts TransportOptions, only string, nodeStructs ...any) (*app, error) {
	tr, err := newTransport(transportName, topts)
	if err != nil {
		return nil, err
	}
	rt := &runtimeState{transport: tr}
	for _, ns := range nodeStructs {
		ptr := reflect.ValueOf(ns)
		if ptr.Kind() != reflect.Pointer || ptr.Elem().Kind() != reflect.Struct {
			return nil, fmt.Errorf("nodes must be pointers to structs, got %T", ns)
		}
		v := ptr.Elem()
		t := v.Type()
		name := snakeCase(t.Name())
		if only != "" && name != only {
			continue
		}
		if err := tr.DeclareNode(name); err != nil {
			return nil, fmt.Errorf("%s: %w", t.Name(), err)
		}
		nr := newNodeRuntime(name)
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
				return nil, fmt.Errorf("%s.%s: %w", t.Name(), f.Name, err)
			}
		}
		rt.nodes = append(rt.nodes, nr)
	}
	if len(rt.nodes) == 0 {
		return nil, fmt.Errorf("no nodes to run (node filter %q matched nothing?)", only)
	}
	if err := tr.Start(); err != nil {
		return nil, err
	}

	for _, nr := range rt.nodes {
		go nr.run()
	}
	for _, th := range rt.timers {
		th.start()
	}
	return &app{rt: rt}, nil
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
	for _, th := range a.rt.timers {
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

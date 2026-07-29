package main

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"net"
	"net/url"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	"conductor.dev/conductor"
	"conductor.dev/conductor/internal/deploy"
	"conductor.dev/conductor/internal/graph"
	"conductor.dev/conductor/internal/scan"
)

// runDashboard serves the fleet view: one page over every process of a
// deployment. The peers come from the same declarations the units were
// generated from, so an environment that deploys four nodes is four rows here
// without anyone writing an address down.
func runDashboard(args []string) error {
	dir, args := splitDir(args)
	fs := flag.NewFlagSet("dashboard", flag.ExitOnError)
	env := fs.String("env", "", "environment whose processes to aggregate (see environments.json)")
	addr := fs.String("addr", ":5000", "serve the fleet view on this address")
	var peerList stringList
	fs.Var(&peerList, "peers", "peer dashboard to aggregate as [name=]url (repeatable, or comma-separated)")
	host := fs.String("host", "", "dial this host instead of the environment's deploy host (an ip, a tunnel, or localhost)")
	timeout := fs.Duration("timeout", 2*time.Second, "how long to wait for each process")
	traces := fs.Int("traces", 0, "collect this many recent spans from across the deployment and stitch them into traces (the processes must run with -dashboard-traces)")
	once := fs.Bool("once", false, "print the merged state as JSON and exit")
	if err := fs.Parse(args); err != nil {
		return err
	}

	peers, opts, err := resolvePeers(dir, *env, peerList)
	if err != nil {
		return err
	}
	opts.Traces = *traces
	if *host != "" {
		peers, err = rehost(peers, *host)
		if err != nil {
			return err
		}
	}
	if len(peers) == 0 {
		return fmt.Errorf("no peers to aggregate: pass -peers, or give the environment a dashboard_addr " +
			"so its units serve one each")
	}

	if *once {
		ctx, cancel := context.WithTimeout(context.Background(), *timeout+time.Second)
		defer cancel()
		enc := json.NewEncoder(os.Stdout)
		enc.SetIndent("", "  ")
		return enc.Encode(conductor.FleetWith(ctx, peers, *timeout, opts))
	}

	srv, err := conductor.ServeFleet(*addr, peers, *timeout, opts)
	if err != nil {
		return err
	}
	fmt.Printf("conductor fleet view on http://%s/\n", displayAddr(*addr))
	if *traces > 0 {
		fmt.Printf("  collecting up to %d spans from across the deployment\n", *traces)
	}
	for _, p := range peers {
		fmt.Printf("  %-16s %s\n", p.Name, p.URL)
	}
	fmt.Println("\nCtrl-C to stop.")

	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM)
	<-sig
	return srv.Close()
}

// resolvePeers takes the explicit list if there is one, and otherwise derives
// it from the environment. Either way it also collects what the application
// declares statically about topics that come from outside it, which the
// running processes cannot know.
func resolvePeers(dir, env string, explicit []string) ([]conductor.Peer, conductor.FleetOptions, error) {
	var opts conductor.FleetOptions
	app, err := resolve(dir, env)
	if err != nil && len(explicit) == 0 {
		return nil, opts, err
	}
	if err == nil {
		opts.External = externalTopics(app)
	}
	if len(explicit) > 0 {
		peers, err := parsePeers(explicit)
		return peers, opts, err
	}
	if app.Env == nil {
		return nil, opts, fmt.Errorf("no environment selected: pass -env, or -peers to aggregate an ad-hoc set")
	}
	g, issues := graph.Validate(app)
	if graph.Errors(issues) {
		// The fleet view is a runtime tool; a broken graph is worth saying
		// out loud but not worth refusing to look at a running system for.
		fmt.Fprintf(os.Stderr, "conductor: warning: %s has graph errors; run `conductor check -env %s`\n",
			app.Name, app.Env.Name())
	}
	order, _ := g.BringupOrder()
	return deploy.Peers(app, order), opts, nil
}

// externalTopics is the set of topics an environment expects someone else to
// publish or consume.
func externalTopics(app *scan.App) map[string]bool {
	out := map[string]bool{}
	for _, e := range app.Externals {
		if e.Role == "publisher" || e.Role == "subscriber" {
			out[e.Topic] = true
		}
	}
	return out
}

// parsePeers reads the -peers form: [name=]url, comma-separated or repeated.
func parsePeers(values []string) ([]conductor.Peer, error) {
	var out []conductor.Peer
	for _, value := range values {
		for _, item := range strings.Split(value, ",") {
			item = strings.TrimSpace(item)
			if item == "" {
				continue
			}
			name, url := "", item
			if i := strings.Index(item, "="); i > 0 {
				name, url = item[:i], item[i+1:]
			}
			if !strings.Contains(url, "://") {
				url = "http://" + url
			}
			if name == "" {
				name = strings.TrimPrefix(strings.TrimPrefix(url, "http://"), "https://")
			}
			out = append(out, conductor.Peer{Name: name, URL: url})
		}
	}
	if len(out) == 0 {
		return nil, fmt.Errorf("-peers was given but named nothing")
	}
	return out, nil
}

// rehost points derived peers at another machine, keeping their ports: the
// same deployment reached over a tunnel, by ip, or installed locally for a
// trial run.
func rehost(peers []conductor.Peer, host string) ([]conductor.Peer, error) {
	out := make([]conductor.Peer, 0, len(peers))
	for _, p := range peers {
		u, err := url.Parse(p.URL)
		if err != nil {
			return nil, fmt.Errorf("peer %s: %w", p.Name, err)
		}
		u.Host = net.JoinHostPort(host, u.Port())
		p.URL = u.String()
		out = append(out, p)
	}
	return out, nil
}

// stringList collects a repeatable string flag.
type stringList []string

func (s *stringList) String() string { return strings.Join(*s, ",") }

func (s *stringList) Set(v string) error {
	*s = append(*s, v)
	return nil
}

func displayAddr(addr string) string {
	if strings.HasPrefix(addr, ":") {
		return "localhost" + addr
	}
	return addr
}

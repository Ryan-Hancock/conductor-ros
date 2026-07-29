// Package deploy turns a validated application graph into a release bundle
// and puts it on a robot.
//
// The shape is deliberately boring: cross-compile one binary, stage it with
// the parameter files, the generated launch file and one systemd unit per
// node, tar it, copy it to the target, and run the bundle's install.sh there.
// Everything Conductor knows statically — the node list, the bringup order,
// the environment's transport — is baked into those artifacts, so the robot
// needs no ROS workspace, no colcon, and no runtime discovery of what to run.
package deploy

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path"
	"path/filepath"
	"runtime"
	"sort"
	"strconv"
	"strings"
	"time"

	"conductor.dev/conductor/internal/gen"
	"conductor.dev/conductor/internal/graph"
	"conductor.dev/conductor/internal/scan"
)

// Options is a deploy request: the environment's declared configuration with
// command-line overrides already applied.
type Options struct {
	Version string
	GOOS    string
	GOARCH  string
	Tags    []string
	CGO     bool
	CC      string

	Host   string // user@host; empty deploys to this machine
	Prefix string
	Scope  string // systemd scope: system or user
	Sudo   string // privilege prefix used for system scope
	Keep   int

	BundleOnly bool // build the bundle, do not ship it
	NoRestart  bool
	NoSystemd  bool
	Rollback   bool
	DryRun     bool

	OutDir string // where the bundle is written (default <app>/gen/deploy)
	Out    io.Writer
}

// Manifest records what was built, from which source, and for where. It ships
// inside the bundle so a robot can answer "what exactly is running here?"
// without anyone guessing from timestamps.
type Manifest struct {
	App          string            `json:"app"`
	Env          string            `json:"env,omitempty"`
	Version      string            `json:"version"`
	BuiltAt      string            `json:"built_at"`
	BuiltBy      string            `json:"built_by"`
	GOOS         string            `json:"goos"`
	GOARCH       string            `json:"goarch"`
	Tags         []string          `json:"tags,omitempty"`
	CGO          bool              `json:"cgo"`
	Git          string            `json:"git,omitempty"`
	Nodes        []string          `json:"nodes"`
	BringupOrder []string          `json:"bringup_order"`
	Units        []string          `json:"units"`
	Graph        string            `json:"graph_fingerprint"`
	Flags        []string          `json:"runtime_flags,omitempty"`
	Files        map[string]string `json:"files"` // path -> sha256
}

// Run performs a deployment (or a rollback) and reports what it did.
func Run(app *scan.App, g *graph.Graph, o Options) error {
	if o.Out == nil {
		o.Out = os.Stdout
	}
	applyDefaults(app, &o)

	if o.Rollback {
		return rollback(app, o)
	}
	if err := validate(app, o); err != nil {
		return err
	}

	dep := gen.Deployment{
		App:           app.Name,
		Env:           envName(app),
		Version:       o.Version,
		Prefix:        o.Prefix,
		Scope:         o.Scope,
		Flags:         runtimeFlags(app, o),
		Environ:       environ(app),
		Metrics:       envAddr(app, func(e *scan.Environment) string { return e.Metrics }),
		Dashboard:     envAddr(app, func(e *scan.Environment) string { return e.Dashboard }),
		SingleProcess: singleProcess(app),
	}

	stage := filepath.Join(o.OutDir, app.Name+"-"+o.Version)
	if err := os.RemoveAll(stage); err != nil {
		return err
	}
	manifest, err := build(app, g, dep, o, stage)
	if err != nil {
		return err
	}
	tarball, err := archive(stage, filepath.Join(o.OutDir, app.Name+"-"+o.Version+".tar.gz"))
	if err != nil {
		return err
	}

	fmt.Fprintf(o.Out, "\nbundle %s\n", rel(app, tarball))
	fmt.Fprintf(o.Out, "  app         %s%s\n", manifest.App, envSuffix(manifest.Env))
	fmt.Fprintf(o.Out, "  version     %s\n", manifest.Version)
	fmt.Fprintf(o.Out, "  platform    %s/%s%s\n", manifest.GOOS, manifest.GOARCH, tagSuffix(manifest.Tags))
	fmt.Fprintf(o.Out, "  graph       %s\n", manifest.Graph)
	fmt.Fprintf(o.Out, "  bringup     %s\n", strings.Join(manifest.BringupOrder, " -> "))
	fmt.Fprintf(o.Out, "  units       %s\n", strings.Join(manifest.Units, " "))
	if dep.SingleProcess {
		fmt.Fprintf(o.Out, "              one process for every node, as the %s transport requires\n", transportName(app))
	}
	fmt.Fprintf(o.Out, "  flags       %s\n", strings.Join(dep.Flags, " "))
	if dep.Metrics != "" {
		var addrs []string
		for i, n := range manifest.BringupOrder {
			addrs = append(addrs, n+" "+dep.MetricsAddr(i))
			if dep.SingleProcess {
				addrs = []string{dep.MetricsAddr(0)}
				break
			}
		}
		fmt.Fprintf(o.Out, "  metrics     %s\n", strings.Join(addrs, ", "))
	}
	if peers := Peers(app, manifest.BringupOrder); len(peers) > 0 {
		var addrs []string
		for _, p := range peers {
			addrs = append(addrs, p.Name+" "+p.URL)
		}
		fmt.Fprintf(o.Out, "  dashboards  %s\n", strings.Join(addrs, ", "))
		fmt.Fprintf(o.Out, "              aggregate them with: conductor dashboard %s -env %s\n",
			rel(app, app.Dir), envName(app))
	}

	if o.BundleOnly {
		fmt.Fprintf(o.Out, "\nnot shipped (-bundle). Copy it to the robot and run install.sh.\n")
		return nil
	}
	return ship(app, o, stage, tarball, dep)
}

func applyDefaults(app *scan.App, o *Options) {
	var d *scan.DeployConfig
	if app.Env != nil {
		d = app.Env.Deploy
	}
	if d == nil {
		d = &scan.DeployConfig{}
	}
	pick := func(dst *string, vals ...string) {
		for _, v := range vals {
			if *dst == "" && v != "" {
				*dst = v
			}
		}
	}
	pick(&o.Host, d.Host)
	if o.Host == "local" {
		// Deploy an environment to this machine — how a robot environment
		// gets tried out before there is a robot to try it on.
		o.Host = ""
	}
	pick(&o.GOOS, d.GOOS, "linux")
	pick(&o.GOARCH, d.GOARCH, runtime.GOARCH)
	pick(&o.Scope, d.Scope, "system")
	pick(&o.Prefix, d.Prefix, defaultPrefix(o.Scope, o.Host))
	pick(&o.CC, d.CC)
	if o.Sudo == "" {
		o.Sudo = d.Sudo
		if o.Sudo == "" && o.Scope == "system" {
			o.Sudo = "sudo -n"
		}
	}
	if o.Sudo == "none" || o.Scope == "user" {
		o.Sudo = ""
	}
	if len(o.Tags) == 0 {
		o.Tags = d.Tags
	}
	o.CGO = o.CGO || d.CGO
	if o.Keep == 0 {
		o.Keep = d.Keep
	}
	if o.Version == "" {
		o.Version = version()
	}
	if o.OutDir == "" {
		o.OutDir = filepath.Join(app.Dir, "gen", "deploy")
	}
}

// defaultPrefix picks an install root. User-scope units are generated with
// absolute paths, and the deploying machine cannot know a remote user's home,
// so a remote user-scope deploy has to be told where to install.
func defaultPrefix(scope, host string) string {
	if scope != "user" {
		return "/opt/conductor"
	}
	if host != "" {
		return "" // reported by validate
	}
	home, err := os.UserHomeDir()
	if err != nil {
		return ""
	}
	return filepath.Join(home, ".local", "share", "conductor")
}

// validate rejects the combinations that would only fail on the robot: a
// binary built without the transport its environment asks for, or a cgo
// cross-build with no cross compiler. Both are the deployment equivalent of
// an unwired graph — knowable here, painful there.
func validate(app *scan.App, o Options) error {
	transport := ""
	if app.Env != nil {
		transport = app.Env.Transport
	}
	if transport == "zenoh" && !hasTag(o.Tags, "zenoh") {
		return fmt.Errorf("environment %q uses the zenoh transport, but the build has no zenoh tag: "+
			"add \"tags\": [\"zenoh\"] and \"cgo\": true to its deploy config (the binary would exit at startup with "+
			"\"unknown transport\")", envName(app))
	}
	if hasTag(o.Tags, "zenoh") && !o.CGO {
		return fmt.Errorf("the zenoh transport is cgo; set \"cgo\": true in the environment's deploy config")
	}
	if o.CGO && o.CC == "" && o.GOARCH != runtime.GOARCH {
		return fmt.Errorf("cross-compiling %s/%s with cgo needs a cross compiler: set \"cc\" in the environment's "+
			"deploy config (e.g. aarch64-linux-gnu-gcc)", o.GOOS, o.GOARCH)
	}
	if o.Scope != "system" && o.Scope != "user" {
		return fmt.Errorf("unknown systemd scope %q (want system or user)", o.Scope)
	}
	if !filepath.IsAbs(o.Prefix) {
		return fmt.Errorf("install prefix must be absolute (the units name it directly); "+
			"set \"prefix\" in the environment's deploy config or pass -prefix, got %q", o.Prefix)
	}
	return nil
}

// build compiles the binary and stages every file the release consists of.
func build(app *scan.App, g *graph.Graph, dep gen.Deployment, o Options, stage string) (*Manifest, error) {
	binDir := filepath.Join(stage, "bin")
	if err := os.MkdirAll(binDir, 0o755); err != nil {
		return nil, err
	}

	relPkg, err := filepath.Rel(app.ModuleRoot, app.Dir)
	if err != nil {
		return nil, err
	}
	args := []string{"build", "-trimpath", "-o", filepath.Join(binDir, app.Name)}
	if len(o.Tags) > 0 {
		args = append(args, "-tags", strings.Join(o.Tags, ","))
	}
	args = append(args, "./"+filepath.ToSlash(relPkg))
	cmd := exec.Command("go", args...)
	cmd.Dir = app.ModuleRoot
	cmd.Env = append(os.Environ(), "GOOS="+o.GOOS, "GOARCH="+o.GOARCH, "CGO_ENABLED="+boolEnv(o.CGO))
	if o.CC != "" {
		cmd.Env = append(cmd.Env, "CC="+o.CC)
	}
	cmd.Stdout, cmd.Stderr = o.Out, os.Stderr
	fmt.Fprintf(o.Out, "building %s/%s%s\n", o.GOOS, o.GOARCH, tagSuffix(o.Tags))
	if err := cmd.Run(); err != nil {
		return nil, fmt.Errorf("go build: %w", err)
	}

	// Parameter files: the generated defaults first, then the environment's
	// overlays, in the order the units will load them.
	write := func(rel, content string, mode os.FileMode) error {
		p := filepath.Join(stage, rel)
		if err := os.MkdirAll(filepath.Dir(p), 0o755); err != nil {
			return err
		}
		return os.WriteFile(p, []byte(content), mode)
	}
	if err := write("params.yaml", gen.ParamsYAML(g), 0o644); err != nil {
		return nil, err
	}
	for _, pf := range envParams(app) {
		src, err := os.ReadFile(filepath.Join(app.Dir, pf))
		if err != nil {
			return nil, fmt.Errorf("environment parameter file: %w", err)
		}
		if err := write(filepath.Base(pf), string(src), 0o644); err != nil {
			return nil, err
		}
	}
	// The transform tree ships with the release: the robot's geometry is
	// part of what is deployed, not something to install separately.
	if app.FramesFile != "" {
		src, err := os.ReadFile(filepath.Join(app.Dir, app.FramesFile))
		if err != nil {
			return nil, fmt.Errorf("frames file: %w", err)
		}
		if err := write("frames.json", string(src), 0o644); err != nil {
			return nil, err
		}
	}
	launchBin := path.Join(dep.CurrentDir(), "bin", app.Name)
	if err := write(app.Name+".launch.xml", gen.LaunchXML(g, launchBin), 0o644); err != nil {
		return nil, err
	}

	units := gen.SystemdUnits(g, dep)
	names := make([]string, 0, len(units))
	for name := range units {
		names = append(names, name)
	}
	sort.Strings(names)
	for _, name := range names {
		if err := write(filepath.Join("systemd", name), units[name], 0o644); err != nil {
			return nil, err
		}
	}
	if err := write("install.sh", gen.InstallScript(dep, names, o.Keep), 0o755); err != nil {
		return nil, err
	}

	order, _ := g.BringupOrder()
	nodes := make([]string, len(app.Nodes))
	for i, n := range app.Nodes {
		nodes[i] = n.Name
	}
	m := &Manifest{
		App:          app.Name,
		Env:          envName(app),
		Version:      o.Version,
		BuiltAt:      time.Now().UTC().Format(time.RFC3339),
		BuiltBy:      builtBy(),
		GOOS:         o.GOOS,
		GOARCH:       o.GOARCH,
		Tags:         o.Tags,
		CGO:          o.CGO,
		Git:          gitDescribe(app.ModuleRoot),
		Nodes:        nodes,
		BringupOrder: order,
		Units:        names,
		Graph:        Fingerprint(g),
		Flags:        dep.Flags,
	}
	m.Files, err = checksums(stage)
	if err != nil {
		return nil, err
	}
	b, err := json.MarshalIndent(m, "", "  ")
	if err != nil {
		return nil, err
	}
	if err := write("manifest.json", string(b)+"\n", 0o644); err != nil {
		return nil, err
	}
	return m, nil
}

// ship copies the bundle to the target and runs its install.sh there.
func ship(app *scan.App, o Options, stage, tarball string, dep gen.Deployment) error {
	r := Runner{Host: o.Host, Sudo: o.Sudo, DryRun: o.DryRun, Out: o.Out}
	flags := installFlags(o)

	if o.Host == "" {
		fmt.Fprintf(o.Out, "\ninstalling locally under %s\n", dep.AppDir())
		return r.Script(shJoin(append([]string{"bash", filepath.Join(stage, "install.sh")}, flags...)))
	}

	remote := path.Join("/tmp", filepath.Base(tarball))
	unpack := path.Join("/tmp", app.Name+"-"+o.Version)
	fmt.Fprintf(o.Out, "\nshipping to %s\n", o.Host)
	if err := r.Copy(tarball, remote); err != nil {
		return err
	}
	script := strings.Join([]string{
		"set -e",
		shJoin([]string{"rm", "-rf", unpack}),
		shJoin([]string{"mkdir", "-p", unpack}),
		shJoin([]string{"tar", "-xzf", remote, "-C", unpack}),
		shJoin(append([]string{"bash", path.Join(unpack, "install.sh")}, flags...)),
		shJoin([]string{"rm", "-rf", unpack, remote}),
	}, "\n")
	return r.Script(script)
}

func rollback(app *scan.App, o Options) error {
	r := Runner{Host: o.Host, Sudo: o.Sudo, DryRun: o.DryRun, Out: o.Out}
	script := shJoin([]string{
		"bash", path.Join(o.Prefix, app.Name, "current", "install.sh"),
		"--rollback", "--prefix", o.Prefix, "--scope", o.Scope,
	})
	where := "locally"
	if o.Host != "" {
		where = "on " + o.Host
	}
	fmt.Fprintf(o.Out, "rolling %s back to its previous release %s\n", app.Name, where)
	return r.Script(script)
}

func installFlags(o Options) []string {
	flags := []string{"--prefix", o.Prefix, "--scope", o.Scope}
	if o.Keep > 0 {
		flags = append(flags, "--keep", strconv.Itoa(o.Keep))
	}
	if o.NoRestart {
		flags = append(flags, "--no-restart")
	}
	if o.NoSystemd {
		flags = append(flags, "--no-systemd")
	}
	return flags
}

// runtimeFlags are the flags the units pass to the binary: the environment's
// transport settings plus its parameter files, resolved to their installed
// paths. They are explicit rather than -env so that what runs on the robot is
// readable in `systemctl cat`.
func runtimeFlags(app *scan.App, o Options) []string {
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
	current := path.Join(o.Prefix, app.Name, "current")
	flags = append(flags, "-params", path.Join(current, "params.yaml"))
	for _, pf := range env.Params {
		flags = append(flags, "-params", path.Join(current, filepath.Base(pf)))
	}
	if app.FramesFile != "" {
		flags = append(flags, "-frames", path.Join(current, "frames.json"))
	}
	if env.Trace {
		flags = append(flags, "-trace")
	}
	return flags
}

// singleProcess reports whether the release should run as one unit. The
// in-process transport's bus does not leave the process, so splitting the
// application into a unit per node would leave the nodes unable to talk —
// a mistake that is obvious here and mystifying on a robot.
func singleProcess(app *scan.App) bool {
	return transportName(app) == "inproc"
}

func envAddr(app *scan.App, pick func(*scan.Environment) string) string {
	if app.Env == nil {
		return ""
	}
	return pick(app.Env)
}

func transportName(app *scan.App) string {
	if app.Env == nil || app.Env.Transport == "" {
		return "inproc"
	}
	return app.Env.Transport
}

func environ(app *scan.App) map[string]string {
	if app.Env == nil || app.Env.Deploy == nil {
		return nil
	}
	return app.Env.Deploy.Env
}

func envParams(app *scan.App) []string {
	if app.Env == nil {
		return nil
	}
	return app.Env.Params
}

func envName(app *scan.App) string {
	if app.Env == nil {
		return ""
	}
	return app.Env.Name()
}

// Fingerprint is a stable hash of everything the graph declares: nodes, their
// endpoints and types, QoS, parameters and timers. Two releases with the same
// fingerprint talk to the ROS graph identically, which is the question worth
// answering when a robot misbehaves after a deploy.
func Fingerprint(g *graph.Graph) string {
	var b strings.Builder
	for _, n := range g.App.Nodes {
		fmt.Fprintf(&b, "node %s\n", n.Name)
		for _, t := range n.Timers {
			fmt.Fprintf(&b, " timer %s %s\n", t.Field, t.Rate)
		}
		for _, s := range n.Subs {
			fmt.Fprintf(&b, " sub %s %s %s\n", s.Topic, s.GoType, s.QoS)
		}
		for _, p := range n.Pubs {
			fmt.Fprintf(&b, " pub %s %s %s\n", p.Topic, p.GoType, p.QoS)
		}
		for _, s := range n.Services {
			fmt.Fprintf(&b, " svc %s %s %s\n", s.Service, s.ReqType, s.ResType)
		}
		for _, c := range n.Clients {
			fmt.Fprintf(&b, " call %s %s %s\n", c.Service, c.ReqType, c.ResType)
		}
		for _, a := range n.Actions {
			fmt.Fprintf(&b, " act %s %s %s %s\n", a.Action, a.GoalType, a.FeedbackType, a.ResultType)
		}
		for _, a := range n.ActionClients {
			fmt.Fprintf(&b, " send %s %s %s %s\n", a.Action, a.GoalType, a.FeedbackType, a.ResultType)
		}
		for _, p := range n.Params {
			fmt.Fprintf(&b, " param %s %s %s\n", p.Name, p.GoType, p.Default)
		}
	}
	ext := make([]string, 0, len(g.App.Externals))
	for _, e := range g.App.Externals {
		ext = append(ext, fmt.Sprintf("external %s %s %s %s\n", e.Topic, e.Type, e.Role, e.QoS))
	}
	sort.Strings(ext)
	b.WriteString(strings.Join(ext, ""))
	sum := sha256.Sum256([]byte(b.String()))
	return "sha256:" + hex.EncodeToString(sum[:])[:16]
}

func checksums(stage string) (map[string]string, error) {
	out := map[string]string{}
	err := filepath.WalkDir(stage, func(p string, d os.DirEntry, err error) error {
		if err != nil || d.IsDir() {
			return err
		}
		f, err := os.Open(p)
		if err != nil {
			return err
		}
		defer f.Close()
		h := sha256.New()
		if _, err := io.Copy(h, f); err != nil {
			return err
		}
		rel, err := filepath.Rel(stage, p)
		if err != nil {
			return err
		}
		out[filepath.ToSlash(rel)] = hex.EncodeToString(h.Sum(nil))
		return nil
	})
	return out, err
}

// archive tars the stage directory. The bundle is what gets copied, and what
// can be archived alongside a robot's logs.
func archive(stage, dest string) (string, error) {
	if err := os.MkdirAll(filepath.Dir(dest), 0o755); err != nil {
		return "", err
	}
	cmd := exec.Command("tar", "-czf", dest, "-C", stage, ".")
	if out, err := cmd.CombinedOutput(); err != nil {
		return "", fmt.Errorf("tar: %v: %s", err, strings.TrimSpace(string(out)))
	}
	return dest, nil
}

// version identifies a release: sortable time plus the source revision, so
// `ls releases/` on a robot reads as a history.
func version() string {
	return time.Now().UTC().Format("20060102-150405")
}

func gitDescribe(dir string) string {
	rev, err := exec.Command("git", "-C", dir, "rev-parse", "--short", "HEAD").Output()
	if err != nil {
		return ""
	}
	out := strings.TrimSpace(string(rev))
	if status, err := exec.Command("git", "-C", dir, "status", "--porcelain").Output(); err == nil && len(strings.TrimSpace(string(status))) > 0 {
		out += "-dirty"
	}
	return out
}

func builtBy() string {
	host, _ := os.Hostname()
	if u := os.Getenv("USER"); u != "" && host != "" {
		return u + "@" + host
	}
	return host
}

func hasTag(tags []string, want string) bool {
	for _, t := range tags {
		if t == want {
			return true
		}
	}
	return false
}

func boolEnv(b bool) string {
	if b {
		return "1"
	}
	return "0"
}

func tagSuffix(tags []string) string {
	if len(tags) == 0 {
		return ""
	}
	return " (tags: " + strings.Join(tags, ",") + ")"
}

func envSuffix(env string) string {
	if env == "" {
		return ""
	}
	return " [env " + env + "]"
}

func rel(app *scan.App, p string) string {
	if r, err := filepath.Rel(app.ModuleRoot, p); err == nil && !strings.HasPrefix(r, "..") {
		return r
	}
	return p
}

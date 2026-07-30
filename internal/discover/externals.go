package discover

import (
	"bytes"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"

	"conductor.dev/conductor/internal/scan"
)

// Options controls how much of the graph becomes a declaration.
type Options struct {
	// All emits every interface on the graph, not only the ones this
	// application uses. It is how you look at a stack you are adopting; the
	// default answers the narrower and more useful question, "what does this
	// application depend on out there?"
	All bool

	// Infrastructure keeps the parameter and lifecycle services every ROS
	// node carries. Conductor provides those itself, so they are noise in an
	// externals block — until you are writing something that drives them.
	Infrastructure bool
}

// Finding is one difference between what an application declares and what the
// graph actually offers.
type Finding struct {
	Kind    FindingKind
	Topic   string
	Role    string
	Message string
}

type FindingKind string

const (
	// FindingMissing: the application uses it, the graph provides it, nothing
	// declares it — so the checker cannot see it and the build cannot verify
	// it.
	FindingMissing FindingKind = "missing"
	// FindingMismatch: declared, but the graph says a different type or QoS.
	// This is the one that matters: a wrong type or an incompatible profile is
	// silence at runtime, and the checker cannot catch what the declaration
	// itself gets wrong.
	FindingMismatch FindingKind = "mismatch"
	// FindingAbsent: declared, but nothing on the graph offers it. Often
	// benign — the stack may be half up — so it is reported, never "fixed".
	FindingAbsent FindingKind = "absent"
	// FindingConflict: two peers advertise the same interface with different
	// type hashes, so they are running different definitions of it.
	FindingConflict FindingKind = "conflict"
)

// Report is the result of comparing an application's declarations with a live
// graph.
type Report struct {
	Graph     *Graph
	App       string
	Env       string
	Externals []scan.External // the merged block: declared, corrected, extended
	Findings  []Finding
	Ours      []string // the application's own nodes, seen on the graph
}

// Changed reports whether anything the graph knows would change the
// declarations — that is, whether -write has work to do.
func (r *Report) Changed() bool {
	for _, f := range r.Findings {
		if f.Kind == FindingMissing || f.Kind == FindingMismatch {
			return true
		}
	}
	return false
}

// Externals derives the externals block for app from a live graph, and reports
// how it differs from what the application declares.
//
// The application's own endpoints are excluded: a conductor process on the
// graph advertises everything it publishes and subscribes to, and declaring
// itself external would tell the checker that its own topics come from
// somewhere else.
func Externals(app *scan.App, g *Graph, opts Options) *Report {
	r := &Report{Graph: g, App: app.Name}
	if app.Env != nil {
		r.Env = app.Env.Name()
	}

	ours := map[string]bool{}
	for _, n := range app.Nodes {
		ours[n.Name] = true
	}
	onGraph := map[string]bool{}
	for _, name := range g.Nodes() {
		if ours[name] {
			onGraph[name] = true
		}
	}
	r.Ours = sorted(onGraph)

	used := usedInterfaces(app)
	declared := map[string]scan.External{}
	for _, e := range app.Externals {
		declared[key(e.Topic, e.Role)] = e
	}

	// Walk the graph, deciding for each interface whether this application
	// needs a declaration for it and what that declaration should say.
	merged := map[string]scan.External{}
	seen := map[string]bool{}
	for _, i := range g.Interfaces() {
		if i.Infrastructure && !opts.Infrastructure {
			continue
		}
		if len(i.Hashes) > 1 {
			r.Findings = append(r.Findings, Finding{FindingConflict, i.Name, "", fmt.Sprintf(
				"%s %q is advertised with %d different type hashes (%s): the peers are running "+
					"different definitions of %s", i.Kind, i.Name, len(i.Hashes),
				strings.Join(shortHashes(i.Hashes), ", "), i.Type)})
		}
		if profiles, disagree := i.Disagreement(); disagree {
			r.Findings = append(r.Findings, Finding{FindingConflict, i.Name, "", fmt.Sprintf(
				"topic %q has publishers offering %s; %q is declared, being the weakest offer "+
					"and so the one a subscriber must match", i.Name,
				strings.Join(profiles, " and "), i.QoS)})
		}

		use, inUse := used[i.Name]
		for _, role := range rolesFor(&i, ours, use, inUse, opts.All) {
			want := scan.External{Topic: i.Name, Type: i.Type, Role: role, QoS: qosFor(&i, role)}
			k := key(want.Topic, want.Role)
			seen[k] = true
			merged[k] = want

			have, ok := declared[k]
			switch {
			case !ok:
				r.Findings = append(r.Findings, Finding{FindingMissing, want.Topic, role, fmt.Sprintf(
					"%s is on the graph (%s%s) but not declared; %s",
					describe(&i, role), want.Type, qosNote(want.QoS), providedBy(&i, role))})
			case have.Type != want.Type:
				r.Findings = append(r.Findings, Finding{FindingMismatch, want.Topic, role, fmt.Sprintf(
					"declared as %s, but the graph offers %s", have.Type, want.Type)})
			case want.QoS != "" && have.QoS != "" && have.QoS != want.QoS:
				r.Findings = append(r.Findings, Finding{FindingMismatch, want.Topic, role, fmt.Sprintf(
					"declared qos %q, but the publisher offers %q", have.QoS, want.QoS)})
			}
		}
	}

	// Anything declared that the graph did not answer for. Reported, not
	// removed: half a stack being up is the normal case while developing.
	for k, e := range declared {
		if seen[k] {
			continue
		}
		merged[k] = e
		r.Findings = append(r.Findings, Finding{FindingAbsent, e.Topic, e.Role, fmt.Sprintf(
			"declared as an external %s of %s, but nothing on the graph offers it", e.Role, e.Type)})
	}

	// Emit in the file's own order, with anything new appended: this block is
	// maintained by hand as well as by this command, and a tool that reshuffles
	// a file to say the same thing produces a diff nobody can review.
	r.Externals = make([]scan.External, 0, len(merged))
	emitted := map[string]bool{}
	for _, e := range app.Externals {
		k := key(e.Topic, e.Role)
		if emitted[k] {
			continue
		}
		emitted[k] = true
		if updated, ok := merged[k]; ok {
			r.Externals = append(r.Externals, updated)
		} else {
			r.Externals = append(r.Externals, e)
		}
	}
	added := make([]scan.External, 0, len(merged))
	for k, e := range merged {
		if !emitted[k] {
			added = append(added, e)
		}
	}
	sort.Slice(added, func(a, b int) bool {
		if added[a].Topic != added[b].Topic {
			return added[a].Topic < added[b].Topic
		}
		return added[a].Role < added[b].Role
	})
	r.Externals = append(r.Externals, added...)
	sort.SliceStable(r.Findings, func(a, b int) bool {
		if r.Findings[a].Kind != r.Findings[b].Kind {
			return findingOrder(r.Findings[a].Kind) < findingOrder(r.Findings[b].Kind)
		}
		return r.Findings[a].Topic < r.Findings[b].Topic
	})
	return r
}

func findingOrder(k FindingKind) int {
	switch k {
	case FindingMismatch:
		return 0
	case FindingConflict:
		return 1
	case FindingMissing:
		return 2
	default:
		return 3
	}
}

// usage is how this application uses an interface: which side of it we are on.
type usage struct {
	publishes, subscribes bool
	serves, calls         bool
	servesAction          bool
	callsAction           bool
}

// usedInterfaces is every interface the application names, keyed by the name
// the graph would use for it.
func usedInterfaces(app *scan.App) map[string]usage {
	out := map[string]usage{}
	mark := func(name string, f func(*usage)) {
		u := out[strings.TrimPrefix(name, "/")]
		f(&u)
		out[strings.TrimPrefix(name, "/")] = u
	}
	for _, n := range app.Nodes {
		for _, e := range n.Pubs {
			mark(e.Topic, func(u *usage) { u.publishes = true })
		}
		for _, e := range n.Subs {
			mark(e.Topic, func(u *usage) { u.subscribes = true })
		}
		for _, e := range n.Services {
			mark(e.Service, func(u *usage) { u.serves = true })
		}
		for _, e := range n.Clients {
			mark(e.Service, func(u *usage) { u.calls = true })
		}
		for _, e := range n.Actions {
			mark(e.Action, func(u *usage) { u.servesAction = true })
		}
		for _, e := range n.ActionClients {
			mark(e.Action, func(u *usage) { u.callsAction = true })
		}
	}
	return out
}

// rolesFor decides which externals roles an interface needs, from the point of
// view of this application: the role names what is *outside*, so our
// subscription needs an external publisher, and our client needs an external
// server.
func rolesFor(i *Interface, ours map[string]bool, use usage, inUse, all bool) []string {
	provided := othersAmong(i.Providers(), ours)
	consumed := othersAmong(i.Consumers(), ours)

	var roles []string
	switch i.Kind {
	case KindTopic:
		if provided && (use.subscribes || (all && !inUse)) {
			roles = append(roles, "publisher")
		}
		if consumed && (use.publishes || (all && !inUse)) {
			roles = append(roles, "subscriber")
		}
	case KindService:
		if provided && (use.calls || (all && !inUse)) {
			roles = append(roles, "server")
		}
		if consumed && (use.serves || (all && !inUse)) {
			roles = append(roles, "client")
		}
	case KindAction:
		if provided && (use.callsAction || (all && !inUse)) {
			roles = append(roles, "action_server")
		}
		if consumed && (use.servesAction || (all && !inUse)) {
			roles = append(roles, "action_client")
		}
	}
	return roles
}

// othersAmong reports whether anyone outside this application is in the list.
func othersAmong(nodes []string, ours map[string]bool) bool {
	for _, n := range nodes {
		if !ours[n] {
			return true
		}
	}
	return false
}

// qosFor is the profile to declare. Services and actions carry none, matching
// the externals schema.
func qosFor(i *Interface, role string) string {
	if i.Kind != KindTopic {
		return ""
	}
	return i.QoS
}

func describe(i *Interface, role string) string {
	switch role {
	case "publisher", "subscriber":
		return fmt.Sprintf("topic %q", i.Name)
	case "server", "client":
		return fmt.Sprintf("service %q", i.Name)
	default:
		return fmt.Sprintf("action %q", i.Name)
	}
}

// providedBy names the peers, because "who is publishing this?" is the next
// question a missing declaration raises.
func providedBy(i *Interface, role string) string {
	var who []string
	var verb string
	switch role {
	case "publisher":
		who, verb = i.Publishers, "published by"
	case "subscriber":
		who, verb = i.Subscribers, "subscribed by"
	case "server", "action_server":
		who, verb = i.Servers, "served by"
	default:
		who, verb = i.Clients, "called by"
	}
	if len(who) == 0 {
		return "nobody is on it"
	}
	return verb + " " + strings.Join(who, ", ")
}

func qosNote(qos string) string {
	if qos == "" {
		return ""
	}
	return ", qos " + qos
}

func shortHashes(hashes []string) []string {
	out := make([]string, 0, len(hashes))
	for _, h := range hashes {
		trimmed := strings.TrimPrefix(h, "RIHS01_")
		if len(trimmed) > 12 {
			trimmed = trimmed[:12] + "…"
		}
		out = append(out, trimmed)
	}
	return out
}

func key(topic, role string) string { return topic + "\x00" + role }

// Render is the externals block as it would appear in conductor.json.
func Render(externals []scan.External) ([]byte, error) {
	return json.MarshalIndent(map[string]any{"externals": externals}, "", "  ")
}

// Write replaces the externals block in the application's conductor.json,
// leaving every other key — and the file's own ordering — as it was. A
// generated file that discards the author's other settings would be a poor
// trade for the transcription it saves.
func Write(app *scan.App, externals []scan.External) error {
	// An environment may add externals or drop them (`without`), and the list
	// this command sees is the merged one. Writing that back would flatten the
	// overlay into the base file — quietly making a simulation's stand-in
	// drivers part of every environment.
	if app.Env != nil && (len(app.Env.Externals) > 0 || len(app.Env.Without) > 0) {
		return fmt.Errorf("environment %q overlays the externals block (%d added, %d dropped), "+
			"and writing would flatten that into conductor.json; run without -env to update the "+
			"base list, or copy the entries you want into environments.json",
			app.Env.Name(), len(app.Env.Externals), len(app.Env.Without))
	}
	path := filepath.Join(app.Dir, "conductor.json")
	raw, err := os.ReadFile(path)
	if err != nil {
		return err
	}
	// Decoding into ordered pairs keeps "app" before "externals" and any key
	// this version of conductor does not know about.
	var doc map[string]json.RawMessage
	if err := json.Unmarshal(raw, &doc); err != nil {
		return fmt.Errorf("%s: %w", path, err)
	}
	block, err := json.Marshal(externals)
	if err != nil {
		return err
	}
	doc["externals"] = block

	var out bytes.Buffer
	out.WriteString("{\n")
	keys := make([]string, 0, len(doc))
	for k := range doc {
		keys = append(keys, k)
	}
	sort.Slice(keys, func(a, b int) bool { return fieldOrder(keys[a]) < fieldOrder(keys[b]) })
	for n, k := range keys {
		var indented bytes.Buffer
		if err := json.Indent(&indented, doc[k], "  ", "  "); err != nil {
			return err
		}
		fmt.Fprintf(&out, "  %q: %s", k, indented.String())
		if n < len(keys)-1 {
			out.WriteString(",")
		}
		out.WriteString("\n")
	}
	out.WriteString("}\n")
	return os.WriteFile(path, out.Bytes(), 0o644)
}

// fieldOrder keeps conductor.json readable: the application's name first, its
// externals last, anything else in between and alphabetical.
func fieldOrder(key string) string {
	switch key {
	case "app":
		return "0"
	case "externals":
		return "2"
	default:
		return "1" + key
	}
}

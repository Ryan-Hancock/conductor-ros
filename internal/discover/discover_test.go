package discover

import (
	"encoding/json"
	"os"
	"path/filepath"
	"strings"
	"testing"

	conductor "conductor.dev/conductor"
	"conductor.dev/conductor/internal/scan"
	"conductor.dev/conductor/transport/rmwzenoh"
)

// The graphs in these tests are built the way a peer would advertise itself —
// as liveliness tokens, parsed back — so the derivation is exercised through
// the same path a live query takes, without needing a router.
func graphOf(t *testing.T, tokens ...string) *Graph {
	t.Helper()
	g := &Graph{Domain: 0, Endpoint: "test"}
	for _, tok := range tokens {
		e, err := rmwzenoh.ParseToken(tok)
		if err != nil {
			t.Fatalf("token %q: %v", tok, err)
		}
		g.Entities = append(g.Entities, e)
	}
	return g
}

func node(name string) string {
	return rmwzenoh.NodeToken(0, "zid"+name, 0, 0, name)
}

func endpoint(kind, nodeName, topic, typeName, profile string) string {
	q, ok := conductor.QoSProfile(profile)
	if !ok {
		panic("unknown profile " + profile)
	}
	parts := strings.Split(typeName, "/")
	dds := parts[0] + "::" + parts[1] + "::dds_::" + parts[2] + "_"
	return rmwzenoh.EndpointToken(0, "zid"+nodeName, 0, 1, kind, nodeName,
		topic, dds, "RIHS01_"+parts[2], q)
}

// An action is seven endpoints on the wire and one interface to everyone else.
func TestActionsRollUp(t *testing.T) {
	g := graphOf(t,
		node("bt_navigator"),
		endpoint(rmwzenoh.EntityService, "bt_navigator", "/navigate_to_pose/_action/send_goal",
			"nav2_msgs/action/NavigateToPose_SendGoal", "reliable"),
		endpoint(rmwzenoh.EntityService, "bt_navigator", "/navigate_to_pose/_action/get_result",
			"nav2_msgs/action/NavigateToPose_GetResult", "reliable"),
		endpoint(rmwzenoh.EntityService, "bt_navigator", "/navigate_to_pose/_action/cancel_goal",
			"action_msgs/srv/CancelGoal", "reliable"),
		endpoint(rmwzenoh.EntityPublisher, "bt_navigator", "/navigate_to_pose/_action/feedback",
			"nav2_msgs/action/NavigateToPose_FeedbackMessage", "reliable"),
		endpoint(rmwzenoh.EntityPublisher, "bt_navigator", "/navigate_to_pose/_action/status",
			"action_msgs/msg/GoalStatusArray", "transient"),
	)

	interfaces := g.Interfaces()
	if len(interfaces) != 1 {
		for _, i := range interfaces {
			t.Logf("  %s %s %s", i.Kind, i.Name, i.Type)
		}
		t.Fatalf("got %d interfaces, want one action", len(interfaces))
	}
	got := interfaces[0]
	if got.Kind != KindAction || got.Name != "navigate_to_pose" {
		t.Fatalf("got %s %q", got.Kind, got.Name)
	}
	if got.Type != "nav2_msgs/action/NavigateToPose" {
		t.Errorf("type = %q, want the action's own name", got.Type)
	}
	if len(got.Servers) != 1 || got.Servers[0] != "bt_navigator" {
		t.Errorf("servers = %v", got.Servers)
	}
}

// The endpoints every ROS node carries are recognised by type, so a namespaced
// node's parameter services are still noise.
func TestInfrastructureIsRecognisedByType(t *testing.T) {
	g := graphOf(t,
		node("amcl"),
		endpoint(rmwzenoh.EntityService, "amcl", "/robot_1/amcl/get_parameters",
			"rcl_interfaces/srv/GetParameters", "reliable"),
		endpoint(rmwzenoh.EntityPublisher, "amcl", "/rosout", "rcl_interfaces/msg/Log", "transient"),
		endpoint(rmwzenoh.EntityPublisher, "amcl", "/amcl_pose",
			"geometry_msgs/msg/PoseWithCovarianceStamped", "transient"),
	)
	for _, i := range g.Interfaces() {
		want := i.Name == "amcl_pose"
		if i.Infrastructure == want {
			t.Errorf("%s %q: infrastructure = %v", i.Kind, i.Name, i.Infrastructure)
		}
	}
}

// patrolApp is an application that subscribes to a topic somebody else
// publishes, publishes one somebody else consumes, calls a service and drives
// an action — one of each role the externals block can hold.
func patrolApp(externals ...scan.External) *scan.App {
	return &scan.App{
		Name: "patrol", Dir: "/src/patrol",
		Externals: externals,
		Nodes: []*scan.Node{{
			Name: "commander",
			Subs: []scan.Endpoint{{Topic: "amcl_pose"}},
			Pubs: []scan.Endpoint{{Topic: "diagnostics"}},
			Clients: []scan.ServiceEndpoint{
				{Service: "lifecycle_manager_navigation/manage_nodes"},
			},
			ActionClients: []scan.ActionEndpoint{{Action: "navigate_to_pose"}},
		}},
	}
}

func liveStack(t *testing.T) *Graph {
	return graphOf(t,
		node("amcl"), node("bt_navigator"), node("aggregator"),
		endpoint(rmwzenoh.EntityPublisher, "amcl", "/amcl_pose",
			"geometry_msgs/msg/PoseWithCovarianceStamped", "transient"),
		endpoint(rmwzenoh.EntitySubscription, "aggregator", "/diagnostics",
			"diagnostic_msgs/msg/DiagnosticArray", "reliable"),
		endpoint(rmwzenoh.EntityService, "bt_navigator", "/lifecycle_manager_navigation/manage_nodes",
			"nav2_msgs/srv/ManageLifecycleNodes", "reliable"),
		endpoint(rmwzenoh.EntityService, "bt_navigator", "/navigate_to_pose/_action/send_goal",
			"nav2_msgs/action/NavigateToPose_SendGoal", "reliable"),
	)
}

// With nothing declared, every interface the application uses comes back with
// the role that describes the outside: our subscription needs a publisher, our
// publisher needs a subscriber, our client needs a server.
func TestRolesDescribeTheOutside(t *testing.T) {
	r := Externals(patrolApp(), liveStack(t), Options{})

	want := map[string]string{
		"amcl_pose":   "publisher",
		"diagnostics": "subscriber",
		"lifecycle_manager_navigation/manage_nodes": "server",
		"navigate_to_pose":                          "action_server",
	}
	if len(r.Externals) != len(want) {
		t.Fatalf("derived %d externals, want %d: %+v", len(r.Externals), len(want), r.Externals)
	}
	for _, e := range r.Externals {
		if role, ok := want[e.Topic]; !ok || role != e.Role {
			t.Errorf("%s: role %q, want %q", e.Topic, e.Role, role)
		}
	}
	// All four are missing from the declarations, and each says who provides
	// it — the next question anyone asks.
	if len(r.Findings) != 4 {
		t.Fatalf("findings = %d, want one per undeclared interface: %+v", len(r.Findings), r.Findings)
	}
	for _, f := range r.Findings {
		if f.Kind != FindingMissing {
			t.Errorf("%s: kind %q, want missing", f.Topic, f.Kind)
		}
	}
	if !strings.Contains(findingFor(r, "amcl_pose").Message, "published by amcl") {
		t.Errorf("missing finding does not name the publisher: %q", findingFor(r, "amcl_pose").Message)
	}
	if !r.Changed() {
		t.Error("Changed() is false with four undeclared externals")
	}
}

// A declaration that disagrees with the graph is the case that matters: this
// is the silence the checker cannot otherwise catch, because it is checking
// the wrong claim.
func TestMismatchedTypeAndQoS(t *testing.T) {
	r := Externals(patrolApp(
		scan.External{Topic: "amcl_pose", Type: "geometry_msgs/msg/PoseStamped", Role: "publisher", QoS: "transient"},
		scan.External{Topic: "diagnostics", Type: "diagnostic_msgs/msg/DiagnosticArray", Role: "subscriber", QoS: "sensor"},
		scan.External{Topic: "lifecycle_manager_navigation/manage_nodes",
			Type: "nav2_msgs/srv/ManageLifecycleNodes", Role: "server"},
		scan.External{Topic: "navigate_to_pose", Type: "nav2_msgs/action/NavigateToPose", Role: "action_server"},
	), liveStack(t), Options{})

	typeFinding := findingFor(r, "amcl_pose")
	if typeFinding.Kind != FindingMismatch {
		t.Fatalf("amcl_pose finding = %+v, want a type mismatch", typeFinding)
	}
	if !strings.Contains(typeFinding.Message, "PoseWithCovarianceStamped") {
		t.Errorf("mismatch does not name the live type: %q", typeFinding.Message)
	}
	qosFinding := findingFor(r, "diagnostics")
	if qosFinding.Kind != FindingMismatch || !strings.Contains(qosFinding.Message, "reliable") {
		t.Fatalf("diagnostics finding = %+v, want a qos mismatch naming reliable", qosFinding)
	}

	// The merged block carries the graph's answer, not the declaration's.
	for _, e := range r.Externals {
		if e.Topic == "amcl_pose" && e.Type != "geometry_msgs/msg/PoseWithCovarianceStamped" {
			t.Errorf("merged type = %q, want the graph's", e.Type)
		}
	}
}

// declaredForLiveStack is what the application would declare if it had been
// written against the graph liveStack describes.
func declaredForLiveStack() []scan.External {
	return []scan.External{
		{Topic: "amcl_pose", Type: "geometry_msgs/msg/PoseWithCovarianceStamped",
			Role: "publisher", QoS: "transient"},
		{Topic: "diagnostics", Type: "diagnostic_msgs/msg/DiagnosticArray",
			Role: "subscriber", QoS: "reliable"},
		{Topic: "lifecycle_manager_navigation/manage_nodes",
			Type: "nav2_msgs/srv/ManageLifecycleNodes", Role: "server"},
		{Topic: "navigate_to_pose", Type: "nav2_msgs/action/NavigateToPose", Role: "action_server"},
	}
}

// Declarations that match the graph produce no findings at all, which is the
// state a committed conductor.json is meant to be in.
func TestMatchingDeclarationsAreSilent(t *testing.T) {
	r := Externals(patrolApp(declaredForLiveStack()...), liveStack(t), Options{})
	if len(r.Findings) != 0 {
		t.Fatalf("findings against a matching declaration: %+v", r.Findings)
	}
	if r.Changed() {
		t.Error("Changed() is true with nothing to change")
	}
}

// Something declared that nobody offers is reported and kept: half a stack
// being up is the normal case while developing, and deleting the declaration
// would be the tool overstepping.
func TestAbsentDeclarationIsKeptAndReported(t *testing.T) {
	declared := scan.External{Topic: "battery_state", Type: "sensor_msgs/msg/BatteryState",
		Role: "publisher", QoS: "sensor"}
	r := Externals(patrolApp(append(declaredForLiveStack(), declared)...), liveStack(t), Options{})

	f := findingFor(r, "battery_state")
	if f.Kind != FindingAbsent {
		t.Fatalf("battery_state finding = %+v, want absent", f)
	}
	var kept bool
	for _, e := range r.Externals {
		if e == declared {
			kept = true
		}
	}
	if !kept {
		t.Error("an absent declaration was dropped from the merged block")
	}
	if r.Changed() {
		t.Error("Changed() is true for an absent declaration; there is nothing to write")
	}
}

// Our own nodes are on the graph too. Declaring them external would tell the
// checker that this application's own topics come from somewhere else.
func TestOurOwnEndpointsAreNotExternal(t *testing.T) {
	app := patrolApp()
	g := graphOf(t,
		node("commander"), // us
		endpoint(rmwzenoh.EntityPublisher, "commander", "/diagnostics",
			"diagnostic_msgs/msg/DiagnosticArray", "reliable"),
		endpoint(rmwzenoh.EntitySubscription, "commander", "/amcl_pose",
			"geometry_msgs/msg/PoseWithCovarianceStamped", "transient"),
	)
	r := Externals(app, g, Options{})

	if len(r.Externals) != 0 {
		t.Fatalf("derived %+v from a graph containing only ourselves", r.Externals)
	}
	if len(r.Ours) != 1 || r.Ours[0] != "commander" {
		t.Errorf("Ours = %v, want the node we recognised as our own", r.Ours)
	}
}

// Publishers that disagree about QoS make "the" profile a question. The
// weakest offer is declared, because that is the one a subscriber has to match
// to hear all of them — and picking any other would make the output depend on
// discovery order.
func TestDisagreeingPublishersTakeTheWeakestOffer(t *testing.T) {
	g := graphOf(t,
		node("driver"), node("replay"),
		endpoint(rmwzenoh.EntityPublisher, "driver", "/scan", "sensor_msgs/msg/LaserScan", "reliable"),
		endpoint(rmwzenoh.EntityPublisher, "replay", "/scan", "sensor_msgs/msg/LaserScan", "sensor"),
	)
	app := &scan.App{Name: "app", Nodes: []*scan.Node{{
		Name: "perception", Subs: []scan.Endpoint{{Topic: "scan"}},
	}}}

	r := Externals(app, g, Options{})
	if len(r.Externals) != 1 || r.Externals[0].QoS != "sensor" {
		t.Fatalf("derived %+v, want scan declared with the weakest offer (sensor)", r.Externals)
	}
	var conflict bool
	for _, f := range r.Findings {
		if f.Kind == FindingConflict && strings.Contains(f.Message, "reliable") {
			conflict = true
		}
	}
	if !conflict {
		t.Errorf("the disagreement was not reported: %+v", r.Findings)
	}
}

// Two peers advertising different type hashes for one interface are running
// different definitions of it — a real fault, and invisible to a type-name
// comparison.
func TestHashConflictIsReported(t *testing.T) {
	q, _ := conductor.QoSProfile("reliable")
	g := graphOf(t,
		node("old"), node("new"),
		rmwzenoh.EndpointToken(0, "zidold", 0, 1, rmwzenoh.EntityPublisher, "old",
			"/status", "patrol_msgs::msg::dds_::Status_", "RIHS01_aaaa1111", q),
		rmwzenoh.EndpointToken(0, "zidnew", 0, 1, rmwzenoh.EntityPublisher, "new",
			"/status", "patrol_msgs::msg::dds_::Status_", "RIHS01_bbbb2222", q),
	)
	app := &scan.App{Name: "app", Nodes: []*scan.Node{{
		Name: "watcher", Subs: []scan.Endpoint{{Topic: "status"}},
	}}}

	r := Externals(app, g, Options{})
	var found bool
	for _, f := range r.Findings {
		if f.Kind == FindingConflict && strings.Contains(f.Message, "different type hashes") {
			found = true
		}
	}
	if !found {
		t.Fatalf("no hash conflict reported: %+v", r.Findings)
	}
}

// -all is for looking at a stack you are adopting: everything on the graph,
// including what this application does not touch yet.
func TestAllIncludesInterfacesWeDoNotUse(t *testing.T) {
	g := liveStack(t)
	app := &scan.App{Name: "app", Nodes: []*scan.Node{{Name: "commander"}}}

	if r := Externals(app, g, Options{}); len(r.Externals) != 0 {
		t.Fatalf("default derived %+v for an application that uses nothing", r.Externals)
	}
	r := Externals(app, g, Options{All: true})
	if len(r.Externals) != 4 {
		t.Fatalf("-all derived %d externals, want every interface on the graph: %+v",
			len(r.Externals), r.Externals)
	}
}

// Write replaces the externals block and leaves everything else in
// conductor.json alone, including keys this version does not know about.
func TestWritePreservesTheRestOfTheFile(t *testing.T) {
	dir := t.TempDir()
	original := `{
  "app": "patrol",
  "something_new": {"kept": true},
  "externals": [
    {"topic": "old", "type": "std_msgs/msg/Bool", "role": "publisher", "qos": "reliable"}
  ]
}
`
	path := filepath.Join(dir, "conductor.json")
	if err := os.WriteFile(path, []byte(original), 0o644); err != nil {
		t.Fatal(err)
	}
	// The scanner looks for the module root, so the fixture needs one.
	if err := os.WriteFile(filepath.Join(dir, "go.mod"), []byte("module fixture\n\ngo 1.24\n"), 0o644); err != nil {
		t.Fatal(err)
	}

	app := &scan.App{Name: "patrol", Dir: dir}
	err := Write(app, []scan.External{
		{Topic: "amcl_pose", Type: "geometry_msgs/msg/PoseWithCovarianceStamped",
			Role: "publisher", QoS: "transient"},
		{Topic: "navigate_to_pose", Type: "nav2_msgs/action/NavigateToPose", Role: "action_server"},
	})
	if err != nil {
		t.Fatal(err)
	}

	written, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	var doc struct {
		App          string              `json:"app"`
		SomethingNew struct{ Kept bool } `json:"something_new"`
		Externals    []scan.External     `json:"externals"`
	}
	if err := json.Unmarshal(written, &doc); err != nil {
		t.Fatalf("the written file does not parse: %v\n%s", err, written)
	}
	if doc.App != "patrol" {
		t.Errorf("app = %q", doc.App)
	}
	if !doc.SomethingNew.Kept {
		t.Error("an unknown key was dropped by the rewrite")
	}
	if len(doc.Externals) != 2 || doc.Externals[0].Topic != "amcl_pose" {
		t.Fatalf("externals = %+v", doc.Externals)
	}
	// An action has no QoS, and the file should not claim one.
	if strings.Contains(string(written), `"qos": ""`) {
		t.Errorf("the rewrite wrote an empty qos:\n%s", written)
	}
	// The rescanned app must agree with what we asked for, which is the real
	// contract: the file is read back by the toolchain, not by a human.
	rescanned, err := scan.ScanApp(dir)
	if err != nil {
		t.Fatalf("rescanning the written app: %v", err)
	}
	if len(rescanned.Externals) != 2 {
		t.Fatalf("rescan found %d externals", len(rescanned.Externals))
	}
	if rescanned.Externals[1].Type != "nav2_msgs/action/NavigateToPose" {
		t.Errorf("rescanned %+v", rescanned.Externals[1])
	}
}

// The block keeps the order the file had, with new entries appended: a tool
// that reshuffles a hand-maintained file to say the same thing produces a diff
// nobody can review.
func TestWriteOrderFollowsTheFile(t *testing.T) {
	declared := declaredForLiveStack()
	reversed := []scan.External{declared[3], declared[2], declared[1], declared[0]}
	// One entry the graph will supply that the file does not have.
	app := patrolApp(reversed...)
	app.Nodes[0].Subs = append(app.Nodes[0].Subs, scan.Endpoint{Topic: "scan"})
	g := liveStack(t)
	g.Entities = append(g.Entities, mustParse(t,
		endpoint(rmwzenoh.EntityPublisher, "lidar", "/scan", "sensor_msgs/msg/LaserScan", "sensor")))

	r := Externals(app, g, Options{})
	if len(r.Externals) != 5 {
		t.Fatalf("derived %d externals, want the four declared plus one new", len(r.Externals))
	}
	for i, want := range reversed {
		if r.Externals[i].Topic != want.Topic {
			t.Errorf("position %d = %q, want the file's %q", i, r.Externals[i].Topic, want.Topic)
		}
	}
	if last := r.Externals[4]; last.Topic != "scan" {
		t.Errorf("the new entry is at %q, want it appended", last.Topic)
	}
}

// An environment can add externals and drop them, and the list this package
// sees is already merged — so writing it back would flatten the overlay into
// the base file. That is refused rather than done quietly.
func TestWriteRefusesToFlattenAnEnvironmentOverlay(t *testing.T) {
	dir := t.TempDir()
	if err := os.WriteFile(filepath.Join(dir, "conductor.json"),
		[]byte(`{"app":"patrol","externals":[]}`), 0o644); err != nil {
		t.Fatal(err)
	}
	app := &scan.App{Name: "patrol", Dir: dir, Env: &scan.Environment{
		Externals: []scan.External{{Topic: "battery_state", Type: "sensor_msgs/msg/BatteryState",
			Role: "publisher", QoS: "sensor"}},
	}}

	err := Write(app, []scan.External{{Topic: "amcl_pose", Role: "publisher"}})
	if err == nil {
		t.Fatal("an environment overlay was flattened into conductor.json")
	}
	if !strings.Contains(err.Error(), "flatten") || !strings.Contains(err.Error(), "environments.json") {
		t.Errorf("error does not explain the problem or the fix: %v", err)
	}
}

// A conductor.Lifecycle field names nodes in other people's processes, so the
// checker cannot verify them — but a live graph can, and a name that is not
// there is a stack that will never come up.
func TestManagedNodesAreCheckedAgainstTheGraph(t *testing.T) {
	lifecycleOf := func(node string) string {
		return endpoint(rmwzenoh.EntityService, node, "/"+node+"/change_state",
			"lifecycle_msgs/srv/ChangeState", "reliable")
	}
	g := graphOf(t,
		node("amcl"), node("bt_navigator"),
		lifecycleOf("amcl"), lifecycleOf("bt_navigator"),
	)
	app := &scan.App{Name: "app", Nodes: []*scan.Node{{
		Name: "commander",
		Lifecycle: []scan.LifecycleDecl{{
			Field: "Stack", Nodes: []string{"amcl", "bt_navigator", "planner_server"},
		}},
	}}}

	r := Externals(app, g, Options{})
	f := findingFor(r, "planner_server")
	if f.Kind != FindingAbsent {
		t.Fatalf("planner_server finding = %+v, want absent", f)
	}
	if !strings.Contains(f.Message, "change_state") || !strings.Contains(f.Message, "bt_navigator") {
		t.Errorf("finding neither says what is missing nor what is there: %q", f.Message)
	}
	// The two that are there are not complained about.
	for _, name := range []string{"amcl", "bt_navigator"} {
		if got := findingFor(r, name); got.Kind != "" {
			t.Errorf("%s: unexpected %+v", name, got)
		}
	}
}

// A graph with no lifecycle services at all is a stack that is down, which is
// one fact, not one per declared node.
func TestManagedNodesOnADownStack(t *testing.T) {
	app := &scan.App{Name: "app", Nodes: []*scan.Node{{
		Name: "commander",
		Lifecycle: []scan.LifecycleDecl{{
			Field: "Stack", Nodes: []string{"amcl", "bt_navigator", "planner_server"},
		}},
	}}}

	r := Externals(app, &Graph{}, Options{})
	if len(r.Findings) != 1 {
		t.Fatalf("findings = %+v, want one saying the stack is not running", r.Findings)
	}
	if !strings.Contains(r.Findings[0].Message, "not running") {
		t.Errorf("finding = %q", r.Findings[0].Message)
	}
}

func mustParse(t *testing.T, token string) rmwzenoh.Entity {
	t.Helper()
	e, err := rmwzenoh.ParseToken(token)
	if err != nil {
		t.Fatal(err)
	}
	return e
}

func findingFor(r *Report, topic string) Finding {
	for _, f := range r.Findings {
		if f.Topic == topic {
			return f
		}
	}
	return Finding{}
}

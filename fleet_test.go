package conductor

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"
)

// fakePeer serves a per-process summary the way a running conductor process
// does, so the merge is exercised over the real fetch path.
func fakePeer(t *testing.T, state DashboardState) *httptest.Server {
	t.Helper()
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/api/summary" {
			http.NotFound(w, r)
			return
		}
		state.Now = time.Now()
		json.NewEncoder(w).Encode(state)
	}))
	t.Cleanup(srv.Close)
	return srv
}

// process is the summary of one node running alone, as a per-node deployment
// makes them.
func process(node string, topics []TopicView) DashboardState {
	return DashboardState{
		App:    AppView{Name: "patrol", Transport: "zenoh", Host: "robot-1", Bringup: []string{node}},
		Nodes:  []NodeView{{Name: node, State: "active", Processed: 10}},
		Topics: topics,
	}
}

func pub(topic, typ, qos string, node string, sent uint64) TopicView {
	return TopicView{Name: topic, Type: typ, QoS: qos, Pubs: []string{node}, Subs: []string{}, Sent: sent}
}

func sub(topic, typ, qos string, node string, received uint64) TopicView {
	return TopicView{Name: topic, Type: typ, QoS: qos, Pubs: []string{}, Subs: []string{node}, Recvd: received}
}

func fleetOf(t *testing.T, opts FleetOptions, peers ...Peer) FleetState {
	t.Helper()
	return FleetWith(context.Background(), peers, time.Second, opts)
}

func findings(s FleetState, code string) []Finding {
	var out []Finding
	for _, f := range s.Findings {
		if f.Code == code {
			out = append(out, f)
		}
	}
	return out
}

// The point of the merged view: an edge whose ends are in different processes
// is an edge, not "(ROS graph)" twice.
func TestFleetJoinsCrossProcessEdges(t *testing.T) {
	a := fakePeer(t, process("localizer", []TopicView{pub("amcl_pose", "geometry_msgs/msg/PoseStamped", "reliable", "localizer", 100)}))
	b := fakePeer(t, process("navigator", []TopicView{
		sub("amcl_pose", "geometry_msgs/msg/PoseStamped", "reliable", "navigator", 98),
		pub("cmd_vel", "geometry_msgs/msg/Twist", "reliable", "navigator", 98),
	}))

	s := fleetOf(t, FleetOptions{},
		Peer{Name: "localizer", Host: "robot-1", URL: a.URL},
		Peer{Name: "navigator", Host: "robot-1", URL: b.URL})

	if s.Reachable != 2 {
		t.Fatalf("reachable = %d, want 2 (%v)", s.Reachable, s.Findings)
	}
	var pose *FleetTopic
	for i := range s.Topics {
		if s.Topics[i].Name == "amcl_pose" {
			pose = &s.Topics[i]
		}
	}
	if pose == nil {
		t.Fatalf("amcl_pose missing from %v", s.Topics)
	}
	if len(pose.Pubs) != 1 || pose.Pubs[0].Key != "robot-1/localizer" {
		t.Fatalf("publishers = %+v, want the localizer's process", pose.Pubs)
	}
	if len(pose.Subs) != 1 || pose.Subs[0].Key != "robot-1/navigator" {
		t.Fatalf("subscribers = %+v, want the navigator's process", pose.Subs)
	}
	if pose.Sent != 100 || pose.Recvd != 98 {
		t.Fatalf("counters = %d sent, %d received", pose.Sent, pose.Recvd)
	}
	// The bringup order is derived from the merged graph, across processes.
	if got := strings.Join(s.Order, " -> "); got != "robot-1/localizer -> robot-1/navigator" {
		t.Fatalf("order = %s", got)
	}
	if len(s.Findings) != 0 {
		t.Fatalf("healthy fleet reported %v", s.Findings)
	}
}

// A process that does not answer is the finding, and it keeps its place in
// the graph so the hole is visible.
func TestFleetReportsProcessesThatAreDown(t *testing.T) {
	up := fakePeer(t, process("navigator", []TopicView{
		sub("amcl_pose", "geometry_msgs/msg/PoseStamped", "reliable", "navigator", 5),
	}))
	// A port nothing listens on: httptest hands one over and closes it.
	dead := httptest.NewServer(http.NotFoundHandler())
	deadURL := dead.URL
	dead.Close()

	s := fleetOf(t, FleetOptions{},
		Peer{Name: "localizer", Host: "robot-1", URL: deadURL},
		Peer{Name: "navigator", Host: "robot-1", URL: up.URL})

	if s.Reachable != 1 {
		t.Fatalf("reachable = %d, want 1", s.Reachable)
	}
	down := findings(s, "FLEET01")
	if len(down) != 1 || !strings.Contains(down[0].Msg, "robot-1/localizer") {
		t.Fatalf("FLEET01 = %v, want the localizer named", down)
	}
	// And the consequence: the navigator's topic has nobody publishing it.
	if got := findings(s, "FLEET04"); len(got) != 1 || !strings.Contains(got[0].Msg, "amcl_pose") {
		t.Fatalf("FLEET04 = %v, want amcl_pose reported as unpublished", got)
	}
	var found bool
	for _, n := range s.Order {
		if n == "robot-1/localizer" {
			found = true
		}
	}
	if !found {
		t.Fatalf("a process that is down vanished from the graph: %v", s.Order)
	}
}

// A topic the application declares as external is somebody else's to publish,
// so it is not a missing publisher.
func TestFleetDoesNotBlameExternalTopics(t *testing.T) {
	p := fakePeer(t, process("safety_monitor", []TopicView{
		sub("estop", "std_msgs/msg/Bool", "transient", "safety_monitor", 0),
	}))
	peer := Peer{Name: "safety_monitor", Host: "robot-1", URL: p.URL}

	if got := findings(fleetOf(t, FleetOptions{}, peer), "FLEET04"); len(got) != 1 {
		t.Fatalf("without the declaration, an unpublished topic should be reported: %v", got)
	}
	s := fleetOf(t, FleetOptions{External: map[string]bool{"estop": true}}, peer)
	if got := findings(s, "FLEET04"); len(got) != 0 {
		t.Fatalf("declared external topic reported as unpublished: %v", got)
	}
	if !s.Topics[0].External {
		t.Fatal("the topic is not marked external in the view")
	}
}

// Two processes that disagree about a topic is what a deployment caught
// halfway between two releases looks like from the outside.
func TestFleetFindsPartialDeploys(t *testing.T) {
	old := fakePeer(t, process("localizer", []TopicView{
		pub("amcl_pose", "geometry_msgs/msg/Pose", "reliable", "localizer", 1),
	}))
	new := fakePeer(t, process("navigator", []TopicView{
		sub("amcl_pose", "geometry_msgs/msg/PoseStamped", "reliable", "navigator", 1),
	}))

	s := fleetOf(t, FleetOptions{},
		Peer{Name: "localizer", Host: "robot-1", URL: old.URL},
		Peer{Name: "navigator", Host: "robot-1", URL: new.URL})

	got := findings(s, "FLEET05")
	if len(got) != 1 || !strings.Contains(got[0].Msg, "type") {
		t.Fatalf("FLEET05 = %v, want a type disagreement on amcl_pose", got)
	}
}

// Two robots reporting different transform trees means one of them is running
// an older release — or a different calibration.
func TestFleetFindsDivergentTransformTrees(t *testing.T) {
	tree := func(x float64) []FrameView {
		return []FrameView{{Parent: "base_link", Child: "laser", XYZ: [3]float64{x, 0, 0.19}}}
	}
	a := process("localizer", nil)
	a.Frames = tree(0.12)
	b := process("navigator", nil)
	b.Frames = tree(0.118)

	s := fleetOf(t, FleetOptions{},
		Peer{Name: "localizer", Host: "robot-1", URL: fakePeer(t, a).URL},
		Peer{Name: "navigator", Host: "robot-2", URL: fakePeer(t, b).URL})

	got := findings(s, "FLEET06")
	if len(got) != 1 || !strings.Contains(got[0].Msg, "robot-2/navigator") {
		t.Fatalf("FLEET06 = %v, want the divergent peer named", got)
	}
}

// A node that is not Active, or one dropping messages, is worth saying across
// a deployment where nobody is watching each journal.
func TestFleetReportsUnhealthyNodes(t *testing.T) {
	state := process("navigator", nil)
	state.Nodes = []NodeView{{Name: "navigator", State: "inactive", Dropped: 3}}
	s := fleetOf(t, FleetOptions{}, Peer{Name: "navigator", Host: "robot-1", URL: fakePeer(t, state).URL})

	if got := findings(s, "FLEET02"); len(got) != 1 {
		t.Fatalf("FLEET02 = %v, want the inactive node reported", got)
	}
	if got := findings(s, "FLEET03"); len(got) != 1 || !strings.Contains(got[0].Msg, "3") {
		t.Fatalf("FLEET03 = %v, want the dropped messages reported", got)
	}
}

// A per-node deployment labels each process after the node it runs; the
// merged graph must not read "robot-1/navigator/navigator".
func TestFleetNodeKeys(t *testing.T) {
	cases := [][3]string{
		{"robot-1/navigator", "navigator", "robot-1/navigator"},
		{"patrol", "navigator", "patrol/navigator"},
		{"navigator", "navigator", "navigator"},
	}
	for _, c := range cases {
		if got := nodeKey(c[0], c[1]); got != c[2] {
			t.Errorf("nodeKey(%q, %q) = %q, want %q", c[0], c[1], got, c[2])
		}
	}
}

// The served page and API: JSON that the page can rely on, and a page that
// depends on nothing it has to fetch.
func TestServeFleet(t *testing.T) {
	p := fakePeer(t, process("navigator", []TopicView{pub("cmd_vel", "geometry_msgs/msg/Twist", "reliable", "navigator", 1)}))
	srv, err := ServeFleet("127.0.0.1:0", []Peer{{Name: "navigator", URL: p.URL}}, time.Second, FleetOptions{})
	if err != nil {
		t.Fatal(err)
	}
	defer srv.Close()

	// ServeFleet listens before returning, so the address is bound; ask the
	// server which port it got by dialling through its own handler.
	rec := httptest.NewRecorder()
	srv.Handler.ServeHTTP(rec, httptest.NewRequest(http.MethodGet, "/api/fleet", nil))
	if rec.Code != http.StatusOK {
		t.Fatalf("api status %d", rec.Code)
	}
	var state FleetState
	if err := json.Unmarshal(rec.Body.Bytes(), &state); err != nil {
		t.Fatalf("api response: %v", err)
	}
	if state.Reachable != 1 {
		t.Fatalf("reachable = %d", state.Reachable)
	}

	rec = httptest.NewRecorder()
	srv.Handler.ServeHTTP(rec, httptest.NewRequest(http.MethodGet, "/", nil))
	page := rec.Body.String()
	if rec.Code != http.StatusOK || !strings.Contains(page, "conductor") {
		t.Fatalf("page status %d", rec.Code)
	}
	for _, forbidden := range []string{"http://", "https://", `src="//`} {
		if strings.Contains(page, forbidden) {
			t.Errorf("the fleet page references %q; it must be self-contained "+
				"(a robot's network has no route to a CDN)", forbidden)
		}
	}
}

// Every list the page iterates must be a list, never null — the failure mode
// that takes a dashboard down is a nil slice in the JSON.
func TestFleetStateHasNoNullLists(t *testing.T) {
	s := fleetOf(t, FleetOptions{}, Peer{Name: "gone", URL: "http://127.0.0.1:1"})
	b, err := json.Marshal(s)
	if err != nil {
		t.Fatal(err)
	}
	var raw map[string]any
	if err := json.Unmarshal(b, &raw); err != nil {
		t.Fatal(err)
	}
	for _, key := range []string{"processes", "bringup_order", "topics", "missions", "frames", "findings"} {
		if raw[key] == nil {
			t.Errorf("%s is null in the fleet JSON", key)
		}
	}
	for _, p := range s.Processes {
		if p.Nodes == nil || p.Frames == nil {
			t.Errorf("process %s has null lists: %+v", p.Name, p)
		}
	}
}

// Two robots run the same application on two ROS graphs. Their topics share
// names and nothing else, so merging them into one would draw edges between
// machines that cannot talk.
func TestFleetKeepsRobotsApart(t *testing.T) {
	peer := func(node, robot string, topics []TopicView) Peer {
		s := process(node, topics)
		return Peer{Name: node, Host: robot, Robot: robot, URL: fakePeer(t, s).URL}
	}
	pose := "geometry_msgs/msg/PoseStamped"
	s := fleetOf(t, FleetOptions{},
		peer("localizer", "patrol-1", []TopicView{pub("amcl_pose", pose, "reliable", "localizer", 10)}),
		peer("navigator", "patrol-1", []TopicView{sub("amcl_pose", pose, "reliable", "navigator", 9)}),
		peer("localizer", "patrol-2", []TopicView{pub("amcl_pose", pose, "reliable", "localizer", 20)}),
		peer("navigator", "patrol-2", []TopicView{sub("amcl_pose", pose, "reliable", "navigator", 19)}))

	if len(s.Topics) != 2 {
		t.Fatalf("%d topics, want amcl_pose once per robot: %+v", len(s.Topics), s.Topics)
	}
	for _, topic := range s.Topics {
		if topic.Name != "amcl_pose" {
			t.Fatalf("unexpected topic %+v", topic)
		}
		if len(topic.Pubs) != 1 || len(topic.Subs) != 1 {
			t.Fatalf("%s on %s has %d pubs and %d subs, want one of each",
				topic.Name, topic.Robot, len(topic.Pubs), len(topic.Subs))
		}
		if topic.Pubs[0].Process != topic.Robot+"/localizer" {
			t.Fatalf("publisher %q is not on %s", topic.Pubs[0].Process, topic.Robot)
		}
	}
	if s.Topics[0].Sent == s.Topics[1].Sent {
		t.Fatal("the two robots' counters were merged")
	}

	// Bringup order is per robot, and the fleet summary carries both.
	if len(s.Robots) != 2 {
		t.Fatalf("robots = %+v", s.Robots)
	}
	for _, r := range s.Robots {
		if got := strings.Join(r.Order, " -> "); got != r.Name+"/localizer -> "+r.Name+"/navigator" {
			t.Errorf("%s order = %s", r.Name, got)
		}
		if r.Processes != 2 || r.Reachable != 2 {
			t.Errorf("%s = %d/%d processes", r.Name, r.Reachable, r.Processes)
		}
	}
}

// A missing publisher is reported against the robot it is missing on.
func TestFleetFindingsNameTheRobot(t *testing.T) {
	orphan := process("navigator", []TopicView{
		sub("amcl_pose", "geometry_msgs/msg/PoseStamped", "reliable", "navigator", 1),
	})
	s := fleetOf(t, FleetOptions{},
		Peer{Name: "navigator", Host: "patrol-2", Robot: "patrol-2", URL: fakePeer(t, orphan).URL})

	got := findings(s, "FLEET04")
	if len(got) != 1 || !strings.Contains(got[0].Msg, "on patrol-2") {
		t.Fatalf("FLEET04 = %v, want the robot named", got)
	}
}

// Robots may legitimately be calibrated differently — that is what a robot's
// own frames file is for — so trees are compared within a robot, not across.
func TestFleetComparesTransformTreesWithinARobot(t *testing.T) {
	tree := func(x float64) []FrameView {
		return []FrameView{{Parent: "base_link", Child: "laser", XYZ: [3]float64{x, 0, 0.19}}}
	}
	withFrames := func(node string, frames []FrameView) DashboardState {
		s := process(node, nil)
		s.Frames = frames
		return s
	}
	peer := func(node, robot string, frames []FrameView) Peer {
		return Peer{Name: node, Host: robot, Robot: robot, URL: fakePeer(t, withFrames(node, frames)).URL}
	}

	// Different robots, different calibration: expected, not a finding.
	across := fleetOf(t, FleetOptions{},
		peer("localizer", "patrol-1", tree(0.12)),
		peer("localizer", "patrol-2", tree(0.118)))
	if got := findings(across, "FLEET06"); len(got) != 0 {
		t.Fatalf("two robots with their own calibration reported %v", got)
	}

	// Two processes of one robot disagreeing means one is running an older
	// release, which is a finding.
	within := fleetOf(t, FleetOptions{},
		peer("localizer", "patrol-1", tree(0.12)),
		peer("navigator", "patrol-1", tree(0.118)))
	if got := findings(within, "FLEET06"); len(got) != 1 {
		t.Fatalf("FLEET06 = %v, want one process of patrol-1 reported", got)
	}
}

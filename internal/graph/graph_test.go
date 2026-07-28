package graph

import (
	"path/filepath"
	"testing"

	"conductor.dev/conductor/internal/scan"
)

func codes(issues []Issue) map[string]int {
	m := map[string]int{}
	for _, i := range issues {
		m[i.Code]++
	}
	return m
}

// The sample app under internal/scan/testdata contains deliberate defects;
// this exercises the full scan -> validate pipeline against them.
func TestValidateSampleApp(t *testing.T) {
	app, err := scan.ScanApp(filepath.Join("..", "scan", "testdata", "sampleapp"))
	if err != nil {
		t.Fatal(err)
	}
	_, issues := Validate(app)
	got := codes(issues)

	// battery: best-effort sensor pub vs reliable sub.
	if got["CND013"] == 0 {
		t.Error("expected CND013 qos incompatibility on battery")
	}
	// lidar: subscribed, never published.
	if got["CND010"] == 0 {
		t.Error("expected CND010 missing publisher for lidar")
	}
	// Monitor has no OnEstop.
	if got["CND003"] == 0 {
		t.Error("expected CND003 missing handler for OnEstop")
	}
	if !Errors(issues) {
		t.Error("Errors should report true")
	}
}

func TestValidateCleanGraph(t *testing.T) {
	app := &scan.App{
		Name:     "clean",
		Messages: map[string]string{"msgs.Twist": "geometry_msgs/msg/Twist"},
		Nodes: []*scan.Node{
			{
				Name: "planner", StructName: "Planner",
				Pubs:    []scan.Endpoint{{Field: "Cmd", Topic: "cmd_vel", QoS: "reliable", GoType: "msgs.Twist"}},
				Methods: map[string]scan.MethodSig{},
			},
			{
				Name: "driver", StructName: "Driver",
				Subs:    []scan.Endpoint{{Field: "Cmd", Topic: "cmd_vel", QoS: "reliable", GoType: "msgs.Twist"}},
				Methods: map[string]scan.MethodSig{"OnCmd": {Params: 1}},
			},
		},
	}
	g, issues := Validate(app)
	if len(issues) != 0 {
		t.Fatalf("expected clean graph, got %v", issues)
	}
	if len(g.Topics) != 1 || g.Topics[0].RosType() != "geometry_msgs/msg/Twist" {
		t.Fatalf("topics = %+v", g.Topics)
	}
}

func TestValidateTypeMismatch(t *testing.T) {
	app := &scan.App{
		Name:     "t",
		Messages: map[string]string{},
		Nodes: []*scan.Node{
			{
				Name: "a", StructName: "A",
				Pubs:    []scan.Endpoint{{Field: "Out", Topic: "x", GoType: "msgs.Twist"}},
				Methods: map[string]scan.MethodSig{},
			},
			{
				Name: "b", StructName: "B",
				Subs:    []scan.Endpoint{{Field: "In", Topic: "x", GoType: "msgs.Pose"}},
				Methods: map[string]scan.MethodSig{"OnIn": {Params: 1}},
			},
		},
	}
	_, issues := Validate(app)
	if codes(issues)["CND012"] == 0 {
		t.Errorf("expected CND012 type mismatch, got %v", issues)
	}
}

func TestValidateExternalSatisfiesSubscription(t *testing.T) {
	app := &scan.App{
		Name:      "t",
		Messages:  map[string]string{"msgs.Bool": "std_msgs/msg/Bool"},
		Externals: []scan.External{{Topic: "estop", Type: "std_msgs/msg/Bool", Role: "publisher", QoS: "transient"}},
		Nodes: []*scan.Node{
			{
				Name: "safety", StructName: "Safety",
				Subs:    []scan.Endpoint{{Field: "Estop", Topic: "estop", QoS: "transient", GoType: "msgs.Bool"}},
				Methods: map[string]scan.MethodSig{"OnEstop": {Params: 1}},
			},
		},
	}
	_, issues := Validate(app)
	if codes(issues)["CND010"] != 0 {
		t.Errorf("external publisher should satisfy the subscription, got %v", issues)
	}
}

package graph

import (
	"testing"

	"conductor.dev/conductor"
	"conductor.dev/conductor/internal/scan"
)

func TestValidateFrameSampleApp(t *testing.T) {
	_, issues := Validate(missionApp(t))
	got := codes(issues)
	for code, why := range map[string]string{
		"CND050": `frame:"camera" is not in the declared tree`,
		"CND054": `Lookup("map", "laser") crosses a dynamic transform`,
		"CND057": "a frame tag on a message with no Header",
	} {
		if got[code] == 0 {
			t.Errorf("expected %s: %s", code, why)
		}
	}
}

func frameApp(nodes []*scan.Node, tree *conductor.FrameTree) *scan.App {
	return &scan.App{
		Name: "frames", Messages: map[string]string{"main.Cloud": "sample_msgs/msg/Cloud"},
		Stamped: map[string]bool{"main.Cloud": true},
		Frames:  tree, FramesFile: "frames.json", Nodes: nodes,
	}
}

func tree(transforms ...conductor.Transform) *conductor.FrameTree {
	return &conductor.FrameTree{Path: "frames.json", Transforms: transforms}
}

// The structural faults in a transform tree map to codes of their own, so the
// message says which shape of mistake it is.
func TestValidateFrameTreeShape(t *testing.T) {
	cases := []struct {
		name string
		tree *conductor.FrameTree
		want string
	}{
		{"two parents", tree(
			conductor.Transform{Parent: "a", Child: "c"},
			conductor.Transform{Parent: "b", Child: "c"},
		), "CND051"},
		{"cycle", tree(
			conductor.Transform{Parent: "a", Child: "b"},
			conductor.Transform{Parent: "b", Child: "a"},
		), "CND052"},
		{"two roots", tree(
			conductor.Transform{Parent: "a", Child: "b"},
			conductor.Transform{Parent: "c", Child: "d"},
		), "CND053"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			_, issues := Validate(frameApp(nil, tc.tree))
			if codes(issues)[tc.want] == 0 {
				t.Fatalf("issues %v, want %s", issues, tc.want)
			}
		})
	}
}

// Declaring transforms nobody publishes is worth saying out loud: tf_static
// only goes out if some node declares a conductor.TF field.
func TestValidateWarnsWhenNothingPublishesTF(t *testing.T) {
	app := frameApp([]*scan.Node{{Name: "a", Methods: map[string]scan.MethodSig{}}},
		tree(conductor.Transform{Parent: "base_link", Child: "laser"}))
	_, issues := Validate(app)
	if codes(issues)["CND055"] == 0 {
		t.Fatalf("issues %v, want CND055", issues)
	}

	app.Nodes[0].TF = &scan.TFDecl{Field: "TF"}
	if _, issues := Validate(app); codes(issues)["CND055"] != 0 {
		t.Fatalf("issues %v, want no CND055 once a node declares TF", issues)
	}
}

// Two endpoints of one topic that declare different frames is a warning when
// a transform connects them, and an error when nothing does.
func TestValidateFramePairs(t *testing.T) {
	nodes := func() []*scan.Node {
		return []*scan.Node{
			{
				Name: "pub", Methods: map[string]scan.MethodSig{},
				Pubs: []scan.Endpoint{{Field: "Cloud", Topic: "cloud", QoS: "reliable", GoType: "main.Cloud", Frame: "laser"}},
			},
			{
				Name: "sub", Methods: map[string]scan.MethodSig{"OnCloud": {Params: 1}},
				Subs: []scan.Endpoint{{Field: "Cloud", Topic: "cloud", QoS: "reliable", GoType: "main.Cloud", Frame: "base_link"}},
			},
		}
	}
	connected := frameApp(nodes(), tree(conductor.Transform{Parent: "base_link", Child: "laser"}))
	_, issues := Validate(connected)
	warned := false
	for _, i := range issues {
		if i.Code == "CND056" && i.Severity == Warning {
			warned = true
		}
	}
	if !warned {
		t.Fatalf("issues %v, want a CND056 warning", issues)
	}

	split := frameApp(nodes(), tree(
		conductor.Transform{Parent: "base_link", Child: "imu"},
		conductor.Transform{Parent: "chassis", Child: "laser"},
	))
	_, issues = Validate(split)
	errored := false
	for _, i := range issues {
		if i.Code == "CND056" && i.Severity == Error {
			errored = true
		}
	}
	if !errored {
		t.Fatalf("issues %v, want a CND056 error when no transform connects the frames", issues)
	}
}

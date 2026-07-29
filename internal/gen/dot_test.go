package gen

import (
	"strings"
	"testing"

	"conductor.dev/conductor"
	"conductor.dev/conductor/internal/graph"
	"conductor.dev/conductor/internal/scan"
)

func missionGraph() *graph.Graph {
	app := &scan.App{
		Name: "courier", Messages: map[string]string{}, Stamped: map[string]bool{},
		FramesFile: "frames.json",
		Frames: &conductor.FrameTree{Transforms: []conductor.Transform{
			{Parent: "base_link", Child: "laser", XYZ: [3]float64{0.12, 0, 0.19}},
			{Parent: "map", Child: "odom", By: "amcl", Dynamic: true},
		}},
		Nodes: []*scan.Node{{
			Name: "courier", StructName: "Courier",
			Missions: []scan.Mission{{Field: "Trip", Name: "trip", Start: "pickup"}},
			Steps: []scan.Step{
				{Field: "Pickup", Name: "pickup", Next: "transit"},
				{Field: "Transit", Name: "transit", Next: "done", Fail: "recharge", Timeout: "2m"},
				{Field: "Recharge", Name: "recharge", Next: "transit"},
			},
			Calls: []scan.Call{{Method: "Goto", In: "OnTransit", Args: []string{"recharge"}}},
			Methods: map[string]scan.MethodSig{
				"OnPickup": {Params: 1, Results: 1}, "OnTransit": {Params: 1, Results: 1},
				"OnRecharge": {Params: 1, Results: 1},
			},
		}},
	}
	return graph.Build(app)
}

// The state diagram carries every kind of transition, so what is drawn is
// what the runtime will do.
func TestMissionDot(t *testing.T) {
	dot := MissionDot(missionGraph())
	for _, want := range []string{
		`"trip.start" -> "trip.pickup"`,
		`"trip.pickup" -> "trip.transit"`,
		`"trip.transit" -> "trip.recharge" [style=dashed, label="fail"`,
		`"trip.transit" -> "trip.recharge" [style=dotted, label="goto"`,
		`"trip.done" [label="done", shape=doublecircle`,
		`timeout 2m`,
	} {
		if !strings.Contains(dot, want) {
			t.Errorf("mission.dot is missing %q:\n%s", want, dot)
		}
	}
}

// The transform tree is drawn with the static links we publish separated from
// the dynamic ones we only expect.
func TestFramesDot(t *testing.T) {
	dot := FramesDot(missionGraph())
	if !strings.Contains(dot, `"base_link" -> "laser" [label="0.12 0 0.19"`) {
		t.Errorf("frames.dot is missing the static link:\n%s", dot)
	}
	if !strings.Contains(dot, `"map" -> "odom" [style=dashed, label="amcl"`) {
		t.Errorf("frames.dot is missing the dynamic link:\n%s", dot)
	}
}

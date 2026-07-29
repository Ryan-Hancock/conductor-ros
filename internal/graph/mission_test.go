package graph

import (
	"path/filepath"
	"strings"
	"testing"

	"conductor.dev/conductor"
	"conductor.dev/conductor/internal/scan"
)

// missionApp is the sample under internal/scan/testdata with deliberate
// mission and frame defects; this exercises the whole scan -> validate path,
// including the transitions written in code rather than in tags.
func missionApp(t *testing.T) *scan.App {
	t.Helper()
	app, err := scan.ScanApp(filepath.Join("..", "scan", "testdata", "missionapp"))
	if err != nil {
		t.Fatal(err)
	}
	return app
}

func TestValidateMissionSampleApp(t *testing.T) {
	_, issues := Validate(missionApp(t))
	got := codes(issues)

	for code, why := range map[string]string{
		"CND040": `next:"dropof" and Goto("nowhere") name steps that do not exist`,
		"CND042": "dropoff and idle are unreachable from pickup",
		"CND043": `retry:"soon" is not a count`,
	} {
		if got[code] == 0 {
			t.Errorf("expected %s: %s", code, why)
		}
	}
	if got["CND040"] != 2 {
		t.Errorf("CND040 fired %d times, want 2 (the tag and the Goto)", got["CND040"])
	}
	if !Errors(issues) {
		t.Error("Errors should report true")
	}
}

// The machine the checker resolves is the one the runtime runs and mission.dot
// draws, Gotos included.
func TestMachinesResolveTransitions(t *testing.T) {
	machines := Machines(missionApp(t))
	if len(machines) != 1 {
		t.Fatalf("%d machines, want 1", len(machines))
	}
	m := machines[0]
	if m.Node != "courier" || m.Name != "trip" || m.Start != "pickup" {
		t.Fatalf("machine %+v is not courier's trip starting at pickup", m)
	}
	byName := map[string]MachineStep{}
	for _, s := range m.Steps {
		byName[s.Name] = s
	}
	transit := byName["transit"]
	if got := strings.Join(transit.Targets(), ","); got != "dropof,recharge,nowhere" {
		t.Fatalf("transit targets %s, want the tags and the Goto", got)
	}
	if byName["pickup"].Next != "transit" || !byName["pickup"].Reachable {
		t.Fatalf("pickup %+v", byName["pickup"])
	}
	if byName["idle"].Reachable {
		t.Fatal("idle is reachable, but nothing transitions to it")
	}
	// A step with no next tag ends the mission.
	if got := byName["dropoff"].Next; got != conductor.StepDone {
		t.Fatalf("dropoff next %q, want %q", got, conductor.StepDone)
	}
}

// The structural mission mistakes that are not about one step.
func TestValidateMissionShape(t *testing.T) {
	step := func(field, next string) scan.Step {
		return scan.Step{Field: field, Name: scan.SnakeCase(field), Next: next}
	}
	cases := []struct {
		name string
		node *scan.Node
		want string
	}{
		{
			"start names no step",
			&scan.Node{
				Name: "a", Missions: []scan.Mission{{Field: "Run", Name: "run", Start: "elsewhere"}},
				Steps: []scan.Step{step("Only", "done")}, Methods: map[string]scan.MethodSig{"OnOnly": {Params: 1, Results: 1}},
			},
			"CND041",
		},
		{
			"steps without a mission",
			&scan.Node{
				Name: "b", Steps: []scan.Step{step("Only", "done")},
				Methods: map[string]scan.MethodSig{"OnOnly": {Params: 1, Results: 1}},
			},
			"CND041",
		},
		{
			"two missions",
			&scan.Node{
				Name: "c",
				Missions: []scan.Mission{
					{Field: "Run", Name: "run", Start: "only"},
					{Field: "Other", Name: "other", Start: "only"},
				},
				Steps:   []scan.Step{step("Only", "done")},
				Methods: map[string]scan.MethodSig{"OnOnly": {Params: 1, Results: 1}},
			},
			"CND041",
		},
		{
			"step handler missing",
			&scan.Node{
				Name: "d", Missions: []scan.Mission{{Field: "Run", Name: "run", Start: "only"}},
				Steps: []scan.Step{step("Only", "done")}, Methods: map[string]scan.MethodSig{},
			},
			"CND003",
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			app := &scan.App{Name: "t", Messages: map[string]string{}, Stamped: map[string]bool{}, Nodes: []*scan.Node{tc.node}}
			_, issues := Validate(app)
			if codes(issues)[tc.want] == 0 {
				t.Fatalf("issues %v, want %s", issues, tc.want)
			}
		})
	}
}

// A mission that is entirely well-formed passes, tags and Gotos alike.
func TestValidateCleanMission(t *testing.T) {
	app := &scan.App{
		Name: "clean", Messages: map[string]string{}, Stamped: map[string]bool{},
		Nodes: []*scan.Node{{
			Name: "courier", StructName: "Courier",
			Missions: []scan.Mission{{Field: "Trip", Name: "trip", Start: "pickup"}},
			Steps: []scan.Step{
				{Field: "Pickup", Name: "pickup", Next: "transit"},
				{Field: "Transit", Name: "transit", Next: "done", Fail: "recharge", Timeout: "2m", Retry: "2"},
				{Field: "Recharge", Name: "recharge", Next: "transit"},
			},
			Calls: []scan.Call{{Method: "Goto", In: "OnTransit", Args: []string{"recharge"}}},
			Methods: map[string]scan.MethodSig{
				"OnPickup": {Params: 1, Results: 1}, "OnTransit": {Params: 1, Results: 1},
				"OnRecharge": {Params: 1, Results: 1},
			},
		}},
	}
	if _, issues := Validate(app); len(issues) != 0 {
		t.Fatalf("clean mission reported %v", issues)
	}
}

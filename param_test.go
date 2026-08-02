package conductor

import (
	"os"
	"path/filepath"
	"testing"
	"time"

	"conductor.dev/conductor/internal/msggen"
)

func TestParamWireHashes(t *testing.T) {
	share := "/opt/ros/lyrical/share"
	if _, err := os.Stat(share); err != nil {
		t.Skip("no ROS distro installed")
	}
	r := msggen.NewResolver([]string{share})
	for name, want := range map[string]string{
		"rcl_interfaces/msg/ParameterEvent":     parameterEventHash,
		"rcl_interfaces/srv/GetParameters":      getParametersHash,
		"rcl_interfaces/srv/SetParameters":      setParametersHash,
		"rcl_interfaces/srv/ListParameters":     listParametersHash,
		"rcl_interfaces/srv/DescribeParameters": describeParametersHash,
		"rcl_interfaces/srv/GetParameterTypes":  getParameterTypesHash,
	} {
		td, err := r.Describe(name)
		if err != nil {
			t.Fatalf("%s: %v", name, err)
		}
		if got := td.Hash(); got != want {
			t.Errorf("%s:\n got %s\nwant hardcoded %s", name, got, want)
		}
	}
}

func TestParseParamFile(t *testing.T) {
	src := `# a comment
navigator:
  ros__parameters:
    max_speed: 1.5
    frame_id: "base_link"    # trailing comment
    verbose: true
/**:
  ros__parameters:
    use_sim_time: true
`
	pf, err := parseParamFile("test.yaml", []byte(src))
	if err != nil {
		t.Fatal(err)
	}
	if got := pf["navigator"]["max_speed"]; got != "1.5" {
		t.Errorf("max_speed = %q", got)
	}
	if got := pf["navigator"]["frame_id"]; got != `"base_link"` {
		t.Errorf("frame_id = %q", got)
	}
	if got := pf[paramWildcard]["use_sim_time"]; got != "true" {
		t.Errorf("wildcard use_sim_time = %q", got)
	}

	// Wildcard entries apply to a node, but the node's own value wins.
	merged := pf.forNode("navigator")
	if merged["use_sim_time"] != "true" || merged["max_speed"] != "1.5" {
		t.Errorf("merged = %+v", merged)
	}
}

func TestParseParamFileErrors(t *testing.T) {
	cases := map[string]string{
		"tabs":           "node:\n\tros__parameters:\n\t  x: 1\n",
		"no colon":       "node:\n  ros__parameters:\n    oops\n",
		"param w/o node": "  ros__parameters:\n    x: 1\n",
		"value on node":  "node: 5\n",
		"missing block":  "node:\n  x: 1\n",
		"nested group":   "node:\n  ros__parameters:\n    group:\n",
	}
	for name, src := range cases {
		if _, err := parseParamFile("t.yaml", []byte(src)); err == nil {
			t.Errorf("%s: expected an error", name)
		}
	}
}

type Tunable struct {
	Speed   Param[float64]       `name:"max_speed" default:"1.5"`
	Frame   Param[string]        `name:"frame_id" default:"base_link"`
	Enabled Param[bool]          `default:"false"`
	Period  Param[time.Duration] `default:"250ms"`
}

func TestParamFileOverridesDefaults(t *testing.T) {
	dir := t.TempDir()
	base := filepath.Join(dir, "params.yaml")
	os.WriteFile(base, []byte("tunable:\n  ros__parameters:\n    max_speed: 2.5\n    frame_id: \"odom\"\n"), 0o644)
	overlay := filepath.Join(dir, "params.sim.yaml")
	os.WriteFile(overlay, []byte("tunable:\n  ros__parameters:\n    max_speed: 9.9\n    enabled: true\n"), 0o644)

	files, err := resolveParamFiles([]string{base}, "sim")
	if err != nil {
		t.Fatal(err)
	}
	if len(files) != 2 {
		t.Fatalf("resolved files = %v, want base + overlay", files)
	}
	values, err := LoadParamFiles(files...)
	if err != nil {
		t.Fatal(err)
	}

	n := &Tunable{}
	a, err := newAppWithParams("inproc", TransportOptions{}, "", values, nil, n)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(a.stop)

	if got := n.Speed.Get(); got != 9.9 {
		t.Errorf("max_speed = %v, want 9.9 (overlay wins over base and default)", got)
	}
	if got := n.Frame.Get(); got != "odom" {
		t.Errorf("frame_id = %q, want odom (base wins over default)", got)
	}
	if got := n.Enabled.Get(); got != true {
		t.Errorf("enabled = %v, want true (overlay only)", got)
	}
	if got := n.Period.Get(); got != 250*time.Millisecond {
		t.Errorf("period = %v, want the 250ms default (no file entry)", got)
	}
}

func TestResolveParamFilesMissingEnv(t *testing.T) {
	dir := t.TempDir()
	base := filepath.Join(dir, "params.yaml")
	os.WriteFile(base, []byte("n:\n  ros__parameters:\n    x: 1\n"), 0o644)
	if _, err := resolveParamFiles([]string{base}, "nosuch"); err == nil {
		t.Fatal("expected an error for a missing environment file")
	}
	if _, err := resolveParamFiles([]string{filepath.Join(dir, "gone.yaml")}, ""); err == nil {
		t.Fatal("expected an error for a missing params file")
	}
}

// The parameter services are the ROS-facing surface; drive them the way
// `ros2 param` does.
func TestParameterServices(t *testing.T) {
	n := &Tunable{}
	a := newTestApp(t, n)
	tr := a.rt.transport

	list, err := tr.ServiceClient(ServiceSpec{Service: "tunable/list_parameters", Node: "test"})
	if err != nil {
		t.Fatal(err)
	}
	get, err := tr.ServiceClient(ServiceSpec{Service: "tunable/get_parameters", Node: "test"})
	if err != nil {
		t.Fatal(err)
	}
	set, err := tr.ServiceClient(ServiceSpec{Service: "tunable/set_parameters", Node: "test"})
	if err != nil {
		t.Fatal(err)
	}
	describe, err := tr.ServiceClient(ServiceSpec{Service: "tunable/describe_parameters", Node: "test"})
	if err != nil {
		t.Fatal(err)
	}

	res, err := list(listParametersRequest{}, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	names := res.(listParametersResponse).Result.Names
	// Four declared, plus the use_sim_time every ROS node carries.
	if len(names) != 5 {
		t.Fatalf("list_parameters returned %v, want the four declared plus use_sim_time", names)
	}
	var hasSimTime bool
	for _, n := range names {
		if n == "use_sim_time" {
			hasSimTime = true
		}
	}
	if !hasSimTime {
		t.Errorf("list_parameters = %v, missing use_sim_time", names)
	}

	got, err := get(getParametersRequest{Names: []string{"max_speed", "frame_id", "nope"}}, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	values := got.(getParametersResponse).Values
	if values[0].Type != paramTypeDouble || values[0].DoubleValue != 1.5 {
		t.Errorf("max_speed value = %+v", values[0])
	}
	if values[1].Type != paramTypeString || values[1].StringValue != "base_link" {
		t.Errorf("frame_id value = %+v", values[1])
	}
	if values[2].Type != paramTypeNotSet {
		t.Errorf("unknown parameter should report NOT_SET, got %+v", values[2])
	}

	// Setting a value takes effect immediately for Get.
	out, err := set(setParametersRequest{Parameters: []parameterMsg{
		{Name: "max_speed", Value: parameterValueMsg{Type: paramTypeDouble, DoubleValue: 3.25}},
	}}, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if r := out.(setParametersResponse).Results[0]; !r.Successful {
		t.Fatalf("set failed: %s", r.Reason)
	}
	if n.Speed.Get() != 3.25 {
		t.Errorf("after set, Get() = %v, want 3.25", n.Speed.Get())
	}

	// A type mismatch is refused rather than silently coerced.
	out, err = set(setParametersRequest{Parameters: []parameterMsg{
		{Name: "max_speed", Value: parameterValueMsg{Type: paramTypeString, StringValue: "fast"}},
	}}, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if r := out.(setParametersResponse).Results[0]; r.Successful {
		t.Error("setting a double parameter from a string should fail")
	}

	// Unknown parameters are reported, not fatal.
	out, err = set(setParametersRequest{Parameters: []parameterMsg{
		{Name: "nope", Value: parameterValueMsg{Type: paramTypeBool, BoolValue: true}},
	}}, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if r := out.(setParametersResponse).Results[0]; r.Successful {
		t.Error("setting an undeclared parameter should fail")
	}

	desc, err := describe(describeParametersRequest{Names: []string{"frame_id"}}, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if d := desc.(describeParametersResponse).Descriptors[0]; d.Type != paramTypeString || d.Name != "frame_id" {
		t.Errorf("descriptor = %+v", d)
	}
}

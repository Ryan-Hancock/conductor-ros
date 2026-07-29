package conductor

import (
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func writeFrames(t *testing.T, body string) string {
	t.Helper()
	p := filepath.Join(t.TempDir(), "frames.json")
	if err := os.WriteFile(p, []byte(body), 0o644); err != nil {
		t.Fatal(err)
	}
	return p
}

func loadTree(t *testing.T, body string) *FrameTree {
	t.Helper()
	tree, err := LoadFrames(writeFrames(t, body))
	if err != nil {
		t.Fatal(err)
	}
	if tree == nil {
		t.Fatal("no tree loaded")
	}
	return tree
}

const robotFrames = `{
  "static": [
    {"parent": "base_link", "child": "laser", "xyz": [0.12, 0, 0.19], "rpy": [0, 0, 1.5707963267948966]},
    {"parent": "base_link", "child": "imu",   "xyz": [0, 0, 0.05]}
  ],
  "dynamic": [
    {"parent": "map",  "child": "odom",      "by": "amcl"},
    {"parent": "odom", "child": "base_link", "by": "ekf"}
  ]
}`

func TestFramesLoadAndDescribe(t *testing.T) {
	tree := loadTree(t, robotFrames)
	if got, want := len(tree.Static()), 2; got != want {
		t.Fatalf("%d static transforms, want %d", got, want)
	}
	if got, want := strings.Join(tree.Frames(), ","), "base_link,imu,laser,map,odom"; got != want {
		t.Fatalf("frames %s, want %s", got, want)
	}
	if roots := tree.Roots(); len(roots) != 1 || roots[0] != "map" {
		t.Fatalf("roots %v, want [map]", roots)
	}
	if problems := tree.Check(); len(problems) != 0 {
		t.Fatalf("well-formed tree reported %v", problems)
	}
}

// A missing frames.json is not an error: an application may declare no frames.
func TestFramesMissingFileIsNotAnError(t *testing.T) {
	tree, err := LoadFrames(filepath.Join(t.TempDir(), "frames.json"))
	if err != nil || tree != nil {
		t.Fatalf("LoadFrames = %v, %v; want nil, nil", tree, err)
	}
}

func TestFrameTreeCheckFindsStructuralFaults(t *testing.T) {
	cases := []struct {
		name, body, kind string
	}{
		{
			"two parents",
			`{"static":[{"parent":"a","child":"c"},{"parent":"b","child":"c"}]}`,
			"multiple_parents",
		},
		{
			"cycle",
			`{"static":[{"parent":"a","child":"b"},{"parent":"b","child":"a"}]}`,
			"cycle",
		},
		{
			"two roots",
			`{"static":[{"parent":"a","child":"b"},{"parent":"c","child":"d"}]}`,
			"disconnected",
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got := loadTree(t, tc.body).Check()
			for _, p := range got {
				if p.Kind == tc.kind {
					return
				}
			}
			t.Fatalf("problems %v, want one of kind %s", got, tc.kind)
		})
	}
}

// Composition: the lidar is 0.12 m ahead and 0.19 m up, rotated a quarter
// turn, so a point 1 m in front of the lidar is 0.12 m ahead and 1 m to the
// left of base_link.
func TestFrameLookupComposesStaticTransforms(t *testing.T) {
	tree := loadTree(t, robotFrames)
	at, err := tree.Lookup("base_link", "laser")
	if err != nil {
		t.Fatal(err)
	}
	p := at.Apply([3]float64{1, 0, 0})
	want := [3]float64{0.12, 1, 0.19}
	for i := range want {
		if math.Abs(p[i]-want[i]) > 1e-9 {
			t.Fatalf("point %v, want %v", p, want)
		}
	}
	// And the other way round, which is the inverse.
	back, err := tree.Lookup("laser", "base_link")
	if err != nil {
		t.Fatal(err)
	}
	q := back.Apply(p)
	for i, w := range [3]float64{1, 0, 0} {
		if math.Abs(q[i]-w) > 1e-9 {
			t.Fatalf("round trip gave %v, want [1 0 0]", q)
		}
	}
}

// A lookup that has to cross a transform someone else publishes cannot be
// answered from declarations, and says so rather than guessing.
func TestFrameLookupRefusesToCrossDynamicTransforms(t *testing.T) {
	tree := loadTree(t, robotFrames)
	_, err := tree.Lookup("map", "laser")
	if err == nil {
		t.Fatal("lookup across map -> odom succeeded, want an error")
	}
	for _, want := range []string{"dynamic", "odom", "amcl"} {
		if !strings.Contains(err.Error(), want) {
			t.Fatalf("error %q does not mention %q", err, want)
		}
	}
	// But the frames are still connected, which is what makes a frame
	// mismatch between two endpoints a warning rather than a broken graph.
	if err := tree.Connects("map", "laser"); err != nil {
		t.Fatalf("Connects: %v", err)
	}
}

func TestFrameLookupUnknownFrame(t *testing.T) {
	tree := loadTree(t, robotFrames)
	if _, err := tree.Lookup("base_link", "camera"); err == nil || !strings.Contains(err.Error(), `unknown frame "camera"`) {
		t.Fatalf("error %v, want it to name the unknown frame", err)
	}
}

func TestIsometryRPYRoundTrip(t *testing.T) {
	for _, rpy := range [][3]float64{{0, 0, 0}, {0.1, -0.2, 0.3}, {0, 0, math.Pi / 2}} {
		iso := Transform{RPY: rpy}.Isometry()
		got := iso.RPY()
		for i := range rpy {
			if math.Abs(got[i]-rpy[i]) > 1e-9 {
				t.Fatalf("rpy %v round-tripped to %v", rpy, got)
			}
		}
	}
}

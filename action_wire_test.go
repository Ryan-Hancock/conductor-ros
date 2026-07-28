package conductor

import (
	"os"
	"testing"

	"conductor.dev/conductor/internal/msggen"
)

// Verifies the hardcoded action wire hashes against the installed ROS
// distro's interface definitions (skipped when none is present). If this
// fails, the fixed types in action_wire.go changed upstream.
func TestActionWireHashes(t *testing.T) {
	share := "/opt/ros/lyrical/share"
	if _, err := os.Stat(share); err != nil {
		t.Skip("no ROS distro installed")
	}
	r := msggen.NewResolver([]string{share})
	for name, want := range map[string]string{
		"action_msgs/msg/GoalStatusArray": goalStatusArrayHash,
		"action_msgs/srv/CancelGoal":      cancelGoalHash,
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

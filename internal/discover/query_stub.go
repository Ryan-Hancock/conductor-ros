//go:build !zenoh

package discover

import (
	"errors"
	"time"
)

// Query without the zenoh transport compiled in cannot reach a ROS graph, so
// it says so plainly rather than reporting an empty one — an empty graph would
// read as "nothing is running", which is a different and much more misleading
// answer.
func Query(endpoint string, domain int, timeout time.Duration) (*Graph, error) {
	return nil, errors.New("reading a live ROS graph needs the zenoh transport: " +
		"rebuild this command with -tags zenoh (see .tools/env.sh), or run `make externals`")
}

package conductor

import "fmt"

type Reliability int

const (
	Reliable Reliability = iota
	BestEffort
)

func (r Reliability) String() string {
	if r == BestEffort {
		return "best-effort"
	}
	return "reliable"
}

type Durability int

const (
	Volatile Durability = iota
	TransientLocal
)

func (d Durability) String() string {
	if d == TransientLocal {
		return "transient-local"
	}
	return "volatile"
}

// QoS is a named quality-of-service profile. Conductor exposes a small set of
// intent-level profiles rather than raw DDS knobs; the static checker verifies
// every pub/sub pairing is compatible before deploy.
type QoS struct {
	Name        string
	Reliability Reliability
	Durability  Durability
	Depth       int
}

var qosProfiles = map[string]QoS{
	"reliable":  {Name: "reliable", Reliability: Reliable, Durability: Volatile, Depth: 10},
	"sensor":    {Name: "sensor", Reliability: BestEffort, Durability: Volatile, Depth: 5},
	"transient": {Name: "transient", Reliability: Reliable, Durability: TransientLocal, Depth: 1},
}

// QoSProfile returns the profile for a qos struct-tag value. The empty string
// selects "reliable".
func QoSProfile(name string) (QoS, bool) {
	if name == "" {
		name = "reliable"
	}
	q, ok := qosProfiles[name]
	return q, ok
}

func qosFromTag(tag string) (QoS, error) {
	q, ok := QoSProfile(tag)
	if !ok {
		return QoS{}, fmt.Errorf("unknown qos profile %q (valid: reliable, sensor, transient)", tag)
	}
	return q, nil
}

package rmwzenoh

import (
	"fmt"
	"strconv"
	"strings"

	conductor "conductor.dev/conductor"
)

// Reading the graph is the other direction of this package: rmw_zenoh
// advertises every node, publisher, subscription, service and client as a
// liveliness token, and those tokens carry everything conductor.json's
// externals block needs — the topic, the type, its RIHS01 hash, and the QoS
// profile the peer offered. So a running system can be asked what it is
// rather than transcribed from someone else's source.
//
// The formats are the ones the builders above produce; the parsers here are
// their inverse, and the round trip is tested both ways.

// Entity is one endpoint on the graph, decoded from a liveliness token.
type Entity struct {
	Domain    int
	ZID       string // the zenoh session's id: one per process
	NID, EID  int
	Kind      string // EntityNode, EntityPublisher, EntitySubscription, EntityService, EntityClient
	Enclave   string
	Namespace string
	Node      string

	// Endpoint fields, empty for a node token.
	Topic   string // fully qualified, with its leading slash
	DDSType string // "std_msgs::msg::dds_::String_"
	Type    string // "std_msgs/msg/String"
	Hash    string // "RIHS01_..."
	QoS     conductor.QoS
}

// Process identifies the process an entity belongs to: one zenoh session per
// process, one node id within it.
func (e Entity) Process() string { return e.ZID }

func (e Entity) String() string {
	if e.Kind == EntityNode {
		return fmt.Sprintf("node %s", e.Node)
	}
	return fmt.Sprintf("%s %s %s [%s]", kindWord(e.Kind), e.Topic, e.Type, e.QoS.Name)
}

func kindWord(kind string) string {
	switch kind {
	case EntityPublisher:
		return "publisher"
	case EntitySubscription:
		return "subscription"
	case EntityService:
		return "service"
	case EntityClient:
		return "client"
	case EntityNode:
		return "node"
	}
	return kind
}

// Demangle undoes Mangle, turning rmw_zenoh's single-segment encoding of a
// ROS name back into the name.
func Demangle(s string) string {
	return strings.ReplaceAll(s, "%", "/")
}

// TypeFromDDS converts a DDS type name back to its ROS interface name:
// "std_msgs::msg::dds_::String_" -> "std_msgs/msg/String". A name that does
// not follow the convention is returned with "::" replaced by "/", which is
// the best available guess and still readable.
func TypeFromDDS(dds string) string {
	parts := strings.Split(dds, "::")
	if len(parts) == 4 && parts[2] == "dds_" && strings.HasSuffix(parts[3], "_") {
		return parts[0] + "/" + parts[1] + "/" + strings.TrimSuffix(parts[3], "_")
	}
	return strings.ReplaceAll(dds, "::", "/")
}

// ParseQoSKeyexpr decodes the QoS section of an endpoint token, naming the
// conductor profile it matches. A profile conductor does not have keeps its
// fields with an empty Name, so a caller can report the mismatch rather than
// pretend it is one of ours.
func ParseQoSKeyexpr(s string) (conductor.QoS, error) {
	sections := strings.Split(s, ":")
	if len(sections) < 3 {
		return conductor.QoS{}, fmt.Errorf("qos %q: want at least 3 ':'-separated sections", s)
	}
	q := conductor.QoS{Reliability: conductor.Reliable, Durability: conductor.Volatile}

	// Section 0: reliability. Empty means the RMW default (reliable).
	switch sections[0] {
	case "":
	case "1": // RMW_QOS_POLICY_RELIABILITY_RELIABLE
		q.Reliability = conductor.Reliable
	case "2": // RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
		q.Reliability = conductor.BestEffort
	default:
		return conductor.QoS{}, fmt.Errorf("qos %q: unknown reliability %q", s, sections[0])
	}

	// Section 1: durability. Empty means the RMW default (volatile).
	switch sections[1] {
	case "":
	case "1": // RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
		q.Durability = conductor.TransientLocal
	case "2": // RMW_QOS_POLICY_DURABILITY_VOLATILE
		q.Durability = conductor.Volatile
	default:
		return conductor.QoS{}, fmt.Errorf("qos %q: unknown durability %q", s, sections[1])
	}

	// Section 2: history kind and depth, comma-separated ("kind,depth"); the
	// kind is omitted when it is the default (keep last).
	history := strings.Split(sections[2], ",")
	if len(history) > 1 && history[1] != "" {
		depth, err := strconv.Atoi(history[1])
		if err != nil {
			return conductor.QoS{}, fmt.Errorf("qos %q: history depth %q: %w", s, history[1], err)
		}
		q.Depth = depth
	}

	q.Name = profileName(q)
	return q, nil
}

// profileName is the conductor profile a discovered QoS corresponds to, or ""
// when the peer offers something conductor cannot name.
func profileName(q conductor.QoS) string {
	for _, name := range []string{"reliable", "sensor", "transient"} {
		p, _ := conductor.QoSProfile(name)
		if p.Reliability == q.Reliability && p.Durability == q.Durability {
			return name
		}
	}
	return ""
}

// ParseToken decodes a liveliness token keyexpr into an Entity. It accepts
// both node tokens (9 segments) and endpoint tokens (13).
func ParseToken(keyexpr string) (Entity, error) {
	seg := strings.Split(keyexpr, "/")
	if len(seg) < 9 || seg[0] != AdminSpace {
		return Entity{}, fmt.Errorf("not a %s liveliness token: %q", AdminSpace, keyexpr)
	}
	domain, err := strconv.Atoi(seg[1])
	if err != nil {
		return Entity{}, fmt.Errorf("token %q: domain %q: %w", keyexpr, seg[1], err)
	}
	nid, err := strconv.Atoi(seg[3])
	if err != nil {
		return Entity{}, fmt.Errorf("token %q: node id %q: %w", keyexpr, seg[3], err)
	}
	eid, err := strconv.Atoi(seg[4])
	if err != nil {
		return Entity{}, fmt.Errorf("token %q: entity id %q: %w", keyexpr, seg[4], err)
	}
	e := Entity{
		Domain:    domain,
		ZID:       seg[2],
		NID:       nid,
		EID:       eid,
		Kind:      seg[5],
		Enclave:   Demangle(seg[6]),
		Namespace: Demangle(seg[7]),
		Node:      seg[8],
	}
	switch e.Kind {
	case EntityNode:
		if len(seg) != 9 {
			return Entity{}, fmt.Errorf("token %q: a node token has 9 segments, got %d", keyexpr, len(seg))
		}
		return e, nil
	case EntityPublisher, EntitySubscription, EntityService, EntityClient:
	default:
		return Entity{}, fmt.Errorf("token %q: unknown entity kind %q", keyexpr, e.Kind)
	}
	if len(seg) != 13 {
		return Entity{}, fmt.Errorf("token %q: an endpoint token has 13 segments, got %d", keyexpr, len(seg))
	}
	e.Topic = Demangle(seg[9])
	e.DDSType = Demangle(seg[10])
	e.Type = TypeFromDDS(e.DDSType)
	e.Hash = Demangle(seg[11])
	if e.QoS, err = ParseQoSKeyexpr(seg[12]); err != nil {
		return Entity{}, fmt.Errorf("token %q: %w", keyexpr, err)
	}
	return e, nil
}

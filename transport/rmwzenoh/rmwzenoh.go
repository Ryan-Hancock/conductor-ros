// Package rmwzenoh implements rmw_zenoh_cpp's wire conventions — topic key
// expressions, liveliness (graph) tokens, and the per-message attachment —
// so a conductor process is indistinguishable from a ROS 2 node on a Zenoh
// graph. Formats were verified against live rmw_zenoh 0.10 traffic (ROS 2
// Lyrical); the golden values in rmwzenoh_test.go are captured samples.
//
// This package is pure Go (no zenoh dependency) so the mapping is testable
// without a Zenoh session; the cgo transport in ../zenoh consumes it.
package rmwzenoh

import (
	"encoding/binary"
	"fmt"
	"strings"

	conductor "conductor.dev/conductor"
)

// AdminSpace is the liveliness token prefix rmw_zenoh uses for graph
// discovery.
const AdminSpace = "@ros2_lv"

// Entity kind strings as they appear in liveliness tokens.
const (
	EntityNode         = "NN"
	EntityPublisher    = "MP"
	EntitySubscription = "MS"
	EntityService      = "SS"
	EntityClient       = "SC"
)

// StripName removes the leading/trailing slashes of a ROS name for use in a
// key expression segment.
func StripName(s string) string {
	return strings.Trim(s, "/")
}

// Mangle replaces '/' with '%' — rmw_zenoh's encoding for embedding ROS
// names in a single keyexpr segment.
func Mangle(s string) string {
	return strings.ReplaceAll(s, "/", "%")
}

// TopicKeyexpr is the key expression data flows on:
// "<domain>/<topic>/<dds type>/<type hash>".
func TopicKeyexpr(domain int, topic, ddsType, hash string) string {
	return fmt.Sprintf("%d/%s/%s/%s", domain, StripName(topic), ddsType, hash)
}

// QoSKeyexpr encodes a profile the way rmw_zenoh's qos_to_keyexpr does:
// six ':'-separated sections (reliability, durability, history, deadline,
// lifespan, liveliness), each field printed only when it differs from the
// RMW default, with ',' separating fields within a section. Depth is always
// printed for conductor's profiles.
func QoSKeyexpr(q conductor.QoS) string {
	rel := ""
	if q.Reliability == conductor.BestEffort {
		rel = "2" // RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
	}
	dur := ""
	if q.Durability == conductor.TransientLocal {
		dur = "1" // RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
	}
	return fmt.Sprintf("%s:%s:,%d:,:,:,,", rel, dur, q.Depth)
}

// NodeToken is the liveliness token advertising a node:
// "@ros2_lv/<domain>/<zid>/<nid>/<eid>/NN/<enclave>/<namespace>/<name>",
// with enclave and namespace "/" mangled to "%" (root in both cases here).
func NodeToken(domain int, zid string, nid, eid int, node string) string {
	return fmt.Sprintf("%s/%d/%s/%d/%d/%s/%%/%%/%s",
		AdminSpace, domain, zid, nid, eid, EntityNode, node)
}

// EndpointToken is the liveliness token advertising a publisher (kind
// EntityPublisher) or subscription (EntitySubscription) on a topic.
func EndpointToken(domain int, zid string, nid, eid int, kind, node, topic, ddsType, hash string, q conductor.QoS) string {
	return fmt.Sprintf("%s/%d/%s/%d/%d/%s/%%/%%/%s/%s/%s/%s/%s",
		AdminSpace, domain, zid, nid, eid, kind, node,
		Mangle("/"+StripName(topic)), Mangle(ddsType), Mangle(hash), QoSKeyexpr(q))
}

// EncodeAttachment builds the per-message attachment rmw_zenoh expects:
// sequence number (int64 LE), source timestamp in nanoseconds (int64 LE),
// then the publisher GID as a zenoh-serialized byte array (compact length
// prefix + bytes; the 16-byte GID's length always fits one byte).
func EncodeAttachment(seq, timestampNS int64, gid []byte) []byte {
	buf := make([]byte, 0, 8+8+1+len(gid))
	buf = binary.LittleEndian.AppendUint64(buf, uint64(seq))
	buf = binary.LittleEndian.AppendUint64(buf, uint64(timestampNS))
	buf = append(buf, byte(len(gid)))
	return append(buf, gid...)
}

// DecodeAttachment parses an attachment produced by EncodeAttachment or by
// rmw_zenoh. A service server echoes the decoded sequence number and GID
// back in its reply so the client can correlate it.
func DecodeAttachment(b []byte) (seq, timestampNS int64, gid []byte, err error) {
	if len(b) < 17 {
		return 0, 0, nil, fmt.Errorf("attachment too short: %d bytes", len(b))
	}
	seq = int64(binary.LittleEndian.Uint64(b))
	timestampNS = int64(binary.LittleEndian.Uint64(b[8:]))
	n := int(b[16])
	if len(b) < 17+n {
		return 0, 0, nil, fmt.Errorf("attachment gid truncated: want %d bytes, have %d", n, len(b)-17)
	}
	return seq, timestampNS, b[17 : 17+n], nil
}

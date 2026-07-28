package conductor

import (
	"reflect"
	"strings"
	"sync"
)

// MessageInfo identifies a message type in ROS terms. Name is the ROS
// interface name ("geometry_msgs/msg/Twist"); Hash is its REP-2011 type hash
// ("RIHS01_..."), which rmw_zenoh embeds in topic keyexprs — publishers and
// subscribers only match when it agrees with the peer's.
type MessageInfo struct {
	Name string
	Hash string
}

// DDSType returns the DDS type name used on the wire
// ("geometry_msgs::msg::dds_::Twist_").
func (m MessageInfo) DDSType() string {
	parts := strings.Split(m.Name, "/")
	if len(parts) != 3 {
		return strings.ReplaceAll(m.Name, "/", "::")
	}
	return parts[0] + "::" + parts[1] + "::dds_::" + parts[2] + "_"
}

var (
	msgMu       sync.RWMutex
	msgRegistry = map[reflect.Type]MessageInfo{}
)

// RegisterMessage associates Go type T with its ROS interface name and type
// hash. The msgs package registers the common types it ships; applications
// register their own. Networked transports refuse to wire unregistered types;
// the in-process transport does not need registration.
func RegisterMessage[T any](name, hash string) {
	msgMu.Lock()
	defer msgMu.Unlock()
	msgRegistry[reflect.TypeFor[T]()] = MessageInfo{Name: name, Hash: hash}
}

// MessageInfoOf looks up the registered info for a message type.
func MessageInfoOf(t reflect.Type) (MessageInfo, bool) {
	msgMu.RLock()
	defer msgMu.RUnlock()
	info, ok := msgRegistry[t]
	return info, ok
}

type serviceKey struct {
	req, res reflect.Type
}

var svcRegistry = map[serviceKey]MessageInfo{}

// RegisterService associates the (Req, Res) Go type pair with a ROS service
// name ("std_srvs/srv/SetBool") and its RIHS01 hash. conductor msggen emits
// these calls from .srv definitions.
func RegisterService[Req, Res any](name, hash string) {
	msgMu.Lock()
	defer msgMu.Unlock()
	svcRegistry[serviceKey{reflect.TypeFor[Req](), reflect.TypeFor[Res]()}] = MessageInfo{Name: name, Hash: hash}
}

// ServiceInfoOf looks up the registered service info for a request/response
// type pair.
func ServiceInfoOf(req, res reflect.Type) (MessageInfo, bool) {
	msgMu.RLock()
	defer msgMu.RUnlock()
	info, ok := svcRegistry[serviceKey{req, res}]
	return info, ok
}

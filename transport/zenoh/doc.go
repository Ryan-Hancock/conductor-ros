// Package zenoh provides conductor's rmw_zenoh transport: publishers and
// subscriptions join a live ROS 2 graph through a Zenoh router, speaking
// rmw_zenoh's key expressions, CDR payloads, per-message attachments, and
// liveliness tokens (so conductor nodes appear in ros2 node/topic tooling).
//
// The implementation links zenoh-c via cgo and is gated behind the "zenoh"
// build tag:
//
//	go build -tags zenoh ./...
//
// with CGO_CFLAGS/CGO_LDFLAGS pointing at zenoh-c >= 1.9 built with the
// unstable API (see .tools/env.sh). Importing this package without the tag
// compiles to nothing: the transport stays unregistered and selecting
// -transport zenoh fails at startup with a hint.
package zenoh

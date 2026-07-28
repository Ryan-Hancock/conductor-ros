// Package conductor is an application framework for robotics that treats
// ROS 2 as a runtime rather than an application framework: nodes are plain Go
// structs marked with a //conductor:node directive, and their pub/sub,
// parameter, and timer surface is declared as typed struct fields.
//
// The conductor CLI (cmd/conductor) derives the application's full
// communication graph from these declarations at build time — validating
// wiring, message types, and QoS compatibility before anything runs — and
// generates launch and parameter artifacts from it.
//
// This package is the runtime that wires and executes declared nodes. v0.1
// runs all nodes in-process over an internal bus with single-threaded
// per-node executors (mirroring ROS executor semantics); transports that join
// a live ROS 2 graph over DDS or Zenoh are the subject of DESIGN.md.
package conductor

package main

import (
	"time"
)

// A MoveGroup goal populated everywhere it can be: nested structs, arrays of
// structs, arrays of arrays, fixed-size arrays, signed blobs, strings, times
// and durations. This is the deepest message in common ROS use, which is the
// point of using it as a codec test.
func fullGoal() MoveGroupGoal {
	stamp := time.Unix(1785000000, 123456789).UTC()
	header := Header{Stamp: stamp, FrameId: "world"}

	mesh := Mesh{
		Triangles: []MeshTriangle{{VertexIndices: [3]uint32{0, 1, 2}}, {VertexIndices: [3]uint32{2, 3, 0}}},
		Vertices:  []Point{{X: 0, Y: 0, Z: 0}, {X: 1, Y: 0, Z: 0}, {X: 1, Y: 1, Z: 0}, {X: 0, Y: 1, Z: 0}},
	}
	box := SolidPrimitive{Type: SolidPrimitive_BOX, Dimensions: []float64{0.1, 0.2, 0.3}}
	volume := BoundingVolume{
		Primitives:     []SolidPrimitive{box, {Type: SolidPrimitive_SPHERE, Dimensions: []float64{0.05}}},
		PrimitivePoses: []Pose{{Position: Point{X: 0.4}, Orientation: Quaternion{W: 1}}, {}},
		Meshes:         []Mesh{mesh},
		MeshPoses:      []Pose{{Orientation: Quaternion{W: 1}}},
	}
	object := CollisionObject{
		Header: header, Id: "cube_7", Type: ObjectType{Key: "cube", Db: "{}"},
		Pose:           Pose{Position: Point{X: 0.5, Y: -0.2, Z: 0.1}, Orientation: Quaternion{W: 1}},
		Primitives:     []SolidPrimitive{box},
		PrimitivePoses: []Pose{{Orientation: Quaternion{W: 1}}},
		Meshes:         []Mesh{mesh},
		MeshPoses:      []Pose{{Orientation: Quaternion{W: 1}}},
		Planes:         []Plane{{Coef: [4]float64{0, 0, 1, -0.02}}},
		PlanePoses:     []Pose{{Orientation: Quaternion{W: 1}}},
		SubframeNames:  []string{"grasp", "top"},
		SubframePoses:  []Pose{{Position: Point{Z: 0.05}}, {Position: Point{Z: 0.1}}},
		Operation:      CollisionObject_ADD,
	}

	return MoveGroupGoal{
		Request: MotionPlanRequest{
			WorkspaceParameters: WorkspaceParameters{
				Header:    header,
				MinCorner: Vector3{X: -1, Y: -1, Z: 0},
				MaxCorner: Vector3{X: 1, Y: 1, Z: 1.5},
			},
			StartState: RobotState{
				JointState: JointState{
					Header:   header,
					Name:     []string{"shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"},
					Position: []float64{0, -1.57, 1.57, 0, 1.57, 0},
					Velocity: []float64{0, 0, 0, 0, 0, 0},
					Effort:   []float64{},
				},
				MultiDofJointState: MultiDOFJointState{
					Header:     header,
					JointNames: []string{"virtual_joint"},
					Transforms: []Transform{{Translation: Vector3{Z: 0.1}, Rotation: Quaternion{W: 1}}},
					Twist:      []Twist{{}},
					Wrench:     []Wrench{{}},
				},
				AttachedCollisionObjects: []AttachedCollisionObject{{
					LinkName: "gripper", Object: object,
					TouchLinks: []string{"finger_left", "finger_right"},
					DetachPosture: JointTrajectory{
						Header: header, JointNames: []string{"finger"},
						Points: []JointTrajectoryPoint{{
							Positions: []float64{0.04}, Velocities: []float64{0},
							TimeFromStart: 500 * time.Millisecond,
						}},
					},
					Weight: 0.5,
				}},
				IsDiff: true,
			},
			GoalConstraints: []Constraints{{
				Name: "pick",
				JointConstraints: []JointConstraint{
					{JointName: "elbow", Position: 1.2, ToleranceAbove: 0.01, ToleranceBelow: 0.01, Weight: 1},
				},
				PositionConstraints: []PositionConstraint{{
					Header: header, LinkName: "tool0",
					TargetPointOffset: Vector3{Z: 0.02},
					ConstraintRegion:  volume,
					Weight:            1,
				}},
				OrientationConstraints: []OrientationConstraint{{
					Header: header, LinkName: "tool0",
					Orientation:            Quaternion{W: 1},
					AbsoluteXAxisTolerance: 0.1,
					AbsoluteYAxisTolerance: 0.1,
					AbsoluteZAxisTolerance: 3.14,
					Weight:                 1,
				}},
				VisibilityConstraints: []VisibilityConstraint{{
					TargetRadius: 0.05, TargetPose: PoseStamped{Header: header},
					ConeSides: 4, SensorPose: PoseStamped{Header: header},
					MaxViewAngle: 0.5, Weight: 1,
				}},
			}},
			PathConstraints: Constraints{Name: "keep_upright"},
			TrajectoryConstraints: TrajectoryConstraints{
				Constraints: []Constraints{{Name: "waypoint_1"}},
			},
			ReferenceTrajectories: []GenericTrajectory{{
				Header: header,
				JointTrajectory: []JointTrajectory{{
					Header: header, JointNames: []string{"elbow"},
					Points: []JointTrajectoryPoint{{Positions: []float64{1}, TimeFromStart: time.Second}},
				}},
			}},
			PipelineId: "ompl", PlannerId: "RRTConnect", GroupName: "manipulator",
			NumPlanningAttempts: 3, AllowedPlanningTime: 5,
			MaxVelocityScalingFactor: 0.5, MaxAccelerationScalingFactor: 0.25,
			CartesianSpeedLimitedLink: "tool0", MaxCartesianSpeed: 0.1,
		},
		PlanningOptions: PlanningOptions{
			PlanningSceneDiff: PlanningScene{
				Name: "scene", RobotModelName: "ur5",
				AllowedCollisionMatrix: AllowedCollisionMatrix{
					EntryNames: []string{"table", "cube_7"},
					EntryValues: []AllowedCollisionEntry{
						{Enabled: []bool{false, true}},
						{Enabled: []bool{true, false}},
					},
					DefaultEntryNames:  []string{"table"},
					DefaultEntryValues: []bool{true},
				},
				World: PlanningSceneWorld{
					CollisionObjects: []CollisionObject{object},
					Octomap: OctomapWithPose{
						Header: header,
						Origin: Pose{Orientation: Quaternion{W: 1}},
						Octomap: Octomap{
							Header: header, Binary: true, Id: "OcTree", Resolution: 0.05,
							Data: []int8{-1, 0, 1, 127, -128},
						},
					},
				},
				ObjectColors: []ObjectColor{{Id: "cube_7", Color: ColorRGBA{R: 1, A: 1}}},
				LinkPadding:  []LinkPadding{{LinkName: "tool0", Padding: 0.01}},
				LinkScale:    []LinkScale{{LinkName: "tool0", Scale: 1.01}},
				IsDiff:       true,
			},
			PlanOnly: true, LookAround: false, Replan: true,
			ReplanAttempts: 2, ReplanDelay: 0.5,
		},
	}
}

package msgs

import conductor "conductor.dev/conductor"

// Type hashes are REP-2011 RIHS01 values harvested from the ROS 2 Lyrical
// type description files (share/<pkg>/msg/*.json). They are middleware- and
// platform-independent, but must match the peer's hash for rmw_zenoh
// keyexprs to align — regenerate against the targeted ROS distro when these
// interfaces change.
func init() {
	conductor.RegisterMessage[Bool]("std_msgs/msg/Bool",
		"RIHS01_feb91e995ff9ebd09c0cb3d2aed18b11077585839fb5db80193b62d74528f6c9")
	conductor.RegisterMessage[String]("std_msgs/msg/String",
		"RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18")
	conductor.RegisterMessage[Header]("std_msgs/msg/Header",
		"RIHS01_f49fb3ae2cf070f793645ff749683ac6b06203e41c891e17701b1cb597ce6a01")
	conductor.RegisterMessage[Vector3]("geometry_msgs/msg/Vector3",
		"RIHS01_cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d")
	conductor.RegisterMessage[Point]("geometry_msgs/msg/Point",
		"RIHS01_6963084842a9b04494d6b2941d11444708d892da2f4b09843b9c43f42a7f6881")
	conductor.RegisterMessage[Quaternion]("geometry_msgs/msg/Quaternion",
		"RIHS01_8a765f66778c8ff7c8ab94afcc590a2ed5325a1d9a076ffff38fbce36f458684")
	conductor.RegisterMessage[Pose]("geometry_msgs/msg/Pose",
		"RIHS01_d501954e9476cea2996984e812054b68026ae0bfae789d9a10b23daf35cc90fa")
	conductor.RegisterMessage[PoseStamped]("geometry_msgs/msg/PoseStamped",
		"RIHS01_10f3786d7d40fd2b54367835614bff85d4ad3b5dab62bf8bca0cc232d73b4cd8")
	conductor.RegisterMessage[Twist]("geometry_msgs/msg/Twist",
		"RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a")
}

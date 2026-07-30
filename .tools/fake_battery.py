#!/usr/bin/env python3
"""A battery for a robot that has none.

Neither the TurtleBot3 simulation nor examples/nav2stub has a battery, and the
Nav2 example's docking branch needs one to be worth watching. So the battery is
what it always was in reality: somebody else's driver, declared as a `requires`
process of the environment rather than pretended at inside the application.

It drains while the robot is moving, charges while it is parked *at the dock*,
and does neither while it is parked anywhere else — a robot dwelling at a
waypoint is not charging, which is the difference that makes the example's
low-battery branch reachable.

    python3 .tools/fake_battery.py [--drain 0.03] [--charge 0.12] [--dock 0.8]
"""

import argparse
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState


class FakeBattery(Node):
    def __init__(self, drain: float, charge: float, dock_radius: float):
        super().__init__("battery_driver")
        self.drain, self.charge_rate, self.dock_radius = drain, charge, dock_radius
        self.percentage = 1.0
        self.moving = False
        self.at_dock = True

        # sensor_msgs on a best-effort profile, as a driver would publish it.
        sensor = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(BatteryState, "battery_state", sensor)
        # Whether the robot is moving is readable from the controller's command,
        # and where it is from the localizer's pose. Both are what any other
        # node on the graph would use.
        self.create_subscription(TwistStamped, "cmd_vel", self.on_cmd, 10)
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.on_pose, 10)
        self.create_timer(1.0, self.on_tick)

    def on_cmd(self, msg: TwistStamped) -> None:
        v = msg.twist.linear
        self.moving = math.hypot(v.x, v.y) > 0.01 or abs(msg.twist.angular.z) > 0.01

    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        self.at_dock = math.hypot(p.x, p.y) < self.dock_radius

    def on_tick(self) -> None:
        if self.moving:
            self.percentage = max(0.0, self.percentage - self.drain)
            status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        elif self.at_dock:
            self.percentage = min(1.0, self.percentage + self.charge_rate)
            status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.voltage = 11.1 + float(self.percentage)
        msg.percentage = float(self.percentage)
        msg.present = True
        msg.power_supply_status = status
        self.pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--drain", type=float, default=0.03, help="per second while moving")
    parser.add_argument("--charge", type=float, default=0.12, help="per second at the dock")
    parser.add_argument("--dock", type=float, default=0.8, help="how close to the origin is docked")
    args = parser.parse_args()

    rclpy.init()
    node = FakeBattery(args.drain, args.charge, args.dock)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

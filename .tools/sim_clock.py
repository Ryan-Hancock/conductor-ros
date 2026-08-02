#!/usr/bin/env python3
"""A simulator's clock, without the simulator.

Gazebo publishes /clock and every node is expected to follow it. This publishes
the same topic so simulated time can be exercised — and, more usefully,
exercised at a *rate*: --scale 0.25 runs the robot's world at a quarter speed,
which is what shakes out code that assumed the wall clock.

    python3 .tools/sim_clock.py [--scale 1.0] [--rate 50] [--start 1000]
"""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class SimClock(Node):
    def __init__(self, scale: float, rate: float, start: float):
        super().__init__("sim_clock")
        self.scale = scale
        self.simulated = start
        self.period = 1.0 / rate
        # A simulator's clock is a fast, lossy, always-current topic.
        self.pub = self.create_publisher(
            Clock, "/clock", QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.timer = self.create_timer(self.period, self.tick)

    def tick(self) -> None:
        self.simulated += self.period * self.scale
        msg = Clock()
        msg.clock.sec = int(self.simulated)
        msg.clock.nanosec = int((self.simulated % 1) * 1e9)
        self.pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scale", type=float, default=1.0, help="simulated seconds per real second")
    parser.add_argument("--rate", type=float, default=50.0, help="publish rate in Hz")
    parser.add_argument("--start", type=float, default=1000.0, help="simulated time to start at")
    args = parser.parse_args()

    rclpy.init()
    node = SimClock(args.scale, args.rate, args.start)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

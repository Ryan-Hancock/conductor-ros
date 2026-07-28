"""Reference rclpy Fibonacci action server, used by .tools/interop.sh to
check conductor's action client against a real ROS 2 implementation.

Note the ReentrantCallbackGroup + MultiThreadedExecutor: without them rclpy
cannot service cancel requests while a goal is executing, which looks
exactly like a client-side bug.
"""

import time

import rclpy
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def execute(goal_handle):
    seq = [0, 1]
    # Fibonacci values are int32 on the wire; beyond order 46 python raises
    # OverflowError while Go silently wraps.
    for _ in range(min(goal_handle.request.order, 40) - 1):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = Fibonacci.Result()
            result.sequence = seq
            return result
        time.sleep(0.4)
        seq.append(seq[-1] + seq[-2])
        feedback = Fibonacci.Feedback()
        feedback.sequence = seq
        goal_handle.publish_feedback(feedback)
    goal_handle.succeed()
    result = Fibonacci.Result()
    result.sequence = seq
    return result


def main():
    rclpy.init()
    node = Node('py_fib_server')
    ActionServer(
        node, Fibonacci, 'fibonacci', execute,
        cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
        callback_group=ReentrantCallbackGroup(),
    )
    print('python fibonacci action server up', flush=True)
    rclpy.spin(node, executor=MultiThreadedExecutor())


if __name__ == '__main__':
    main()

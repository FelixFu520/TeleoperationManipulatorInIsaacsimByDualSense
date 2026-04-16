#!/usr/bin/env python3
"""最小化测试: 用 Python rclpy 发布 /joint_command 看 IsaacSim 是否响应。

运行方式:
    source /opt/ros/humble/setup.bash
    python test_ros2_pub.py
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main():
    rclpy.init()
    node = rclpy.create_node("test_joint_pub")
    pub = node.create_publisher(JointState, "/joint_command", 10)

    msg = JointState()
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 0
    msg.header.frame_id = ""
    msg.name = [
        "panda_finger_joint1",
        "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7",
    ]
    msg.position = [0.0, 0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.785]
    msg.velocity = []
    msg.effort = []

    print(f"Topic: /joint_command")
    print(f"Names: {msg.name}")
    print(f"Positions: {msg.position}")
    print("开始持续发布 (Ctrl+C 退出) ...")

    try:
        while rclpy.ok():
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0)
            print(f"\r已发布: {msg.position}", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    print("\n退出。")


if __name__ == "__main__":
    main()

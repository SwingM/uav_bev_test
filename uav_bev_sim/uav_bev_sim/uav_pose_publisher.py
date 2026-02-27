#!/usr/bin/env python3
"""Republish UAV pose onto a stable project topic."""

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node


class UAVPosePublisher(Node):
    """Bridge-independent pose republisher for the UAV model."""

    def __init__(self) -> None:
        super().__init__('uav_pose_publisher')

        # Use a direct model pose topic that works on Humble bridges.
        self.declare_parameter('input_pose_topic', '/model/uav_platform/pose')
        self.declare_parameter('output_topic', '/uav_platform/pose')

        self.input_pose_topic = self.get_parameter('input_pose_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.pose_pub = self.create_publisher(Pose, self.output_topic, 10)
        self.pose_sub = self.create_subscription(
            Pose,
            self.input_pose_topic,
            self.pose_callback,
            10,
        )

        self._published_once = False
        self.get_logger().info(
            f'Listening on {self.input_pose_topic}; publishing to {self.output_topic}.'
        )

    def pose_callback(self, msg: Pose) -> None:
        self.pose_pub.publish(msg)
        if not self._published_once:
            self._published_once = True
            self.get_logger().info('First UAV pose received and published.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UAVPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

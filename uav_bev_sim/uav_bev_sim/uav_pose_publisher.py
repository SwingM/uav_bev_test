#!/usr/bin/env python3
"""Publish the UAV pose from Gazebo entity poses to a plain ROS Pose topic."""

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from ros_gz_interfaces.msg import Pose_V


class UAVPosePublisher(Node):
    """Extract the UAV pose from Gazebo world pose stream and republish it."""

    def __init__(self) -> None:
        super().__init__('uav_pose_publisher')

        self.declare_parameter('world_pose_topic', '/world/mosaic_world/pose/info')
        self.declare_parameter('uav_name', 'uav_platform')
        self.declare_parameter('output_topic', '/uav_platform/pose')

        self.world_pose_topic = self.get_parameter('world_pose_topic').value
        self.uav_name = self.get_parameter('uav_name').value
        self.output_topic = self.get_parameter('output_topic').value

        self.pose_pub = self.create_publisher(Pose, self.output_topic, 10)
        self.pose_sub = self.create_subscription(
            Pose_V,
            self.world_pose_topic,
            self.pose_callback,
            10,
        )

        self._last_pose = None
        self.get_logger().info(
            f'Listening on {self.world_pose_topic} for entity "{self.uav_name}"; '
            f'publishing pose to {self.output_topic}.'
        )

    def _iter_entities(self, msg: Pose_V):
        """Handle Pose_V variants used by Gazebo bridges across versions."""
        for field in ('pose', 'poses', 'data'):
            entities = getattr(msg, field, None)
            if entities:
                return entities
        return []

    def pose_callback(self, msg: Pose_V) -> None:
        for entity_pose in self._iter_entities(msg):
            if getattr(entity_pose, 'name', '') != self.uav_name:
                continue

            pose_msg = Pose()
            pose_msg.position = entity_pose.position
            pose_msg.orientation = entity_pose.orientation
            self.pose_pub.publish(pose_msg)

            if self._last_pose is None:
                self.get_logger().info('First UAV pose received and published.')
            self._last_pose = pose_msg
            return


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

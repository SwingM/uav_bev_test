#!/usr/bin/env python3
"""Publish UAV pose from Gazebo pose stream as geometry_msgs/Pose."""

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from ros_gz_interfaces.msg import Pose_V


class UAVPosePublisher(Node):
    """Extract one UAV entity pose from Pose_V and republish it on a ROS topic."""

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

        self._published_once = False
        self._miss_count = 0
        self.get_logger().info(
            f'Listening on {self.world_pose_topic} for "{self.uav_name}", '
            f'publishing to {self.output_topic}.'
        )

    @staticmethod
    def _iter_entities(msg: Pose_V):
        for field in ('pose', 'poses', 'data'):
            entities = getattr(msg, field, None)
            if entities is not None:
                return entities
        return []

    @staticmethod
    def _entity_name(entity) -> str:
        return str(getattr(entity, 'name', ''))

    @staticmethod
    def _entity_pose(entity):
        # ros_gz_interfaces/Pose contains a nested `pose` field.
        nested = getattr(entity, 'pose', None)
        if nested is not None and hasattr(nested, 'position') and hasattr(nested, 'orientation'):
            return nested

        # Fallback for possible flattened structures.
        if hasattr(entity, 'position') and hasattr(entity, 'orientation'):
            return entity

        return None

    def pose_callback(self, msg: Pose_V) -> None:
        entities = self._iter_entities(msg)
        for entity in entities:
            if self._entity_name(entity) != self.uav_name:
                continue

            src_pose = self._entity_pose(entity)
            if src_pose is None:
                self.get_logger().warning('Matched UAV entity but pose fields were missing.')
                return

            pose_msg = Pose()
            pose_msg.position = src_pose.position
            pose_msg.orientation = src_pose.orientation
            self.pose_pub.publish(pose_msg)

            if not self._published_once:
                self.get_logger().info('First UAV pose received and published.')
                self._published_once = True
            self._miss_count = 0
            return

        # Throttled debug if entity name did not match.
        self._miss_count += 1
        if self._miss_count % 50 == 0:
            sample_names = [self._entity_name(e) for e in list(entities)[:8]]
            self.get_logger().warning(
                f'No pose matched "{self.uav_name}" in Pose_V. Sample entity names: {sample_names}'
            )


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

#!/usr/bin/env python3
"""Autopilot node that executes rectangular lawnmower coverage using /cmd_vel."""

from dataclasses import dataclass
import math
from typing import List

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node


@dataclass
class Waypoint:
    x: float
    y: float


class UAVAutopilot(Node):
    def __init__(self) -> None:
        super().__init__('uav_autopilot')

        # Topics / naming.
        self.declare_parameter('pose_topic', '/uav_platform/pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Rectangle bounds and altitude hold.
        self.declare_parameter('x_min', -8.0)
        self.declare_parameter('x_max', 8.0)
        self.declare_parameter('y_min', -8.0)
        self.declare_parameter('y_max', 8.0)
        self.declare_parameter('target_altitude_m', 5.0)

        # Camera-informed coverage parameters.
        self.declare_parameter('camera_interval_s', 0.7)
        self.declare_parameter('camera_footprint_x_m', 4.0)
        self.declare_parameter('camera_footprint_y_m', 3.0)
        self.declare_parameter('coverage_overlap_ratio', 0.25)

        # Motion parameters.
        self.declare_parameter('max_speed_mps', 1.5)
        self.declare_parameter('position_tolerance_m', 0.25)
        self.declare_parameter('control_rate_hz', 10.0)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.x_min = float(self.get_parameter('x_min').value)
        self.x_max = float(self.get_parameter('x_max').value)
        self.y_min = float(self.get_parameter('y_min').value)
        self.y_max = float(self.get_parameter('y_max').value)
        self.target_altitude = float(self.get_parameter('target_altitude_m').value)

        self.camera_interval = max(0.05, float(self.get_parameter('camera_interval_s').value))
        self.camera_footprint_x = max(0.2, float(self.get_parameter('camera_footprint_x_m').value))
        self.camera_footprint_y = max(0.2, float(self.get_parameter('camera_footprint_y_m').value))
        self.overlap = min(0.9, max(0.0, float(self.get_parameter('coverage_overlap_ratio').value)))

        self.max_speed = max(0.1, float(self.get_parameter('max_speed_mps').value))
        self.pos_tol = max(0.05, float(self.get_parameter('position_tolerance_m').value))
        self.control_rate = max(1.0, float(self.get_parameter('control_rate_hz').value))

        self.pose_sub = self.create_subscription(Pose, self.pose_topic, self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / self.control_rate, self.on_timer)

        self.current_pose: Pose | None = None
        self.coverage_path = self.generate_lawnmower_path()
        self.waypoint_index = 0
        self.finished = False

        self.path_speed = self._compute_path_speed()

        self.get_logger().info(
            f'Autopilot ready. Bounds=({self.x_min}, {self.y_min}) -> ({self.x_max}, {self.y_max}), '
            f'waypoints={len(self.coverage_path)}, planned speed={self.path_speed:.2f} m/s.'
        )

    def _compute_path_speed(self) -> float:
        # Ensure longitudinal sampling along the sweep direction doesn't skip areas.
        along_track_step = self.camera_footprint_x * (1.0 - self.overlap)
        requested_speed = along_track_step / self.camera_interval
        speed = min(self.max_speed, max(0.05, requested_speed))
        self.get_logger().info(
            f'Camera interval={self.camera_interval:.2f}s, along-track step={along_track_step:.2f}m -> '
            f'speed={speed:.2f} m/s (capped by max_speed={self.max_speed:.2f}).'
        )
        return speed

    def generate_lawnmower_path(self) -> List[Waypoint]:
        x0, x1 = sorted([self.x_min, self.x_max])
        y0, y1 = sorted([self.y_min, self.y_max])

        # Cross-track line spacing from camera footprint with overlap margin.
        line_spacing = self.camera_footprint_y * (1.0 - self.overlap)
        line_spacing = max(0.3, line_spacing)

        path: List[Waypoint] = []
        direction = 1
        y = y0

        while y <= y1 + 1e-6:
            if direction > 0:
                path.append(Waypoint(x=x0, y=y))
                path.append(Waypoint(x=x1, y=y))
            else:
                path.append(Waypoint(x=x1, y=y))
                path.append(Waypoint(x=x0, y=y))
            y += line_spacing
            direction *= -1

        if not path:
            path = [Waypoint(x=x0, y=y0)]

        return path

    def pose_callback(self, msg: Pose) -> None:
        self.current_pose = msg

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def on_timer(self) -> None:
        if self.current_pose is None:
            return

        if self.finished:
            self._publish_stop()
            return

        if self.waypoint_index >= len(self.coverage_path):
            self.get_logger().info('Coverage complete. Holding position.')
            self.finished = True
            self._publish_stop()
            return

        target = self.coverage_path[self.waypoint_index]

        dx = target.x - self.current_pose.position.x
        dy = target.y - self.current_pose.position.y
        dz = self.target_altitude - self.current_pose.position.z
        distance_xy = math.hypot(dx, dy)

        if distance_xy <= self.pos_tol:
            self.waypoint_index += 1
            self.get_logger().info(
                f'Reached waypoint {self.waypoint_index}/{len(self.coverage_path)} '
                f'at ({target.x:.2f}, {target.y:.2f}).'
            )
            return

        cmd = Twist()
        cmd.linear.x = self.path_speed * (dx / max(distance_xy, 1e-6))
        cmd.linear.y = self.path_speed * (dy / max(distance_xy, 1e-6))
        cmd.linear.z = max(-0.5, min(0.5, 0.6 * dz))
        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UAVAutopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

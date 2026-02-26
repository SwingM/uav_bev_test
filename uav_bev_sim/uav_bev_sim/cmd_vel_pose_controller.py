#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node


class CmdVelPoseController(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_pose_controller')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('pose_topic', '/uav_platform/pose_cmd')
        self.declare_parameter('fixed_altitude_m', 5.0)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('pose_feedback_topic', '/uav_platform/pose')

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.fixed_altitude = self.get_parameter('fixed_altitude_m').value
        self.rate_hz = self.get_parameter('rate_hz').value
        self.pose_feedback_topic = self.get_parameter('pose_feedback_topic').value

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = 0.0
        self.cmd_vel = Twist()

        self.subscription = self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)
        self.publisher = self.create_publisher(Pose, self.pose_topic, 10)
        self.feedback_publisher = self.create_publisher(Pose, self.pose_feedback_topic, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(
            f'Bridging {self.cmd_vel_topic} to {self.pose_topic} at fixed altitude {self.fixed_altitude:.2f} m.'
        )

    def on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_vel = msg

    def on_timer(self) -> None:
        dt = 1.0 / self.rate_hz

        # Integrate horizontal velocity in world frame; enforce constant altitude.
        self.x += float(self.cmd_vel.linear.x) * dt
        self.y += float(self.cmd_vel.linear.y) * dt
        self.yaw += float(self.cmd_vel.angular.z) * dt

        msg = Pose()
        msg.position.x = self.x
        msg.position.y = self.y
        msg.position.z = float(self.fixed_altitude)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(self.yaw / 2.0)
        msg.orientation.w = math.cos(self.yaw / 2.0)

        self.publisher.publish(msg)
        self.feedback_publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = CmdVelPoseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

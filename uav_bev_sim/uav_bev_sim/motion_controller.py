#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node


class MotionController(Node):
    def __init__(self) -> None:
        super().__init__('motion_controller')

        self.declare_parameter('topic', '/uav_platform/pose_cmd')
        self.declare_parameter('altitude_m', 5.0)
        self.declare_parameter('speed_mps', 1.0)
        self.declare_parameter('x_min', -10.0)
        self.declare_parameter('x_max', 10.0)
        self.declare_parameter('y_min', -10.0)
        self.declare_parameter('y_max', 10.0)
        self.declare_parameter('lane_spacing', 2.0)
        self.declare_parameter('rate_hz', 20.0)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.altitude = self.get_parameter('altitude_m').value
        self.speed = self.get_parameter('speed_mps').value
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        self.lane_spacing = self.get_parameter('lane_spacing').value
        self.rate_hz = self.get_parameter('rate_hz').value

        self.publisher = self.create_publisher(Pose, self.topic, 10)

        self.current_x = self.x_min
        self.current_y = self.y_min
        self.direction = 1.0
        self.finished = False

        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)
        self.get_logger().info('Motion controller started with lawn-mower path.')

    def on_timer(self) -> None:
        if self.finished:
            return

        dt = 1.0 / self.rate_hz
        next_x = self.current_x + self.direction * self.speed * dt

        reached_edge = (self.direction > 0 and next_x >= self.x_max) or (
            self.direction < 0 and next_x <= self.x_min
        )

        if reached_edge:
            self.current_x = self.x_max if self.direction > 0 else self.x_min
            self.current_y += self.lane_spacing
            self.direction *= -1.0
            if self.current_y > self.y_max:
                self.finished = True
                self.get_logger().info('Completed lawn-mower coverage.')
        else:
            self.current_x = next_x

        msg = Pose()
        msg.position.x = float(self.current_x)
        msg.position.y = float(self.current_y)
        msg.position.z = float(self.altitude)

        yaw = math.pi if self.direction < 0 else 0.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(yaw / 2.0)
        msg.orientation.w = math.cos(yaw / 2.0)

        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = MotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

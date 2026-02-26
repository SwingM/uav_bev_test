#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MotionController(Node):
    def __init__(self) -> None:
        super().__init__('motion_controller')

        self.declare_parameter('topic', '/cmd_vel')
        self.declare_parameter('forward_speed_mps', 1.0)
        self.declare_parameter('sideways_speed_mps', 0.0)
        self.declare_parameter('rate_hz', 20.0)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.forward_speed = self.get_parameter('forward_speed_mps').value
        self.sideways_speed = self.get_parameter('sideways_speed_mps').value
        self.rate_hz = self.get_parameter('rate_hz').value

        self.publisher = self.create_publisher(Twist, self.topic, 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)
        self.get_logger().info('Motion controller started with constant horizontal velocity.')

    def on_timer(self) -> None:
        msg = Twist()
        msg.linear.x = float(self.forward_speed)
        msg.linear.y = float(self.sideways_speed)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

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

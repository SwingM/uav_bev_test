#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MotionController(Node):
    def __init__(self) -> None:
        super().__init__('motion_controller')

        self.declare_parameter('topic', '/cmd_vel')
        self.declare_parameter('forward_speed_mps', 1.0)
        self.declare_parameter('sideways_speed_mps', 0.3)
        self.declare_parameter('segment_duration_s', 4.0)
        self.declare_parameter('rate_hz', 20.0)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.forward_speed = float(self.get_parameter('forward_speed_mps').value)
        self.sideways_speed = float(self.get_parameter('sideways_speed_mps').value)
        self.segment_duration_s = float(self.get_parameter('segment_duration_s').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.publisher = self.create_publisher(Twist, self.topic, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.phase = 0
        self.phase_elapsed_s = 0.0

        self.get_logger().info('Motion controller started (horizontal scan on /cmd_vel).')

    def on_timer(self) -> None:
        dt = 1.0 / self.rate_hz
        self.phase_elapsed_s += dt

        if self.phase_elapsed_s >= self.segment_duration_s:
            self.phase = (self.phase + 1) % 4
            self.phase_elapsed_s = 0.0

        msg = Twist()

        # 4-phase horizontal scan (no vertical movement):
        # 0: forward, 1: sideways step, 2: backward, 3: sideways step.
        if self.phase == 0:
            msg.linear.x = self.forward_speed
            msg.linear.y = 0.0
        elif self.phase == 1:
            msg.linear.x = 0.0
            msg.linear.y = self.sideways_speed
        elif self.phase == 2:
            msg.linear.x = -self.forward_speed
            msg.linear.y = 0.0
        else:
            msg.linear.x = 0.0
            msg.linear.y = self.sideways_speed

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

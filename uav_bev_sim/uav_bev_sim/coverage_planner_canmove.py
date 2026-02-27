import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # map bounds
        self.xmin = -7.0
        self.xmax = 12.0
        self.ymin = -8.0
        self.ymax = 8.0

        self.line_spacing = 2.0
        self.speed = 0.03
        self.turn_speed = 0.5

        # generate waypoints in lawn-mower pattern
        self.waypoints = self.generate_lawnmower()
        self.current_wp = 0
        self.reached_start = False

        # timer
        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info(f"Generated {len(self.waypoints)} waypoints")

        # assume UAV starts at center
        self.position = [-5.5, 7.5]  # only for open-loop demo
        self.yaw = 0.0

    def generate_lawnmower(self):
        waypoints = []
        y = self.ymin
        direction = 1  # sweep +x first
        while y <= self.ymax:
            if direction == 1:
                waypoints.append((self.xmin, y))
                waypoints.append((self.xmax, y))
            else:
                waypoints.append((self.xmax, y))
                waypoints.append((self.xmin, y))
            y += self.line_spacing
            direction *= -1
        return waypoints

    def update(self):
        cmd = Twist()

        if not self.reached_start:
            # fly to first waypoint
            target_x, target_y = self.waypoints[0]
            # simple open-loop approximation
            dx = target_x - self.position[0]
            dy = target_y - self.position[1]

            if abs(dx) < 0.1 and abs(dy) < 0.1:
                self.reached_start = True
                self.get_logger().info("Reached start corner, starting coverage")
            else:
                cmd.linear.x = self.speed if dx > 0 else -self.speed
                cmd.linear.y = self.speed if dy > 0 else -self.speed
                self.pub.publish(cmd)
                return

        # coverage phase: fly forward constantly (simplified)
        cmd.linear.x = self.speed
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
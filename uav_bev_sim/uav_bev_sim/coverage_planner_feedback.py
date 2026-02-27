#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from ros_gz_msgs.msg import Entity_V
import math

class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')

        # Publisher to command UAV velocity
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to Gazebo dynamic pose topic
        self.pose_sub = self.create_subscription(
            Entity_V,
            '/world/mosaic_world/dynamic_pose/info',
            self.pose_cb,
            10
        )

        # UAV current pose
        self.uav_pose = None

        # Coverage box [xmin, ymin, xmax, ymax] (adjust to landmarks)
        self.box = [-5.0, -6.0, 8.0, 6.0]

        # Coverage parameters
        self.speed = 1.0        # linear speed in m/s
        self.step_size = 2.0    # distance between coverage lines
        self.state = 'goto_start'

        # Coverage line targets
        self.next_x = None
        self.next_y = None
        self.direction = 1  # 1: forward along X, -1: backward along X

        # Timer to call update
        self.timer = self.create_timer(0.1, self.update)

    def pose_cb(self, msg):
        # Find UAV pose from dynamic entities
        for entity in msg.data:
            if entity.name == 'uav_platform':
                p = Pose()
                p.position.x = entity.pose.position.x
                p.position.y = entity.pose.position.y
                p.position.z = entity.pose.position.z
                # ignoring orientation for now
                self.uav_pose = p
                break

    def goto(self, x, y):
        """Move UAV toward (x, y) and return True if reached"""
        if self.uav_pose is None:
            return False

        dx = x - self.uav_pose.position.x
        dy = y - self.uav_pose.position.y
        dist = math.hypot(dx, dy)

        cmd = Twist()
        if dist < 0.2:
            # reached target
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            self.cmd_pub.publish(cmd)
            return True
        else:
            angle = math.atan2(dy, dx)
            cmd.linear.x = self.speed * math.cos(angle)
            cmd.linear.y = self.speed * math.sin(angle)
            cmd.linear.z = 0.0
            self.cmd_pub.publish(cmd)
            return False

    def update(self):
        if self.state == 'goto_start':
            # fly to bottom-left corner
            start_x = self.box[0]
            start_y = self.box[1]
            reached = self.goto(start_x, start_y)
            if reached:
                self.state = 'coverage'
                self.next_x = self.box[0]
                self.next_y = self.box[1]
                self.direction = 1
                self.get_logger().info('Reached start, beginning coverage.')

        elif self.state == 'coverage':
            reached = self.goto(self.next_x, self.next_y)
            if reached:
                # Move to next line along Y
                self.next_y += self.step_size
                if self.next_y > self.box[3]:
                    self.get_logger().info('Coverage complete!')
                    self.state = 'done'
                    self.cmd_pub.publish(Twist())  # stop UAV
                else:
                    # Switch X direction for lawn-mower pattern
                    if self.direction == 1:
                        self.next_x = self.box[2]
                    else:
                        self.next_x = self.box[0]
                    self.direction *= -1

def main():
    rclpy.init()
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
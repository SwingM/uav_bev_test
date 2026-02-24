#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose


class LawnMowerAutopilot(Node):
    def __init__(self):
        super().__init__('lawnmower_autopilot')

        self.declare_parameter('entity_name', 'flying_camera')
        self.declare_parameter('altitude_m', 20.0)
        self.declare_parameter('ground_size_m', 40.0)
        self.declare_parameter('stripe_spacing_m', 5.0)
        self.declare_parameter('speed_mps', 2.0)
        self.declare_parameter('update_hz', 10.0)

        self.entity_name = self.get_parameter('entity_name').value
        self.altitude_m = float(self.get_parameter('altitude_m').value)
        self.ground_size_m = float(self.get_parameter('ground_size_m').value)
        self.stripe_spacing_m = float(self.get_parameter('stripe_spacing_m').value)
        self.speed_mps = float(self.get_parameter('speed_mps').value)
        self.update_hz = float(self.get_parameter('update_hz').value)

        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.get_logger().info('Waiting for /gazebo/set_entity_state service...')
        self.client.wait_for_service()

        self.path_points = self._build_lawnmower_path()
        self.current_segment = 0
        self.progress = 0.0

        self.timer = self.create_timer(1.0 / self.update_hz, self._step)
        self.get_logger().info(f'Autopilot started for entity={self.entity_name}')

    def _build_lawnmower_path(self):
        half = self.ground_size_m / 2.0
        rows = max(2, int(math.ceil(self.ground_size_m / self.stripe_spacing_m)) + 1)
        ys = [-half + i * (self.ground_size_m / (rows - 1)) for i in range(rows)]

        points = []
        left = -half
        right = half
        for i, y in enumerate(ys):
            if i % 2 == 0:
                points.append((left, y, self.altitude_m))
                points.append((right, y, self.altitude_m))
            else:
                points.append((right, y, self.altitude_m))
                points.append((left, y, self.altitude_m))
        return points

    def _step(self):
        if len(self.path_points) < 2:
            return

        p0 = self.path_points[self.current_segment]
        p1 = self.path_points[(self.current_segment + 1) % len(self.path_points)]

        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        dz = p1[2] - p0[2]
        seg_len = math.sqrt(dx * dx + dy * dy + dz * dz)
        if seg_len < 1e-6:
            self.current_segment = (self.current_segment + 1) % len(self.path_points)
            self.progress = 0.0
            return

        self.progress += (self.speed_mps / self.update_hz) / seg_len
        if self.progress >= 1.0:
            self.current_segment = (self.current_segment + 1) % len(self.path_points)
            self.progress = 0.0
            return

        x = p0[0] + self.progress * dx
        y = p0[1] + self.progress * dy
        z = p0[2] + self.progress * dz

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0

        state = EntityState()
        state.name = self.entity_name
        state.pose = pose

        req = SetEntityState.Request()
        req.state = state
        self.client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = LawnMowerAutopilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

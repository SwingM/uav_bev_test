# uav_pose_publisher.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros_gz_interfaces.msg import Entity

class UAVPosePublisher(Node):
    def __init__(self):
        super().__init__('uav_pose_pub')

        # 发布话题
        self.pose_pub = self.create_publisher(Pose, '/uav_platform/pose', 10)

        # 订阅 Gazebo 实体信息
        self.subscription = self.create_subscription(
            Entity,
            '/model/manager/entities',  # Humble / Garden 默认实体话题
            self.entity_callback,
            10
        )

        self.drone_name = 'uav_platform'  # SDF 模型名

    def entity_callback(self, msg: Entity):
        if msg.name == self.drone_name:
            pose = Pose()
            pose.position.x = msg.pose.position.x
            pose.position.y = msg.pose.position.y
            pose.position.z = msg.pose.position.z
            pose.orientation = msg.pose.orientation
            self.pose_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = UAVPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
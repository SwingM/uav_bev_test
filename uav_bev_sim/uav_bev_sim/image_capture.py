#!/usr/bin/env python3
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageCaptureNode(Node):
    def __init__(self) -> None:
        super().__init__('image_capture')

        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('output_dir', 'captures')
        self.declare_parameter('save_every_n', 5)

        camera_topic = self.get_parameter('camera_topic').value
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.save_every_n = int(self.get_parameter('save_every_n').value)

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.frame_counter = 0

        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.on_image,
            10,
        )
        self.get_logger().info(f'Saving images to: {self.output_dir.resolve()}')

    def on_image(self, msg: Image) -> None:
        self.frame_counter += 1
        if self.frame_counter % self.save_every_n != 0:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        timestamp = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}"
        output_path = self.output_dir / f'image_{timestamp}.png'
        cv2.imwrite(str(output_path), cv_image)
        self.get_logger().info(f'Saved {output_path.name}')


def main() -> None:
    rclpy.init()
    node = ImageCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSaver(Node):
    def __init__(self):
        super().__init__('bev_image_saver')

        self.declare_parameter('image_topic', '/flying_camera/camera/image_raw')
        self.declare_parameter('save_dir', 'bev_images')
        self.declare_parameter('save_every_n_frames', 10)

        topic = self.get_parameter('image_topic').value
        self.save_dir = Path(self.get_parameter('save_dir').value)
        self.save_every_n = int(self.get_parameter('save_every_n_frames').value)

        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.counter = 0

        self.sub = self.create_subscription(Image, topic, self.image_cb, 10)
        self.get_logger().info(f'Subscribed to {topic}, saving to {self.save_dir}')

    def image_cb(self, msg: Image):
        self.counter += 1
        if self.counter % self.save_every_n != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        out = self.save_dir / f'bev_{ts}.png'
        cv2.imwrite(str(out), frame)
        self.get_logger().info(f'Saved {out}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

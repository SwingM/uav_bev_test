#!/usr/bin/env python3
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class StitchingNode(Node):
    def __init__(self) -> None:
        super().__init__('stitching_node')

        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('output_file', 'mosaic_result.png')
        self.declare_parameter('process_every_n', 10)

        camera_topic = self.get_parameter('camera_topic').value
        self.output_file = Path(self.get_parameter('output_file').value)
        self.process_every_n = int(self.get_parameter('process_every_n').value)

        self.bridge = CvBridge()
        self.orb = cv2.ORB_create(1500)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        self.frame_counter = 0
        self.reference = None
        self.canvas = None
        self.last_gray = None
        self.last_transform = np.eye(3, dtype=np.float64)

        self.subscription = self.create_subscription(Image, camera_topic, self.on_image, 10)
        self.get_logger().info('Stitching node started.')

    def on_image(self, msg: Image) -> None:
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n != 0:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.reference is None:
            self.reference = frame
            h, w = frame.shape[:2]
            self.canvas = np.zeros((h * 8, w * 8, 3), dtype=np.uint8)
            offset = np.array([[1, 0, w * 3], [0, 1, h * 3], [0, 0, 1]], dtype=np.float64)
            self.last_transform = offset
            self._blend(frame, self.last_transform)
            self.last_gray = gray
            self.get_logger().info('Initialized mosaic canvas.')
            return

        kp1, des1 = self.orb.detectAndCompute(self.last_gray, None)
        kp2, des2 = self.orb.detectAndCompute(gray, None)

        if des1 is None or des2 is None or len(kp1) < 12 or len(kp2) < 12:
            self.get_logger().warn('Not enough features for stitching frame.')
            return

        matches = self.matcher.match(des1, des2)
        if len(matches) < 20:
            self.get_logger().warn('Not enough feature matches for homography.')
            return

        matches = sorted(matches, key=lambda m: m.distance)[:200]
        src_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)

        h_mat, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        if h_mat is None or mask is None or int(mask.sum()) < 15:
            self.get_logger().warn('Homography rejected for current frame.')
            return

        self.last_transform = self.last_transform @ h_mat
        self._blend(frame, self.last_transform)
        self.last_gray = gray

    def _blend(self, image: np.ndarray, transform: np.ndarray) -> None:
        h, w = self.canvas.shape[:2]
        warped = cv2.warpPerspective(image, transform, (w, h))
        mask = np.any(warped > 0, axis=2)
        self.canvas[mask] = warped[mask]

    def save_result(self) -> None:
        if self.canvas is None:
            self.get_logger().warn('No canvas available to save.')
            return

        ys, xs = np.where(np.any(self.canvas > 0, axis=2))
        if len(xs) == 0 or len(ys) == 0:
            self.get_logger().warn('Mosaic canvas is empty.')
            return

        cropped = self.canvas[ys.min() : ys.max() + 1, xs.min() : xs.max() + 1]
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.output_file), cropped)
        self.get_logger().info(f'Saved mosaic: {self.output_file.resolve()}')


def main() -> None:
    rclpy.init()
    node = StitchingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_result()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

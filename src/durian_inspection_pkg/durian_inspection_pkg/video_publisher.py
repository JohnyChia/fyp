#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_pub')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.bridge = CvBridge()
        self.video_path = '/home/johny/durian_ws/data/durian.mp4'
        self.cap = cv2.VideoCapture(self.video_path)
        
        # 确认视频是否真的打开了
        if not self.cap.isOpened():
            self.get_logger().error(f"无法打开视频文件: {self.video_path}")
        else:
            self.get_logger().info("✅ 视频打开成功，开始推流...")
        
        self.publisher = self.create_publisher(Image, '/camera/image_raw', qos)
        self.timer = self.create_timer(0.05, self.timer_callback) # 约 30 FPS

    def timer_callback(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if ret:
            # 读取成功，发布图像
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header.frame_id = 'camera_link'
            self.publisher.publish(msg)
        else:
            # 🎥 视频结束，自动循环（无需 sleep，直接重置）
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
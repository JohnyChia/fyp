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

        if not self.cap.isOpened():
            self.get_logger().error(f"Unable to open video file: {self.video_path}")
        else:
            self.get_logger().info("Video opened successfully, starting stream...")
        
        self.publisher = self.create_publisher(Image, '/camera/image_raw', qos)
        self.timer = self.create_timer(0.05, self.timer_callback) 

    def timer_callback(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header.frame_id = 'camera_link'
            self.publisher.publish(msg)
        else:
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
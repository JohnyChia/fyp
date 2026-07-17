#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np
import sys
sys.path.append("/home/johny/durian_ws/src/Depth-Anything-V2") 
from depth_anything_v2.dpt import DepthAnythingV2

class DepthNode(Node):
    def __init__(self):
        super().__init__('depth_anything_node')
        self.bridge = CvBridge()
        
        model_configs = {'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]}}
        self.model = DepthAnythingV2(**model_configs['vits']).to('cuda').eval()
        self.model.load_state_dict(torch.load('/home/johny/durian_ws/models/depth.pth'))
        self.pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        
    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        depth = self.model.infer_image(frame) 
        depth_in_meters = depth * 5.0 
        
        depth_msg = self.bridge.cv2_to_imgmsg(depth_in_meters.astype(np.float32), "32FC1")
        depth_msg.header = msg.header
        depth_msg.header.frame_id = "camera_link_optical"
        self.pub.publish(depth_msg)

def main():
    rclpy.init(); rclpy.spin(DepthNode()); rclpy.shutdown()
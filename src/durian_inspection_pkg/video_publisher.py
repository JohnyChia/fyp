# #!/usr/bin/env python3

# import time
# import threading
# import requests
# import cv2
# import numpy as np

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge


# class VideoPublisher(Node):

#     def __init__(self):
#         super().__init__('video_pub')

#         self.bridge = CvBridge()

#         # Phone IP Camera URL
#         self.phone_stream_url = "http://192.168.100.8:8080/video"

#         self.latest_data = None
#         self.data_lock = threading.Lock()

#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.RELIABLE,
#             durability=DurabilityPolicy.VOLATILE,
#             depth=1
#         )

#         self.rgb_pub = self.create_publisher(
#             Image,
#             "/camera/image_raw",
#             qos_profile
#         )

#         self.info_pub = self.create_publisher(
#             CameraInfo,
#             "/camera/camera_info",
#             qos_profile
#         )

#         # Thread for grabbing frames
#         self.grab_thread = threading.Thread(
#             target=self.grab_frames_worker,
#             daemon=True
#         )
#         self.grab_thread.start()

#         # Thread for publishing frames
#         self.pub_thread = threading.Thread(
#             target=self.publish_loop,
#             daemon=True
#         )
#         self.pub_thread.start()

#         self.get_logger().info("Video Publisher Node Started.")

#     def grab_frames_worker(self):

#         while rclpy.ok():

#             try:
#                 stream = requests.get(
#                     self.phone_stream_url,
#                     stream=True,
#                     timeout=5
#                 )

#                 stream.raise_for_status()

#                 bytes_data = b''

#                 for chunk in stream.iter_content(chunk_size=1024):

#                     bytes_data += chunk

#                     start = bytes_data.find(b'\xff\xd8')
#                     end = bytes_data.find(b'\xff\xd9')

#                     if start != -1 and end != -1:

#                         jpg = bytes_data[start:end + 2]
#                         bytes_data = bytes_data[end + 2:]

#                         frame = cv2.imdecode(
#                             np.frombuffer(jpg, dtype=np.uint8),
#                             cv2.IMREAD_COLOR
#                         )

#                         if frame is not None:

#                             stamp = self.get_clock().now().to_msg()

#                             with self.data_lock:
#                                 self.latest_data = (frame, stamp)

#             except Exception as e:

#                 self.get_logger().warning(
#                     f"Stream error: {e}, retrying..."
#                 )

#                 time.sleep(1.0)

#     def publish_loop(self):
#         publish_period = 0.2  # 5 Hz
        
#         while rclpy.ok():
#             data = None
#             with self.data_lock:
#                 if self.latest_data is not None:
#                     # 获取当前最新的数据
#                     data = self.latest_data
#                     # 获取后立即置空，确保下次循环必须拿到“新”帧
#                     self.latest_data = None

#             if data is not None:
#                 frame, _ = data # 忽略旧的时间戳

#                 # 【关键修复】发布时生成绝对最新的时间戳
#                 now_stamp = self.get_clock().now().to_msg()
                
#                 # 1. 发布 Image
#                 frame = cv2.resize(frame, (320, 240))
#                 rgb_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
#                 rgb_msg.header.stamp = now_stamp
#                 rgb_msg.header.frame_id = "camera_link_optical"
#                 self.rgb_pub.publish(rgb_msg)

#                 # 2. 发布 CameraInfo
#                 info = CameraInfo()
#                 info.header.stamp = now_stamp
#                 info.header.frame_id = "camera_link_optical"
#                 info.width = 320
#                 info.height = 240
#                 info.k = [262.5, 0.0, 159.75, 0.0, 262.5, 119.75, 0.0, 0.0, 1.0]
#                 info.p = [262.5, 0.0, 159.75, 0.0, 0.0, 262.5, 119.75, 0.0, 0.0, 0.0, 1.0, 0.0]
#                 self.info_pub.publish(info)

#             time.sleep(publish_period)


# def main(args=None):

#     rclpy.init(args=args)

#     node = VideoPublisher()

#     try:
#         rclpy.spin(node)

#     except KeyboardInterrupt:
#         pass

#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()
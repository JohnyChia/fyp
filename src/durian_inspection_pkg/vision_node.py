
#!/usr/bin/env python3
import os
import time
import traceback
from std_msgs.msg import Header
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np
import threading
import queue
from ultralytics import YOLO
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rtabmap_msgs.msg import RGBDImage
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from durian_message.srv import TreeStatus

class VisionNode(Node):
    def __init__(self):
        os.environ["LD_LIBRARY_PATH"] = "/usr/local/cuda/lib64:" + os.environ.get("LD_LIBRARY_PATH", "")
        os.environ["CUDA_VISIBLE_DEVICES"] = "0"

        super().__init__('vision_node')
        self.models_ready = False
        
        self.declare_parameter("model_tree_path", "/home/johny/durian_ws/models/durian_tree/best_v8.pt")
        self.declare_parameter("model_leaf_path", "/home/johny/durian_ws/models/durian_leaf/best_v26.pt")
        self.declare_parameter("depth_min_meters", 0.5)
        self.declare_parameter("depth_max_meters", 8.0)
        
        self.tree_config = {
            'd101': {'name': 'Musang King', 'is_disease': False},
            'd175': {'name': 'Red Prawn', 'is_disease': False},
            'd197': {'name': 'Durian 197', 'is_disease': False},
            'd2':   {'name': 'Durian 2', 'is_disease': False},
            'd24':  {'name': 'Durian 24', 'is_disease': False},
            'Leaf_Algal':           {'name': 'Algal Spot', 'remedy': 'Copper-based', 'severity': 0.4},
            'Leaf_Blight':          {'name': 'Blight', 'remedy': 'Difenoconazole', 'severity': 0.8},
            'Leaf_Colletotrichum':  {'name': 'Anthracnose', 'remedy': 'Azoxystrobin', 'severity': 0.6},
            'Leaf_Healthy':         {'name': 'Healthy', 'remedy': 'None', 'severity': 0.0},
            'Leaf_Phomopsis':       {'name': 'Phomopsis', 'remedy': 'Carbendazim', 'severity': 0.7},
            'Leaf_Rhizoctonia':     {'name': 'Rhizoctonia', 'remedy': 'Jinggangmycin', 'severity': 0.8}
        }

        self.durian_type = {0: 'd101', 1: 'd175', 2: 'd197', 3: 'd2', 4: 'd24'}
        self.severity_map = {
            'Leaf_Healthy': 0.0, 'Leaf_Algal': 0.4, 'Leaf_Blight': 0.8,
            'Leaf_Colletotrichum': 0.6, 'Leaf_Phomopsis': 0.7, 'Leaf_Rhizoctonia': 0.8
        }

        self.model_tree = None
        self.model_leaf = None
        self.transform = None
        self.latest_depth_frame = None
        self.depth_mult = 5.0  
        self.pixel_to_m = 0.002
        self.depth_frame_counter = 0
        self.tree_tracker = {}
        self.next_tree_id = 0
        self.camera_info_msg = None
        self.gpu_lock = threading.Lock()

        self.bridge = CvBridge()
        self.frame_queue = queue.Queue(maxsize=1)
        self.marker_queue = queue.Queue(maxsize=1) 
        
        self._cached_device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.load_all_models()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        callback_group = ReentrantCallbackGroup()

        self.marker_pub = self.create_publisher(MarkerArray, '/tree_markers', 10)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_sensor/camera_info", 
            self.camera_info_callback, sub_qos, callback_group=callback_group
        )
        self.image_sub = self.create_subscription(
            Image,  "/camera/image_raw", 
            self.image_callback, sub_qos, callback_group=callback_group
        )

        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image_raw", 
            self.depth_callback, 10
        )
        self.srv_status = self.create_service(TreeStatus, 'get_tree_status', self.handle_get_status)
        self.last_detected_tree = {}

        self.create_timer(0.5, self.delayed_init)

        self.tree_pose_pub = self.create_publisher(PoseStamped, '/discovered_tree_pose', 10)

        self.yolo_thread = threading.Thread(target=self.yolo_worker, daemon=True)
        self.yolo_thread.start()

    def delayed_init(self):
        self.load_all_models()
        # 销毁定时器，只执行一次
        self.destroy_timer(self.create_timer)

    def load_all_models(self):
        try:
            self.get_logger().info("Loading models onto GPU...")
            device = self._cached_device
            self.model_tree = YOLO(self.get_parameter('model_tree_path').value).to(device)
            self.model_leaf = YOLO(self.get_parameter('model_leaf_path').value).to(device)
            self.models_ready = True
            self.get_logger().info("All models loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"Critical error: Failed to load models: {e}")

    def image_callback(self, msg):
        if not self.models_ready:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        if self.marker_queue.full():
            try: self.marker_queue.get_nowait()
            except queue.Empty: pass

        try:
            self.marker_queue.put( (cv_image.copy(), msg.header.stamp), block=False )
        except queue.Full:
            pass

    def camera_info_callback(self, msg):
        msg.header.frame_id = "camera_link_optical"
        self.camera_info_msg = msg

    def depth_callback(self, msg):
        try:
            self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Depth callback error: {e}")
            
    def yolo_worker(self):
        while rclpy.ok():
            time.sleep(1.0) 
            if not self.marker_queue.empty():
                cv_image, stamp = self.marker_queue.get()
                
                with self.gpu_lock:
                    self.detect_and_publish_trees(cv_image, stamp)
       
    def detect_and_publish_trees(self, frame, stamp):
        try:
            # 增加一个检查：如果 TF 还没准备好，不要盲目去 lookup
            if not self.tf_buffer.can_transform("odom", "camera_link_optical", stamp):
                return
            
            trans = self.tf_buffer.lookup_transform("odom", "camera_link_optical", stamp, timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            # 记录一下为什么转换失败
            self.get_logger().debug(f"TF lookup failed: {e}")
            return
        
        marker_array = MarkerArray()
        
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.ns = "durian_trees"
        marker_array.markers.append(delete_marker)
        
        delete_text = Marker()
        delete_text.action = Marker.DELETEALL
        delete_text.ns = "durian_info_text"
        marker_array.markers.append(delete_text)

        if self.camera_info_msg is None:
            return

        fx = self.camera_info_msg.k[0]
        fy = self.camera_info_msg.k[4]
        cx_cam = self.camera_info_msg.k[2]
        cy_cam = self.camera_info_msg.k[5]
        
        height, width = frame.shape[:2]

        with torch.no_grad():
            results = self.model_tree(frame, verbose=False)
           
            for box in results[0].boxes:
                if box.conf < 0.4: continue
                
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

                if self.latest_depth_frame is not None:
                    d_cy = min(cy, self.latest_depth_frame.shape[0] - 1)
                    d_cx = min(cx, self.latest_depth_frame.shape[1] - 1)
                    depth_val = self.latest_depth_frame[d_cy, d_cx]

                    if depth_val <= 0.1 or depth_val > 10.0 or np.isnan(depth_val):
                        depth_val = 2.0 
                else:
                    depth_val = 2.0 

                pt = PointStamped()
                pt.header.frame_id = "camera_link_optical"
                pt.header.stamp = stamp
                
                pt.point.z = float(depth_val) 
                pt.point.x = float((cx - cx_cam) * depth_val / fx)
                pt.point.y = float((cy - cy_cam) * depth_val / fy)

                try:
                    map_pt = tf2_geometry_msgs.do_transform_point(pt, trans)
                    real_x, real_y = map_pt.point.x, map_pt.point.y
                except Exception:
                    continue

                matched_id = None
                for tid, (tx, ty) in self.tree_tracker.items():
                    if np.sqrt((real_x - tx)**2 + (real_y - ty)**2) < 0.8:
                        matched_id = tid
                        break
                
                if matched_id is None:
                    matched_id = self.next_tree_id
                    self.next_tree_id += 1
                
                self.tree_tracker[matched_id] = (real_x, real_y)

                self.last_detected_tree[matched_id] = {
                    'type': type_info['name'],
                    'status': disease_info['name']
                }

                self.process_tree(frame, box, matched_id, marker_array, stamp, real_x, real_y)

        if len(marker_array.markers) > 2: 
            self.marker_pub.publish(marker_array)

    def handle_get_status(self, request, response):
        # 根据请求的 tree_id 返回最新数据
        data = self.last_detected_tree.get(request.tree_id, {})
        response.durian_type = data.get('type', 'Unknown')
        response.disease_status = data.get('status', 'Healthy')
        return response
                
    def detect_durian_type(self, crop):
        if crop.size == 0: return "Unknown"
        result = self.model_tree(crop, verbose=False, conf=0.20)
        if len(result[0].boxes) == 0: return "Unknown"
        class_id = int(result[0].boxes.cls.cpu().numpy()[0])
        return self.durian_type.get(class_id, "Unknown")
    
    def process_tree(self, frame, box, index, marker_array, stamp, real_x, real_y):
        height, width = frame.shape[:2]
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "odom"
        pose_msg.header.stamp = stamp
        pose_msg.pose.position.x = real_x
        pose_msg.pose.position.y = real_y
        pose_msg.pose.orientation.w = 1.0
        self.tree_pose_pub.publish(pose_msg)
        
        durian_type_id = "d101"
        d_name = "Healthy"
        severity = 0.0
        
        if self.model_leaf:
            crop = frame[max(0,y1):min(height,y2), max(0,x1):min(width,x2)]
            if crop.size > 0:
                durian_type_id = self.detect_durian_type(crop)
                d_name, severity = self.detect_leaf_disease(crop)

        type_info = self.tree_config.get(durian_type_id, {'name': 'Unknown'})
        if type_info.get('name') == 'Unknown':
            return 

        old_tree = self.tree_tracker.get(index)
        if old_tree is not None:
            real_x = 0.7 * old_tree[0] + 0.3 * real_x
            real_y = 0.7 * old_tree[1] + 0.3 * real_y
    
        disease_info = self.tree_config.get(d_name, {'name': 'Healthy', 'remedy': 'None'})
        
        marker_array.markers.append(self.create_tree_marker(index, real_x, real_y, severity, stamp))
        marker_array.markers.append(self.create_text_marker(index, real_x, real_y, type_info['name'], disease_info, stamp))

    def detect_leaf_disease(self, crop):
        d_name = "Healthy"
        severity = 0.0
        if crop.size == 0: return d_name, severity
        result = self.model_leaf(crop, verbose=False, conf=0.01)
        if len(result[0].boxes) == 0: return d_name, severity
        d_name = self.model_leaf.names[int(result[0].boxes.cls[0])]
        severity = self.severity_map.get(d_name, 0.0)
        return d_name, severity
        
    def create_tree_marker(self, index, real_x, real_y, severity, stamp):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = stamp
        marker.ns = "durian_trees"
        marker.id = index
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = real_x
        marker.pose.position.y = real_y
        marker.pose.position.z = 0.15
        marker.scale.x, marker.scale.y, marker.scale.z = 0.3, 0.3, 0.3
        marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        marker.color.a = 1.0
        if severity < 0.3:
            marker.color.g = 1.0
        elif severity < 0.6:
            marker.color.r = 1.0
            marker.color.g = 1.0
        else:
            marker.color.r = 1.0
        return marker
    
    # 【修复】：补全此前被莫名截断的标记生成逻辑
    def create_text_marker(self, index, real_x, real_y, type_name, disease_info, stamp):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = stamp
        marker.ns = "durian_info_text"
        marker.id = index + 1000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        if disease_info.get('remedy') == 'None':
            marker.text = f"Type: {type_name}\nStatus: Healthy"
        else:
            marker.text = f"Type: {type_name}\nDisease: {disease_info['name']}\nRemedy: {disease_info['remedy']}" 

        marker.pose.position.x = real_x
        marker.pose.position.y = real_y
        marker.pose.position.z = 0.6 
        marker.scale.z = 0.20
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 1.0
        return marker
        
def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
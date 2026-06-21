import os
import time

from std_msgs.msg import Header
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import Marker,MarkerArray
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np
import scipy.ndimage
import threading
import queue
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros
import tf2_geometry_msgs
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from rclpy.callback_groups import ReentrantCallbackGroup

class VisionNode(Node):
    def __init__(self):
        os.environ["LD_LIBRARY_PATH"] = "/usr/local/cuda/lib64:" + os.environ.get("LD_LIBRARY_PATH", "")
        os.environ["CUDA_VISIBLE_DEVICES"] = "0"

        super().__init__('vision_node')
        self.models_ready = False
        self.declare_parameter("model_tree_path", "/home/johny/durian_ws/models/durian_tree/best_v8.pt")
        self.declare_parameter("model_leaf_path", "/home/johny/durian_ws/models/durian_leaf/best_v26.pt")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('lidar_use_normalization', True),
                ('lidar_depth_to_meter_scale', 20.0),
                ('lidar_angle_min', -3.14159),
                ('lidar_angle_max', 3.14159),
                ('lidar_range_min', 0.1),
                ('lidar_range_max', 20.0),
                ('lidar_depth_offset_meter', 0.5)
            ]
        )


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

        self.durian_type = {
            0: 'd101',
            1: 'd175',
            2: 'd197',
            3: 'd2',
            4: 'd24'
        }
        
        self.severity_map = {
            'Leaf_Healthy': 0.0,
            'Leaf_Algal': 0.4,
            'Leaf_Blight': 0.8,
            'Leaf_Colletotrichum': 0.6,
            'Leaf_Phomopsis': 0.7,
            'Leaf_Rhizoctonia': 0.8
        }

        self.model_tree = None
        self.model_leaf = None
        self.midas = None
        self.transform = None
        self.frame_count = 0
        self.depth_mult = 5.0  
        self.pixel_to_m = 0.002
        self.tree_tracker = {}
        self.next_tree_id = 0
        self.last_depth_map = None
        self.depth_update_count = 0 

        self.bridge = CvBridge()
        self.frame_queue = queue.Queue(maxsize=1)
        self.inference_lock = threading.Lock()
        self._cached_device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"深度估计计算单元已初始化为: {self._cached_device}")
        self.load_all_models()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            durability=DurabilityPolicy.VOLATILE, 
            depth=100
        )

        self.scan_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, 
            durability=DurabilityPolicy.VOLATILE, 
            depth=100
        )

        self.depth_pub = self.create_publisher(Image, '/camera/depth_image', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/tree_markers', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/tree_cloud', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_processed', self.scan_qos_profile)

        self.cb_group = ReentrantCallbackGroup()
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, self.qos_profile,callback_group=self.cb_group)

        self.timer = self.create_timer(0.033, self.timer_callback, callback_group=self.cb_group)

    def load_all_models(self):
        try:
            self.get_logger().info("Loading all models onto GPU...")
            device = torch.device("cuda:0")

            self.midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small", trust_repo=True).to(device).eval()
            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
            self.transform = midas_transforms.small_transform

            self.model_tree = YOLO(self.get_parameter('model_tree_path').value).to(device)
            self.model_leaf = YOLO(self.get_parameter('model_leaf_path').value).to(device)
            
            self.models_ready = True
            self.get_logger().info("Model loading complete!")
        except Exception as e:
            self.get_logger().error(f"Critical error: Failed to load models: {e}")

    def ensure_models_loaded(self):
        with self.inference_lock:
            if getattr(self, 'models_ready', False):
                return True
            
            try:
                self.get_logger().info("Starting to load models onto GPU")
                device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
                
                self.midas = torch.hub.load("/home/johny/.cache/torch/hub/intel-isl_MiDaS_master", 
                                            "MiDaS_small", source='local', trust_repo=True).to(device).eval()

                midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", source='github', trust_repo=True)
                self.transform = midas_transforms.small_transform
                self.model_tree = YOLO(self.get_parameter('model_tree_path').value).to(device)
                self.model_leaf = YOLO(self.get_parameter('model_leaf_path').value).to(device)
                
                self.models_ready = True
                self.get_logger().info("All models successfully loaded onto GPU")
                return True
            except Exception as e:
                self.get_logger().error(f"An error occurred while loading models: {str(e)}")
                return False

    def processing_worker(self):
        while rclpy.ok():
            try:
                frame, stamp = self.frame_queue.get(timeout=1.0)
                if self.models_ready:
                    self.run_pipeline_logic(frame, stamp)
            except queue.Empty:
                continue
        
    def image_callback(self, msg):
        try:
            if self.frame_queue.full():
                self.frame_queue.get_nowait()
            self.frame_queue.put_nowait((self.bridge.imgmsg_to_cv2(msg, "bgr8"), msg.header.stamp))
        except Exception as e:
            self.get_logger().error(f"Callback 错误: {e}")

    def timer_callback(self):
        try:
            if not self.frame_queue.empty():
                frame, stamp = self.frame_queue.get_nowait()
                if self.models_ready:
                    self.run_pipeline_logic(frame, stamp)
        except Exception as e:
            self.get_logger().error(f"Pipeline error: {e}")

    def run_pipeline_logic(self, frame, stamp):
        self.depth_update_count += 1
        if self.depth_update_count >= 3:
            self.last_depth_map = self.compute_depth(frame)
            self.depth_update_count = 0

            with self.inference_lock:
                self.detect_and_publish_trees(frame, self.last_depth_map)

        if self.last_depth_map is not None:
            self.publish_scan(self.last_depth_map)

    def compute_depth(self, frame):
        if self.midas is None or self.transform is None:
            return np.zeros((frame.shape[0], frame.shape[1]), dtype=np.float32)

        try:
            small_frame = cv2.resize(frame, (256, 256))
            img_rgb = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
            input_tensor = self.transform(img_rgb)

            if input_tensor.dim() == 3:
                input_tensor = input_tensor.unsqueeze(0)
            elif input_tensor.dim() == 5:
                input_tensor = input_tensor.squeeze(0)
            
            input_tensor = input_tensor.to(self._cached_device)
            
            with torch.no_grad():
                prediction = self.midas.to(self._cached_device)(input_tensor)
                
                depth = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1) if prediction.dim() == 3 else prediction, 
                    size=(frame.shape[0], frame.shape[1]), 
                    mode="nearest"
                ).squeeze().cpu().numpy()
            
            return depth
            
        except Exception as e:
            self.get_logger().error(f"Critical error: Exception in depth processing: {e}")
            return np.zeros((frame.shape[0], frame.shape[1]), dtype=np.float32)
        
    def publish_scan(self, depth):
        row = depth[depth.shape[0] // 2].astype(np.float32)
        row = scipy.ndimage.median_filter(row, size=5) 
        
        if self.get_parameter('lidar_use_normalization').value:
            row = (row - np.min(row)) / (np.max(row) - np.min(row) + 1e-6)
        
        distances = row * self.get_parameter('lidar_depth_to_meter_scale').value
        num_scans = 360 
        indices = np.linspace(0, len(distances) - 1, num_scans)
        interpolated_ranges = np.interp(indices, np.arange(len(distances)), distances)

        min_range = float(self.get_parameter('lidar_range_min').value)
        max_range = float(self.get_parameter('lidar_range_max').value)
        cleaned_ranges = np.clip(interpolated_ranges, min_range, max_range)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser"
        scan.angle_min = float(self.get_parameter('lidar_angle_min').value)
        scan.angle_max = float(self.get_parameter('lidar_angle_max').value)
        scan.angle_increment = (scan.angle_max - scan.angle_min) / num_scans
        scan.range_min = min_range
        scan.range_max = max_range
        scan.ranges = cleaned_ranges.tolist()
        self.scan_pub.publish(scan)

    def detect_and_publish_trees(self, frame, depth_map):
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.ns = "durian_trees"
        marker_array.markers.append(delete_marker)
        
        delete_text = Marker()
        delete_text.action = Marker.DELETEALL
        delete_text.ns = "durian_info_text"
        marker_array.markers.append(delete_text)

        try:
            trans = self.tf_buffer.lookup_transform("map", "camera_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f"Failed to get TF transform: {e}")
            return

        results = self.model_tree(frame, verbose=False)
        height, width = frame.shape[:2]
        current_time = self.get_clock().now().to_msg()

        for box in results[0].boxes:
            if box.conf < 0.4: continue
            
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            depth_val = depth_map[min(cy, height - 1), min(cx, width - 1)]
            normalized_depth = (depth_val - np.min(depth_map)) / (np.max(depth_map) - np.min(depth_map) + 1e-6)

            pt = PointStamped()
            pt.header.frame_id = "camera_link"
            pt.header.stamp = current_time
            pt.point.x = float(0.5 + normalized_depth * self.depth_mult)
            pt.point.y = float((cx - width/2) * self.pixel_to_m)
            pt.point.z = 0.0

            map_pt = tf2_geometry_msgs.do_transform_point(pt, trans)
            real_x, real_y = map_pt.point.x, map_pt.point.y

            matched_id = None
            for tid, (tx, ty) in self.tree_tracker.items():
                if np.sqrt((real_x - tx)**2 + (real_y - ty)**2) < 0.5:
                    matched_id = tid
                    break
            
            if matched_id is None:
                matched_id = self.next_tree_id
                self.next_tree_id += 1
            
            self.tree_tracker[matched_id] = (real_x, real_y)
            self.process_tree(frame, box, matched_id, marker_array, current_time, real_x, real_y)

        if len(marker_array.markers) > 2:
            self.marker_pub.publish(marker_array)
       

    def detect_durian_type(self, crop):

        if crop.size == 0:
            return "Unknown"

        result = self.model_tree(
            crop,
            verbose=False,
            conf=0.20
        )

        if len(result[0].boxes) == 0:
            return "Unknown"

        if len(result[0].boxes) == 0:
            return "Unknown"

        class_id = int(result[0].boxes.cls.cpu().numpy()[0])

        return self.durian_type.get(
            class_id,
            "Unknown"
        )
    
    def process_tree(self, frame, box, index, marker_array, stamp, real_x, real_y):

        height, width = frame.shape[:2]
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        
        durian_type_id = "d101"
        d_name = "Healthy"
        severity = 0.0
        
        if self.model_leaf:
            crop = frame[max(0,y1):min(height,y2), max(0,x1):min(width,x2)]
            if crop.size > 0:
                durian_type_id = self.detect_durian_type(crop)
                d_name, severity = self.detect_leaf_disease(crop)

        type_info = self.tree_config.get(durian_type_id, {'name': 'Unknown'})

        old_tree = self.tree_tracker.get(index)

        if old_tree is not None:
            real_x = 0.7 * old_tree[0] + 0.3 * real_x
            real_y = 0.7 * old_tree[1] + 0.3 * real_y
        self.publish_trees_as_cloud()

        if type_info.get('name') == 'Unknown':
            self.get_logger().info(f"Skipping Unknown tree at index {index}")
            return 

        disease_info = self.tree_config.get(d_name, {'name': 'Healthy', 'remedy': 'None'})
        marker_array.markers.append(self.create_tree_marker(index, real_x, real_y, severity, stamp))
        marker_array.markers.append(self.create_text_marker(index, real_x, real_y, type_info['name'], disease_info, stamp))

    def detect_leaf_disease(self, crop):

        d_name = "Healthy"
        severity = 0.0

        if crop.size == 0:
            return d_name, severity

        result = self.model_leaf(
            crop,
            verbose=False,
            conf=0.01
        )

        if len(result[0].boxes) == 0:
            return d_name, severity

        d_name = self.model_leaf.names[
            int(result[0].boxes.cls[0])
        ]

        severity = self.severity_map.get(d_name, 0.0)

        return d_name, severity
    
    def publish_trees_as_cloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        points = [[x, y, 0.0] for x, y in self.tree_tracker.values()]
        cloud_msg = pc2.create_cloud(header, fields, points)
        self.cloud_pub.publish(cloud_msg)
        
    def create_tree_marker(
        self,
        index,
        real_x,
        real_y,
        severity,
        stamp
    ):

        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = stamp

        marker.ns = "durian_trees"
        marker.id = index

        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = real_x
        marker.pose.position.y = real_y

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.lifetime = (
            rclpy.duration.Duration(
                seconds=1.0
            ).to_msg()
        )

        marker.color.a = 1.0

        if severity < 0.3:
            marker.color.g = 1.0

        elif severity < 0.6:
            marker.color.r = 1.0
            marker.color.g = 1.0

        else:
            marker.color.r = 1.0

        return marker
    
    def create_text_marker(self, index, real_x, real_y, type_name, disease_info, stamp):

        marker = Marker()

        marker.header.frame_id = "map"
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
        marker.pose.position.z = (
            1.2 + index * 0.1
        )

        marker.scale.z = 0.25

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        return marker

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
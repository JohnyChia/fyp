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

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.models_ready = False
        
        # 1. 基础配置与变量声明
        self.bridge = CvBridge()
        self.frame_queue = queue.Queue(maxsize=1)
        self.inference_lock = threading.Lock()
        
        # 2. 声明参数（必须先声明）
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
            
            # 将模型三的病害名称与药物信息绑定
            'Leaf_Algal':           {'name': 'Algal Spot', 'remedy': 'Copper-based', 'severity': 0.4},
            'Leaf_Blight':          {'name': 'Blight', 'remedy': 'Difenoconazole', 'severity': 0.8},
            'Leaf_Colletotrichum':  {'name': 'Anthracnose', 'remedy': 'Azoxystrobin', 'severity': 0.6},
            'Leaf_Healthy':         {'name': 'Healthy', 'remedy': 'None', 'severity': 0.0},
            'Leaf_Phomopsis':       {'name': 'Phomopsis', 'remedy': 'Carbendazim', 'severity': 0.7},
            'Leaf_Rhizoctonia':     {'name': 'Rhizoctonia', 'remedy': 'Jinggangmycin', 'severity': 0.8}
        }

        # 补全这个缺失的属性，解决 Attribute Error
        self.durian_type = {
            0: 'd101',
            1: 'd175',
            2: 'd197',
            3: 'd2',
            4: 'd24'
        }
        
        # 还要保留你的严重程度映射（detect_leaf_disease 需要它）
        self.severity_map = {
            'Leaf_Healthy': 0.0,
            'Leaf_Algal': 0.4,
            'Leaf_Blight': 0.8,
            'Leaf_Colletotrichum': 0.6,
            'Leaf_Phomopsis': 0.7,
            'Leaf_Rhizoctonia': 0.8
        }

        
        # 3. 初始化模型占位符 (不要在这里调用 load)
        self.model_tree = None
        self.model_leaf = None
        self.midas = None
        self.transform = None
        self.frame_count = 0
        self.depth_mult = 5.0  
        self.pixel_to_m = 0.002
        self.tree_tracker = {}
        self.next_tree_id = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 4. 初始化通信
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, 
                         durability=DurabilityPolicy.VOLATILE, depth=100)
        
        self.depth_pub = self.create_publisher(Image, '/camera/depth_image', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/tree_markers', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/tree_cloud', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos)
        
        # 5. 最后再启动线程 (确保所有参数和对象已就绪)
        threading.Thread(target=self.processing_worker, daemon=True).start()

    def ensure_models_loaded(self):
        if getattr(self, 'models_ready', False):
            return True
        
        # 直接硬编码路径，完全去掉 torch.hub 的网络请求行为
        try:
            self.get_logger().info("正在尝试离线加载模型...")

            model_type = "MiDaS_small"
            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
            self.transform = midas_transforms.small_transform if model_type == "MiDaS_small" else midas_transforms.dpt_transform
                        
            # 强制从本地路径加载，不要使用 torch.hub 的在线下载能力
            # 确保 /home/johny/.cache/torch/hub/intel-isl_MiDaS_master 文件夹下确实有模型文件
            self.midas = torch.hub.load("/home/johny/.cache/torch/hub/intel-isl_MiDaS_master", 
                                        "MiDaS_small", 
                                        source='local', 
                                        trust_repo=True).eval()
            
            model_path = self.get_parameter('model_tree_path').value
            leaf_path = self.get_parameter('model_leaf_path').value
            self.model_tree = YOLO(model_path)
            self.model_leaf = YOLO(leaf_path)
            
            self.models_ready = True
            return True
        except Exception as e:
            self.get_logger().error(f"模型加载彻底失败: {e}")
            return False
        
    def image_callback(self, msg):
        try:
            # 尝试非阻塞入队，如果队列满则丢弃旧帧，保证实时性
            if self.frame_queue.full():
                self.frame_queue.get_nowait()
            self.frame_queue.put_nowait((self.bridge.imgmsg_to_cv2(msg, "bgr8"), msg.header.stamp))
        except Exception as e:
            self.get_logger().error(f"Callback 错误: {e}")

    def processing_worker(self):
        while rclpy.ok():
            try:
                frame, stamp = self.frame_queue.get(timeout=1.0)
                
                # --- 增加：严苛的模型就绪检查 ---
                if not self.ensure_models_loaded():
                    self.get_logger().warn("模型尚未准备就绪，无法执行检测任务。")
                    continue 
                # 检查模型是否真的不为 None
                if self.model_tree is None:
                    self.get_logger().error("model_tree 为空，跳过！")
                    continue
                # ------------------------------

                depth_map = self.compute_depth(frame)
                self.publish_scan(depth_map)
                
                self.frame_count += 1
                if self.frame_count >= 5:
                    with self.inference_lock:
                        self.detect_and_publish_trees(frame, depth_map, stamp)
                    self.frame_count = 0
            except Exception as e:
                self.get_logger().error(f"Worker 循环异常: {str(e)}")

    def run_pipeline_logic(self, frame, stamp):
        # 注意：这里不再递归调用自己，只负责执行算法
        
        # 1. 总是执行深度推理（保证避障实时性）
        depth_map = self.compute_depth(frame)
       
        # 2. 只有每隔 N 帧才执行重型任务（检测树木）
        self.frame_count += 1
        if self.frame_count % 5 == 0:
            with self.inference_lock:
                self.detect_and_publish_trees(frame, depth_map,stamp)
                self.frame_count = 0

    def compute_depth(self, frame):
        # 1. 基础检查
        if self.midas is None or self.transform is None:
            return np.zeros((frame.shape[0], frame.shape[1]), dtype=np.float32)

        try:
            # 2. 预处理
            small_frame = cv2.resize(frame, (320, 240)) 
            img_rgb = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
            
            # 3. 维度校准
            input_tensor = self.transform(img_rgb)
            
            # 关键修复：强制将 input_tensor 变为 [1, 3, 240, 320]
            # 如果它是 [1, 1, 3, 240, 320] 或 [1, 3, 240, 320]，我们统一处理
            if input_tensor.dim() > 4:
                input_tensor = input_tensor.view(-1, 3, 240, 320)
            elif input_tensor.dim() == 3:
                input_tensor = input_tensor.unsqueeze(0)
            
            # 4. 推理
            with torch.no_grad():
                prediction = self.midas(input_tensor)
                
                # resize 回原图大小
                # 注意：如果 prediction 是 [1, 1, H, W]，则不需要再 unsqueeze(1)
                if prediction.dim() == 3:
                    prediction = prediction.unsqueeze(1)
                    
                depth = torch.nn.functional.interpolate(
                    prediction, 
                    size=frame.shape[:2], 
                    mode="bicubic", 
                    align_corners=False
                ).squeeze().cpu().numpy()
            
            return depth
            
        except Exception as e:
            self.get_logger().error(f"深度计算异常: {e}")
            return np.zeros((frame.shape[0], frame.shape[1]), dtype=np.float32)
        
    def publish_scan(self, depth):
        # 1. 提取中心行并进行中值滤波（消除异常 inf 干扰）
        row = depth[depth.shape[0] // 2].astype(np.float32)
        row = scipy.ndimage.median_filter(row, size=5) 
        
        # 2. 归一化与缩放
        if self.get_parameter('lidar_use_normalization').value:
            row = (row - np.min(row)) / (np.max(row) - np.min(row) + 1e-6)
        
        distances = row * self.get_parameter('lidar_depth_to_meter_scale').value
        
        # 3. 关键修复：确保 ranges 的数量与预期的扫描点数匹配
        # 假设我们想要发布 360 个点，如果 row 只有 640 个点，我们需要插值
        num_scans = 360 
        indices = np.linspace(0, len(distances) - 1, num_scans)
        interpolated_ranges = np.interp(indices, np.arange(len(distances)), distances)
        
        # 4. 动态数据清洗：只把真正超出物理极限的设为 inf
        min_range = float(self.get_parameter('lidar_range_min').value)
        max_range = float(self.get_parameter('lidar_range_max').value)
        cleaned_ranges = np.clip(interpolated_ranges, min_range, max_range)
        
        # 5. 构建并发布
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser"
        scan.angle_min = float(self.get_parameter('lidar_angle_min').value)
        scan.angle_max = float(self.get_parameter('lidar_angle_max').value)
        scan.angle_increment = (scan.angle_max - scan.angle_min) / num_scans
        scan.range_min = min_range
        scan.range_max = max_range
        scan.ranges = cleaned_ranges.tolist()

        print("SCAN TIME:", scan.header.stamp)

        self.scan_pub.publish(scan)

    def detect_and_publish_trees(self, frame, depth_map, stamp):
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.ns = "durian_trees"
        marker_array.markers.append(delete_marker)
        
        delete_text = Marker()
        delete_text.action = Marker.DELETEALL
        delete_text.ns = "durian_info_text"
        marker_array.markers.append(delete_text)

        results = self.model_tree(frame, verbose=False)
        height, width = frame.shape[:2]

        for box in results[0].boxes:
            if box.conf < 0.4: continue
            
            # 1. 计算相机坐标系下的相对位置
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            depth_val = depth_map[min(cy, height - 1), min(cx, width - 1)]
            normalized_depth = (depth_val - np.min(depth_map)) / (np.max(depth_map) - np.min(depth_map) + 1e-6)
            
            # 这是相机系下的坐标
            camera_x = float(0.5 + normalized_depth * self.depth_mult)
            camera_y = float((cx - width/2) * self.pixel_to_m)
            
            # 2. 将坐标封装为 PointStamped
            pt = PointStamped()
            pt.header.frame_id = "camera_link" # 假设你的相机坐标系名为 camera_link
            pt.header.stamp = stamp
            pt.point.x = camera_x
            pt.point.y = camera_y
            pt.point.z = 0.0

            # 3. 执行 TF 转换
            try:
                # 等待 TF 转换可用
                if self.tf_buffer.can_transform("map", "camera_link", stamp, timeout=rclpy.duration.Duration(seconds=0.1)):
                    map_pt = self.tf_buffer.transform(pt, "map")
                    real_x = map_pt.point.x
                    real_y = map_pt.point.y
                else:
                    self.get_logger().warn("TF 转换失败，跳过该帧")
                    continue
            except Exception as e:
                self.get_logger().error(f"TF 异常: {e}")
                continue
            
            matched_id = None
            for tid, (tx, ty) in self.tree_tracker.items():
                # 距离判断使用 map 坐标
                if np.sqrt((real_x - tx)**2 + (real_y - ty)**2) < 0.5:
                    matched_id = tid
                    break
            
            if matched_id is None:
                matched_id = self.next_tree_id
                self.next_tree_id += 1
            
            # 存储坐标也使用 map 坐标
            self.tree_tracker[matched_id] = (real_x, real_y)
            
            # 传入 real_x, real_y
            self.process_tree(frame, box, matched_id, marker_array, stamp, real_x, real_y)

        if len(marker_array.markers) > 0:
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

        class_id = int(result[0].boxes.cls[0])

        return self.durian_type.get(
            class_id,
            "Unknown"
        )
    
    def process_tree(self, frame, box, index, marker_array, stamp, real_x, real_y):

        height, width = frame.shape[:2]
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        
        durian_type = "d101"
        d_name = "Healthy"
        severity = 0.0
        
        if self.model_leaf:
            crop = frame[max(0,y1):min(height,y2), max(0,x1):min(width,x2)]
            if crop.size > 0:
                # 1. 获取品种 ID (d101, d175等)
                durian_type_id = self.detect_durian_type(crop)
                # 2. 获取病害名称 (Leaf_Algal等)
                d_name, severity = self.detect_leaf_disease(crop)

        # 核心：利用 tree_config 获取显示名称和药物
        type_info = self.tree_config.get(durian_type_id, {'name': 'Unknown'})

        self.tree_tracker[index] = (real_x, real_y)
        self.publish_trees_as_cloud(stamp)
        
        # --- 新增：过滤 Unknown ---
        if type_info.get('name') == 'Unknown':
            self.get_logger().info(f"Skipping Unknown tree at index {index}")
            return  # 直接跳过，不添加任何 Marker
        # ------------------------
        disease_info = self.tree_config.get(d_name, {'name': 'Healthy', 'remedy': 'None'})

        # 添加标记
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
    
    def publish_trees_as_cloud(self, stamp):
        header = Header()
        header.stamp = stamp  # 必须直接赋值给属性
        header.frame_id = "map"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        # 获取跟踪到的所有树的坐标
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
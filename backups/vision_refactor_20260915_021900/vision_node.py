# FULL VISION NODE AUDIT FIX: session-safe evidence / target association
import os
import math
import time
import traceback
from std_msgs.msg import Header, Bool, Int32
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import torch
import cv2
import numpy as np
import threading
from collections import deque
import queue
from ultralytics import YOLO
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rtabmap_msgs.msg import RGBDImage
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from durian_message.srv import TreeStatus

class VisionNode(Node):
    def log_diagnostic(self, msg):
        self.get_logger().info(msg)

    def __init__(self):
        os.environ["LD_LIBRARY_PATH"] = "/usr/local/cuda/lib64:" + os.environ.get("LD_LIBRARY_PATH", "")
        os.environ["CUDA_VISIBLE_DEVICES"] = "0"
        super().__init__('vision_node')
        self.models_ready = False
        self.declare_parameter("model_tree_path", "/home/johny/durian_ws/models/durian_tree/best_tree_hybrid_v2.pt")
        self.declare_parameter("model_fruit_path", "/home/johny/durian_ws/models/durian/best_v8.pt")
        self.declare_parameter("model_leaf_path", "/home/johny/durian_ws/experiment/outputs/production_efficientnet/weights/best.pt")
        self.declare_parameter('leaf_inference_scales', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('leaf_roi_context_scale', 1.5)
        self.declare_parameter('leaf_roi_unreliable_context_scale', 1.25)
        self.declare_parameter('leaf_roi_max_area_ratio', 0.85)
        self.declare_parameter('leaf_roi_min_size', 80)
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
        self.calibrate_colors()
        self.models_ready = False
        self.scan_disease_votes = {}
        self.scan_active = False
        self.scan_target_tree_id = None
        self.scan_target_map_x = 0.0
        self.scan_target_map_y = 0.0
        self.model_tree = None
        self.model_fruit = None
        self.model_leaf = None
        self.transform = None
        self.latest_depth_frame = None
        self.latest_depth_stamp = None
        self.depth_buffer = deque(maxlen=30)
        self.depth_mult = 5.0
        self.MAX_RGB_DEPTH_DT = 0.15  
        self.pixel_to_m = 0.002
        self.depth_frame_counter = 0
        self.debug_frame_counter = 0
        self.tree_tracker = {}
        self.next_tree_id = 0
        self.camera_info_msg = None
        self.gpu_lock = threading.Lock()
        self.tree_roi_cache = {}
        self.MAX_SCAN_ROI_AGE = 10.0

        # Active-scan target lock:
        # authoritative physical target identity is independent of the
        # transient vision tracking_id.  The lock prevents a weak transient
        # Tree detection from replacing a stronger target merely because its
        # 3D position is marginally closer.
        self.scan_target_lock = {
            'epoch': None,
            'target_id': None,
            'tracking_id': None,
            'real_x': None,
            'real_y': None,
            'real_z': None,
            'conf': 0.0,
            'last_seen': 0.0
        }
        self.SCAN_TARGET_MIN_CONF = 0.20
        self.SCAN_TARGET_MAX_DISTANCE = 1.5
        self.SCAN_TARGET_LOCK_DISTANCE = 0.75

        self.bridge = CvBridge()
        self.frame_queue = queue.Queue(maxsize=1)
        self.marker_queue = queue.Queue(maxsize=1) 
        self._cached_device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.load_all_models()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        callback_group = ReentrantCallbackGroup()
        self.marker_pub = self.create_publisher(MarkerArray, '/tree_markers', 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", 
            self.camera_info_callback, sub_qos, callback_group=callback_group
        )
        self.image_sub = self.create_subscription(
            Image,  "/camera/image_raw", 
            self.image_callback, sub_qos, callback_group=callback_group
        )
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image_raw", 
            self.depth_callback, sub_qos, callback_group=callback_group
        )
        self.scan_target_sub = self.create_subscription(
            Point,
            "/scan_target_tree",
            self.scan_target_callback,
            10,
            callback_group=callback_group
        )
        self.scan_active_sub = self.create_subscription(
            Bool,
            "/scan_active",
            self.scan_active_callback,
            10,
            callback_group=callback_group
        )
        srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.srv_status = self.create_service(TreeStatus, 'get_tree_status', self.handle_get_status, callback_group=srv_cb_group)
        self.last_detected_tree = {}
        
        self.scan_epoch = 0
        self.state_lock = threading.RLock()

        self.init_timer = self.create_timer(0.5, self.delayed_init)
        self.tree_pose_pub = self.create_publisher(PoseStamped, '/discovered_tree_pose', 10)
        self.yolo_thread = threading.Thread(target=self.yolo_worker, daemon=True)
        self.yolo_thread.start()

    def delayed_init(self):
        self.load_all_models()
        if hasattr(self, 'init_timer') and self.init_timer:
            self.init_timer.cancel()

    def load_all_models(self):
        try:
            import torch
            torch.set_num_threads(2)
            self.get_logger().info("Loading models onto GPU...")
            device = self._cached_device
            self.model_tree_path = self.get_parameter('model_tree_path').value
            self.model_fruit_path = self.get_parameter('model_fruit_path').value
            self.model_leaf_path = self.get_parameter('model_leaf_path').value
            self.model_tree = YOLO(self.model_tree_path).to(device)
            self.model_fruit = YOLO(self.model_fruit_path).to(device)
            self.model_leaf = YOLO(self.model_leaf_path).to(device)
            self.models_ready = True
            self.get_logger().info("All models loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"Critical error: Failed to load models: {e}")

    def image_callback(self, msg):
        if not self.models_ready:
            return
            
        with self.state_lock:
            if getattr(self, 'finalizing_scan', False):
                return
            frame_scan_active = self.scan_active
            frame_target_id = self.scan_target_tree_id
            frame_epoch = self.scan_epoch
            frame_map_x = getattr(self, 'scan_target_map_x', 0.0)
            frame_map_y = getattr(self, 'scan_target_map_y', 0.0)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            import hashlib
            callback_hash = hashlib.sha256(cv_image.tobytes()).hexdigest()
            queue_size_before = self.marker_queue.qsize()
            self.log_diagnostic(
                f"[IMAGE CALLBACK FORENSIC] "
                f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
                f"frame_id={msg.header.frame_id} "
                f"shape={cv_image.shape} "
                f"hash={callback_hash} "
                f"queue_before={queue_size_before}"
            )
            self.get_logger().info(f"image_callback: frame received, shape={cv_image.shape}, dtype={cv_image.dtype}", throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return
            
        if self.marker_queue.full():
            try:
                self.marker_queue.get_nowait()
                self.marker_queue.task_done()
            except queue.Empty:
                pass
        try:
            frame_context = {
                'stamp': msg.header.stamp,
                'scan_active': frame_scan_active,
                'target_id': frame_target_id,
                'epoch': frame_epoch,
                'map_x': frame_map_x,
                'map_y': frame_map_y
            }
            self.marker_queue.put( (cv_image.copy(), frame_context), block=False )
        except queue.Full:
            pass

    def camera_info_callback(self, msg):
        msg.header.frame_id = "camera_link_optical"
        with self.state_lock:
            self.camera_info_msg = msg

    def scan_target_callback(self, msg):
        with self.state_lock:
            self.scan_target_map_x = msg.x
            self.scan_target_map_y = msg.y
            new_target_id = int(msg.z)
            
            if new_target_id != self.scan_target_tree_id:
                self.scan_target_tree_id = new_target_id
                with self.state_lock:
                    self.last_valid_odom_target = None

                # The local vision tracking_id belongs to the previous
                # authoritative target and must never leak into the new one.
                self.scan_target_lock = {
                    'epoch': self.scan_epoch,
                    'target_id': new_target_id,
                    'tracking_id': None,
                    'real_x': None,
                    'real_y': None,
                    'real_z': None,
                    'conf': 0.0,
                    'last_seen': 0.0
                }

                # Reset evidence whenever this authoritative target belongs
                # to a different scan epoch.  This prevents an old inspection
                # from contaminating a later inspection of the same tree ID.
                evidence_key = (int(self.scan_epoch), int(self.scan_target_tree_id))
                self.scan_disease_votes[evidence_key] = {
                    'epoch': int(self.scan_epoch),
                    'target_id': int(self.scan_target_tree_id),
                    'counts': {}, 'conf_sum': {},
                    'total_crops': 0, 'yolo_frames': 0,
                    'valid_frames': 0, 'rejected_frames': 0,
                    'sum_raw_conf': 0.0, 'max_raw_conf': 0.0,
                    'max_valid_conf': 0.0
                }
            
            # Read variables into local for logging outside the main logic flow to keep it clean, though it's still under lock
            log_target_id = self.scan_target_tree_id
            log_map_x = self.scan_target_map_x
            log_map_y = self.scan_target_map_y
            
        self.get_logger().info(
            f"[SCAN TARGET] authoritative_id={log_target_id} map_x={log_map_x:.2f} map_y={log_map_y:.2f}"
        )

    def scan_active_callback(self, msg):
        with self.state_lock:
            self.scan_active = bool(msg.data)
            if self.scan_active:
                self.scan_epoch += 1
                self.finalizing_scan = False
            else:
                # Close the current scan session immediately.  Image callbacks
                # delivered late by the ROS executor must not enter a scan
                # after the inspection server has declared it complete.
                self.finalizing_scan = True
                self.scan_target_tree_id = None
                self.scan_target_map_x = 0.0
                self.scan_target_map_y = 0.0
                
            log_active = self.scan_active
            log_target_id = self.scan_target_tree_id
            
        self.get_logger().info(
            f"[SCAN ACTIVE] active={log_active}, target_id={log_target_id}"
        )

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

            depth_t = (
                float(msg.header.stamp.sec)
                + float(msg.header.stamp.nanosec) * 1e-9
            )

            with self.state_lock:
                self.latest_depth_frame = depth
                self.latest_depth_stamp = msg.header.stamp
                self.depth_buffer.append(
                    (depth_t, msg.header.stamp, depth)
                )

        except Exception as e:
            self.get_logger().error(f"Depth callback error: {e}")

    def yolo_worker(self):
        while rclpy.ok():
            time.sleep(0.1)
            try:
                cv_image, frame_context = self.marker_queue.get_nowait()
            except queue.Empty:
                continue

            import hashlib
            worker_hash = hashlib.sha256(cv_image.tobytes()).hexdigest()
            worker_stamp = frame_context.get('stamp')
            self.log_diagnostic(
                f"[YOLO WORKER FORENSIC] "
                f"stamp={worker_stamp.sec}.{worker_stamp.nanosec:09d} "
                f"hash={worker_hash} "
                f"queue_after_get={self.marker_queue.qsize()}"
            )

            self.get_logger().info(
                "yolo_worker: processing frame",
                throttle_duration_sec=2.0
            )

            try:
                with self.gpu_lock:
                    self.detect_and_publish_trees(
                        cv_image,
                        frame_context
                    )
            except Exception as e:
                import traceback
                self.get_logger().error(
                    f"Crash in yolo_worker: {e}\n{traceback.format_exc()}"
                )
            finally:
                self.marker_queue.task_done()

    def calculate_iou(self, boxA, boxB):
        xA, yA = max(boxA[0], boxB[0]), max(boxA[1], boxB[1])
        xB, yB = min(boxA[2], boxB[2]), min(boxA[3], boxB[3])
        interArea = max(0, xB - xA) * max(0, yB - yA)
        boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
        boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
        return interArea / float(boxAArea + boxBArea - interArea + 1e-5)

    def detect_and_publish_trees(self, frame, frame_context):
        import numpy as np
        if not getattr(self, 'models_ready', False):
            return
            
        stamp = frame_context.get('stamp')
        frame_scan_active = frame_context.get('scan_active', False)
        frame_target_id = frame_context.get('target_id', None)
        frame_epoch = frame_context.get('epoch', 0)
        frame_map_x = frame_context.get('map_x', 0.0)
        frame_map_y = frame_context.get('map_y', 0.0)
            
        ts_str = f"{stamp.sec}.{stamp.nanosec}"
        
        try:
            # Use the TF corresponding to this RGB frame timestamp.
            # This keeps the 3D projection temporally consistent with the image.
            trans = self.tf_buffer.lookup_transform(
                "odom",
                "camera_link_optical",
                stamp
            )
        
        except Exception as e:
            self.get_logger().warning(f"TF lookup failed: {e}", throttle_duration_sec=2.0)
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
        with self.state_lock:
            camera_info_msg = self.camera_info_msg

        if camera_info_msg is None:
            self.get_logger().warning("Missing camera_info_msg", throttle_duration_sec=2.0)
            return
        fx = camera_info_msg.k[0]
        fy = camera_info_msg.k[4]
        cx_cam = camera_info_msg.k[2]
        cy_cam = camera_info_msg.k[5]
        height, width = frame.shape[:2]
        
        self.log_diagnostic(
            f"[TREE YOLO INPUT]\n"
            f"shape={frame.shape}\n"
            f"dtype={frame.dtype}\n"
            f"min={frame.min()}\n"
            f"max={frame.max()}\n"
            f"mean={frame.mean():.2f}"
        )
        
        MIN_TREE_CONF = 0.005
        self.log_diagnostic(
            f"[TREE YOLO CONFIG]\n"
            f"model={self.model_tree_path}\n"
            f"imgsz=default\n"
            f"conf={MIN_TREE_CONF}\n"
            f"device={self.model_tree.device}"
        )
        
        cv2.imwrite("/home/johny/durian_ws/debug_tree_yolo_input.jpg", frame)
        self.log_diagnostic(f"[TREE YOLO DEBUG IMAGE]\npath=/home/johny/durian_ws/debug_tree_yolo_input.jpg\nshape={frame.shape}")
        
        # ============================================================
        # LIVE TREE YOLO FORENSIC
        # Exact in-memory frame + exact Ultralytics res.boxes.
        # Detection/NMS/tracking logic is unchanged.
        # ============================================================
        import hashlib
        frame_hash_before = hashlib.sha256(frame.tobytes()).hexdigest()
        self.log_diagnostic(
            f"[TREE YOLO LIVE INPUT FORENSIC]\n"
            f"frame_id={id(frame)}\n"
            f"shape={frame.shape}\n"
            f"dtype={frame.dtype}\n"
            f"nbytes={frame.nbytes}\n"
            f"sha256_before={frame_hash_before}"
        )

        with torch.no_grad():
            NMS_IOU = 0.40
            
            all_raw_boxes = []
            
            res = self.model_tree(
                frame,
                verbose=False,
                conf=MIN_TREE_CONF,
                imgsz=1024
            )[0]

            frame_hash_after = hashlib.sha256(frame.tobytes()).hexdigest()
            actual_raw_boxes = []
            for raw_idx, raw_box in enumerate(res.boxes):
                raw_conf = float(raw_box.conf[0].item())
                raw_xyxy = (
                    raw_box.xyxy[0].tolist()
                    if hasattr(raw_box.xyxy[0], 'tolist')
                    else raw_box.xyxy[0]
                )
                actual_raw_boxes.append({
                    'idx': raw_idx,
                    'conf': raw_conf,
                    'bbox': [float(v) for v in raw_xyxy]
                })

            self.log_diagnostic(
                f"[TREE YOLO LIVE OUTPUT FORENSIC]\n"
                f"frame_id={id(frame)}\n"
                f"sha256_before={frame_hash_before}\n"
                f"sha256_after={frame_hash_after}\n"
                f"frame_unchanged={frame_hash_before == frame_hash_after}\n"
                f"actual_res_boxes={len(actual_raw_boxes)}\n"
                f"actual_raw_boxes={actual_raw_boxes}"
            )
            
            for box in res.boxes:
                conf = float(box.conf[0].item()) if hasattr(box, 'conf') else 1.0
                bx1, by1, bx2, by2 = box.xyxy[0].tolist() if hasattr(box.xyxy[0], 'tolist') else box.xyxy[0]
                
                all_raw_boxes.append({
                    'conf': conf,
                    'bbox': [bx1, by1, bx2, by2]
                })
            
            all_raw_boxes.sort(key=lambda x: x['conf'], reverse=True)
            nms_boxes = []
            for box in all_raw_boxes:
                discard = False
                for keep_box in nms_boxes:
                    if self.calculate_iou(box['bbox'], keep_box['bbox']) > NMS_IOU:
                        discard = True
                        break
                if not discard:
                    nms_boxes.append(box)
                    
            class MockBox:
                def __init__(self, bbox, conf):
                    self.xyxy = [bbox]
                    class MockConf:
                        def __init__(self, val):
                            self.val = val
                        def item(self):
                            return self.val
                    self.conf = [MockConf(conf)]
                    
            boxes = [MockBox(b['bbox'], b['conf']) for b in nms_boxes]
            
            raw_confs = [b['conf'] for b in nms_boxes]
            self.log_diagnostic(
                f"[TREE YOLO POST-NMS]\n"
                f"raw_box_count={len(all_raw_boxes)}\n"
                f"nms_survivor_count={len(boxes)}\n"
                f"nms_survivor_confidences={raw_confs}"
            )
            
            self.log_diagnostic(f"[TREE YOLO SUMMARY]\nboxes={len(boxes)}")
            if len(boxes) == 0:
                self.log_diagnostic(f"[TREE PIPELINE STOP]\nstage=TREE_YOLO\nreason=ZERO_DETECTIONS")
            
            valid_fresh_boxes = []
            used_tracks = set()
            
            for idx, box in enumerate(boxes):
                xyxy = box.xyxy[0].tolist() if hasattr(box.xyxy[0], 'tolist') else box.xyxy[0]
                x1, y1, x2, y2 = map(int, xyxy)
                conf = float(box.conf[0].item()) if hasattr(box, 'conf') else 1.0
                
                self.log_diagnostic(
                    f"[TREE YOLO]\nidx={idx}\nconf={conf:.4f}\nbbox={x1},{y1},{x2},{y2}\nsize={x2-x1}x{y2-y1}"
                )
                
                x1 = max(0, min(width - 1, x1))
                y1 = max(0, min(height - 1, y1))
                x2 = max(0, min(width, x2))
                y2 = max(0, min(height, y2))
                
                if x2 <= x1 or y2 <= y1:
                    self.log_diagnostic(f"[TREE REJECT REASON]\nreason=INVALID_BBOX\nconf={conf:.4f}\nbbox={x1},{y1},{x2},{y2}")
                    continue
                    
                bbox_w = x2 - x1
                bbox_h = y2 - y1
                
                touching_left = (x1 < 10)
                touching_right = (x2 > width - 10)
                touching_top = (y1 < 10)
                touching_bottom = (y2 > height - 10)
                
                touch_count = touching_left + touching_right + touching_top + touching_bottom
                
                if conf < 0.20 and touch_count >= 2:
                    self.log_diagnostic(f"[TREE REJECT REASON]\nreason=LOW_CONF_BOUNDARY_TOUCH\nconf={conf:.4f}\ntouch_count={touch_count}\nbbox={x1},{y1},{x2},{y2}\nthreshold_conf=0.20")
                    continue
                
                if bbox_w < 20 or bbox_h < 20:
                    self.log_diagnostic(
                        f"[TREE REJECT REASON]\nreason=TOO_SMALL\nconf={conf:.4f}\nbbox={x1},{y1},{x2},{y2}\nwidth={bbox_w}\nheight={bbox_h}\nthreshold_min=20"
                    )
                    continue
                    
                aspect = bbox_w / float(bbox_h)
                if aspect < 0.20 or aspect > 5.0:
                    self.log_diagnostic(
                        f"[TREE REJECT REASON]\nreason=INVALID_ASPECT\nconf={conf:.4f}\nbbox={x1},{y1},{x2},{y2}\naspect={aspect:.3f}\nthreshold=[0.20, 5.0]"
                    )
                    continue
                    
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

                # RGB/depth temporal synchronization.
                # The YOLO worker may process an older RGB frame after newer
                # depth frames have arrived. Select the nearest historical
                # depth frame instead of blindly using the latest depth frame.
                rgb_t = (
                    float(stamp.sec)
                    + float(stamp.nanosec) * 1e-9
                )

                matched_depth = None
                matched_depth_stamp = None
                matched_dt = float("inf")

                with self.state_lock:
                    for depth_t_i, depth_stamp_i, depth_frame_i in self.depth_buffer:
                        dt_i = abs(rgb_t - depth_t_i)

                        if dt_i < matched_dt:
                            matched_dt = dt_i
                            matched_depth = depth_frame_i
                            matched_depth_stamp = depth_stamp_i

                is_invalid_depth = True
                depth_val = None

                if (
                    matched_depth is not None
                    and matched_depth_stamp is not None
                    and matched_dt <= float(self.MAX_RGB_DEPTH_DT)
                ):
                    d_cy = min(cy, matched_depth.shape[0] - 1)
                    d_cx = min(cx, matched_depth.shape[1] - 1)

                    candidate_depth = float(
                        matched_depth[d_cy, d_cx]
                    )

                    if (
                        np.isfinite(candidate_depth)
                        and candidate_depth > 0.1
                        and candidate_depth <= 10.0
                    ):
                        depth_val = candidate_depth
                        is_invalid_depth = False
                    else:
                        self.log_diagnostic(
                            f"[DEPTH SYNC] INVALID_VALUE "
                            f"rgb_depth_dt={matched_dt:.4f}s "
                            f"depth={candidate_depth:.4f}"
                        )
                elif matched_depth is None:
                    self.log_diagnostic(
                        "[DEPTH SYNC] NO_DEPTH_TIMESTAMP"
                    )
                else:
                    self.log_diagnostic(
                        f"[DEPTH SYNC] STALE_DEPTH "
                        f"rgb_depth_dt={matched_dt:.4f}s "
                        f"max_allowed={self.MAX_RGB_DEPTH_DT:.3f}s"
                    )

                if is_invalid_depth:
                    # General Gazebo/real-camera fallback:
                    # An invalid depth sample (including the Gazebo far-plane
                    # value) must not discard an otherwise valid 2D Tree YOLO
                    # detection.  Keep the image-space bbox for downstream
                    # disease ROI processing, but do NOT fabricate 3D
                    # coordinates from invalid depth.
                    self.log_diagnostic(
                        "[DEPTH FALLBACK] INVALID_DEPTH_USING_2D_TREE_ROI"
                    )

                    real_x = None
                    real_y = None
                    real_z = None

                    self.log_diagnostic(
                        f"[DEPTH FALLBACK] bbox={x1},{y1},{x2},{y2} "
                        f"conf={conf:.4f}"
                    )
                else:
                    pt = PointStamped()
                    pt.header.frame_id = "camera_link_optical"
                    pt.header.stamp = stamp
                    pt.point.z = float(depth_val)
                    pt.point.x = float((cx - cx_cam) * depth_val / fx)
                    pt.point.y = float((cy - cy_cam) * depth_val / fy)
                    try:
                        odom_pt = tf2_geometry_msgs.do_transform_point(pt, trans)
                        real_x, real_y = odom_pt.point.x, odom_pt.point.y
                        real_z = odom_pt.point.z
                    except Exception as e:
                        self.log_diagnostic(
                            f"[TREE REJECT REASON]\nreason=TF_FAILURE\nconf={conf:.4f}\nbbox={x1},{y1},{x2},{y2}\nerror={e}"
                        )
                        continue
                    
                if real_x is not None and real_y is not None:
                    self.log_diagnostic(
                        f"[TREE POSITION]\n"
                        f"source_frame=camera_link_optical\n"
                        f"target_frame=odom\n"
                        f"x={real_x:.2f}\n"
                        f"y={real_y:.2f}"
                    )
                else:
                    self.log_diagnostic(
                        "[TREE POSITION] 2D_ONLY_NO_VALID_DEPTH"
                    )
                    
                tracking_id = None
                min_dist = float('inf')
                with self.state_lock:
                    tracker_snapshot = list(self.tree_tracker.items())

                for tid, tracker_data in tracker_snapshot:
                    if tid in used_tracks:
                        continue
                    tx, ty = tracker_data['x'], tracker_data['y']
                    if real_x is None or real_y is None:
                        dist = float("inf")
                    else:
                        dist = ((real_x - tx)**2 + (real_y - ty)**2)**0.5
                    if dist < min_dist and dist < 1.5:
                        tracking_id = tid
                        min_dist = dist
                
                dist_str = f"{min_dist:.2f}"
                if tracking_id is not None:
                    used_tracks.add(tracking_id)
                
                if tracking_id is None:
                    if is_invalid_depth:
                        # Invalid-depth detections are 2D-only observations.
                        # Do not create a 3D tracker or consume a persistent
                        # tracking ID because they cannot participate in
                        # 3D spatial association.
                        tracking_id = None
                        dist_str = "NO_3D_TRACK"
                        self.log_diagnostic(
                            "[TREE TRACKING]\n"
                            "candidate_id=None\n"
                            "distance=NO_3D_TRACK\n"
                            "association=2D_ONLY_NO_TRACK"
                        )
                    else:
                        dist_str = "NEW_TRACK"
                        tracking_id = self.next_tree_id
                        self.next_tree_id += 1
                        self.log_diagnostic(
                            f"[TREE TRACKING]\n"
                            f"candidate_id={tracking_id}\n"
                            f"distance=NEW_TRACK\n"
                            f"association=NEW_TRACK"
                        )
                else:
                    self.log_diagnostic(
                        f"[TREE TRACKING]\n"
                        f"candidate_id={tracking_id}\n"
                        f"distance={dist_str}\n"
                        f"association=EXISTING_TRACK"
                    )
                        
                with self.state_lock:
                    old_tracker = self.tree_tracker.get(tracking_id)
                    old_hit_count = (
                        old_tracker.get('hit_count', 0)
                        if old_tracker is not None
                        else 0
                    )
                new_hit_count = old_hit_count + 1
                
                if not is_invalid_depth:
                    if real_x is not None and real_y is not None:
                        with self.state_lock:
                            self.tree_tracker[tracking_id] = {
                                'x': real_x,
                                'y': real_y,
                                'z': real_z,
                                'last_seen': stamp.sec,
                                'hit_count': new_hit_count
                            }
                else:
                    with self.state_lock:
                        tracker_entry = self.tree_tracker.get(tracking_id)

                    if tracker_entry is not None:
                        with self.state_lock:
                            tracker_entry = self.tree_tracker.get(tracking_id)
                            if tracker_entry is not None:
                                tracker_entry['last_seen'] = stamp.sec
                                tracker_entry['hit_count'] = new_hit_count
                        self.log_diagnostic(
                            f"[DEPTH FALLBACK]\n"
                            f"reason=invalid_depth\n"
                            f"identity_retained={tracking_id}\n"
                            f"position_source=previous_valid_track"
                        )
                        with self.state_lock:
                            tracker_entry = self.tree_tracker.get(tracking_id)

                        if tracker_entry is not None:
                            real_x = tracker_entry['x']
                            real_y = tracker_entry['y']
                            real_z = tracker_entry.get('z', real_z)
                    else:
                        # Do not create a tracker entry containing None
                        # coordinates.  The detection remains usable as a
                        # 2D disease ROI, but it has no valid 3D position.
                        self.log_diagnostic(
                            f"[DEPTH FALLBACK]\n"
                            f"reason=invalid_depth_new_track\n"
                            f"identity_retained={tracking_id}\n"
                            f"position_source=2d_only"
                        )
                
                if frame_scan_active and frame_target_id is not None:
                    self.log_diagnostic(
                        f"[SCAN TARGET MATCH]\n"
                        f"target_frame=odom\n"
                        f"authoritative_target_id={frame_target_id}\n"
                        f"candidate_tracking_id={tracking_id}\n"
                        f"candidate_distance={dist_str}\n"
                        f"id_namespace_comparison=SKIPPED\n"
                        f"association_method=PHYSICAL_SPATIAL_MATCH"
                    )

                # During active inspection, keep low-confidence detections
                # until authoritative spatial association has been evaluated.
                # The later strong/weak spatial gates decide whether the
                # detection can become the inspection target.
                if conf < 0.20 and new_hit_count < 3 and not frame_scan_active:
                    self.log_diagnostic(
                        f"[TREE REJECT REASON]\n"
                        f"reason=LOW_CONF_INSUFFICIENT_TEMPORAL_EVIDENCE\n"
                        f"conf={conf:.4f}\n"
                        f"hit_count={new_hit_count}\n"
                        f"threshold_hits=3"
                    )
                    continue

                valid_fresh_boxes.append({
                    "box_obj": box,
                    "tracking_id": tracking_id,
                    "real_x": real_x,
                    "real_y": real_y,
                    "real_z": real_z,
                    "conf": conf,
                    "bbox": [x1, y1, x2, y2],
                    "x1": x1, "y1": y1, "x2": x2, "y2": y2
                })
                
            if not frame_scan_active or frame_target_id is None:
                for v in valid_fresh_boxes:
                    self.log_diagnostic(
                        f"[TREE IDENTITY]\n"
                        f"tracking_id={v['tracking_id']}\n"
                        f"previous_tracking_id={v['tracking_id']}\n"
                        f"scan_target_id=None\n"
                        f"vote_target_id=None"
                    )
                    self.process_tree(frame, v["box_obj"], v["tracking_id"], None, marker_array, stamp, v["real_x"], v["real_y"], v["real_z"], v["conf"])
            else:
                scan_target_id = frame_target_id
                
                odom_target_x = None
                odom_target_y = None
                odom_target_z = 1.2
                try:
                    trans = self.tf_buffer.lookup_transform("odom", "map", rclpy.time.Time())
                    pt = PointStamped()
                    pt.header.frame_id = "map"
                    pt.point.x = float(frame_map_x)
                    pt.point.y = float(frame_map_y)
                    pt.point.z = 1.2
                    odom_pt = tf2_geometry_msgs.do_transform_point(pt, trans)
                    odom_target_x = odom_pt.point.x
                    odom_target_y = odom_pt.point.y
                    odom_target_z = odom_pt.point.z
                    
                    with self.state_lock:
                        self.last_valid_odom_target = {
                            'x': odom_target_x,
                            'y': odom_target_y,
                            'z': odom_target_z,
                            'time': time.time()
                        }
                except Exception as e:
                    self.get_logger().warn(f"Failed to transform map->odom for target: {e}")
                    with self.state_lock:
                        cached_odom_target = (
                            dict(self.last_valid_odom_target)
                            if getattr(self, 'last_valid_odom_target', None)
                            else None
                        )

                    if cached_odom_target and (
                        time.time() - cached_odom_target['time'] < 5.0
                    ):
                        odom_target_x = cached_odom_target['x']
                        odom_target_y = cached_odom_target['y']
                        odom_target_z = cached_odom_target['z']
                        self.get_logger().warn(
                            f"Using cached odom target position from "
                            f"{time.time() - cached_odom_target['time']:.2f}s ago"
                        )
                    else:
                        odom_target_x = None
                        odom_target_y = None
                        odom_target_z = 1.2
                
                # Active inspection target is identified by authoritative
                # physical map/odom coordinates, NOT by local vision tracking_id.
                #
                # scan_target_id = inspection_server authoritative tree ID
                # v["tracking_id"] = vision-local dynamic track ID
                #
                # These ID namespaces must never be compared directly.
                #
                # IMPORTANT:
                # The old implementation selected the nearest candidate.
                # That allowed a very weak Tree YOLO detection to replace a
                # substantially stronger detection.  Selection is now
                # confidence-aware and target-lock aware.

                fresh_target = None
                min_target_dist = float('inf')
                target_association_mode = "NONE"

                if odom_target_x is not None and odom_target_y is not None:

                    spatial_candidates = []

                    for v in valid_fresh_boxes:
                        if v["real_x"] is None or v["real_y"] is None:
                            continue

                        dist = math.hypot(
                            v["real_x"] - odom_target_x,
                            v["real_y"] - odom_target_y
                        )

                        if dist <= self.SCAN_TARGET_MAX_DISTANCE:
                            spatial_candidates.append((v, dist))

                    # ----------------------------------------------------
                    # Confidence gate:
                    #
                    # If reliable candidates exist, weak candidates are
                    # completely excluded from target selection.
                    #
                    # This prevents e.g. conf=0.0059 from replacing
                    # conf=0.5378 simply because it is closer.
                    # ----------------------------------------------------
                    strong_candidates = [
                        (v, dist)
                        for v, dist in spatial_candidates
                        if v["conf"] >= self.SCAN_TARGET_MIN_CONF
                    ]

                    if strong_candidates:
                        # First preference is confidence.  Distance is only
                        # the tie-breaker among similarly reliable candidates.
                        strong_candidates.sort(
                            key=lambda item: (-item[0]["conf"], item[1])
                        )

                        fresh_target, min_target_dist = strong_candidates[0]
                        target_association_mode = "3D_STRONG"

                    else:
                        # No strong target exists in this frame.
                        # Allow a physically close weak candidate to provide
                        # the ROI, but only when it is within the strict
                        # physical lock distance. Strong candidates always
                        # take priority.
                        weak_candidates = sorted(
                            spatial_candidates,
                            key=lambda item: (item[1], -item[0]["conf"])
                        )

                        if weak_candidates:


                            # Keep weak candidates generic, but validate their ability to


                            # support the existing leaf ROI pipeline.


                            #


                            # No tree/model/scene-specific threshold is introduced here.


                            # Reuse the existing leaf_roi_min_size ROS parameter.


                            weak_usable_candidates = []



                            leaf_roi_min_size = float(


                                self.get_parameter("leaf_roi_min_size").value


                            )



                            for weak_candidate, weak_candidate_dist in weak_candidates:


                                weak_bbox = weak_candidate.get("bbox")



                                if weak_bbox is None or len(weak_bbox) < 4:


                                    self.log_diagnostic(


                                        f"[SCAN TARGET WEAK ROI REJECT]\n"


                                        f"authoritative_tree_id={scan_target_id}\n"


                                        f"candidate_tracking_id={weak_candidate.get('tracking_id')}\n"


                                        f"candidate_distance={weak_candidate_dist:.3f}\n"


                                        f"candidate_conf={weak_candidate.get('conf', 0.0):.4f}\n"


                                        f"reason=INVALID_BBOX"


                                    )


                                    continue



                                wx1, wy1, wx2, wy2 = map(float, weak_bbox[:4])


                                weak_w = max(1.0, wx2 - wx1)


                                weak_h = max(1.0, wy2 - wy1)


                                weak_min_dim = min(weak_w, weak_h)



                                if weak_min_dim < leaf_roi_min_size:


                                    self.log_diagnostic(


                                        f"[SCAN TARGET WEAK ROI REJECT]\n"


                                        f"authoritative_tree_id={scan_target_id}\n"


                                        f"candidate_tracking_id={weak_candidate.get('tracking_id')}\n"


                                        f"candidate_distance={weak_candidate_dist:.3f}\n"


                                        f"candidate_conf={weak_candidate.get('conf', 0.0):.4f}\n"


                                        f"bbox={wx1:.1f},{wy1:.1f},{wx2:.1f},{wy2:.1f}\n"


                                        f"size={weak_w:.1f}x{weak_h:.1f}\n"


                                        f"min_dim={weak_min_dim:.1f}\n"


                                        f"required_min_dim={leaf_roi_min_size:.1f}\n"


                                        f"reason=INSUFFICIENT_ROI_DIMENSION"


                                    )


                                    continue



                                weak_usable_candidates.append(


                                    (weak_candidate, weak_candidate_dist)


                                )



                            if weak_usable_candidates:


                                weak_best, weak_dist = weak_usable_candidates[0]



                                if weak_dist <= self.SCAN_TARGET_LOCK_DISTANCE:


                                    fresh_target = weak_best


                                    min_target_dist = weak_dist


                                    self.log_diagnostic(


                                        f"[SCAN TARGET SPATIAL WEAK FALLBACK]\n"


                                        f"authoritative_tree_id={scan_target_id}\n"


                                        f"candidate_tracking_id={weak_best['tracking_id']}\n"


                                        f"candidate_distance={weak_dist:.3f}\n"


                                        f"candidate_conf={weak_best['conf']:.4f}\n"


                                        f"spatial_fallback_max={self.SCAN_TARGET_LOCK_DISTANCE:.3f}\n"


                                        f"selected=True"


                                    )


                                else:


                                    self.log_diagnostic(


                                        f"[SCAN TARGET WEAK REJECT]\n"


                                        f"authoritative_tree_id={scan_target_id}\n"


                                        f"candidate_tracking_id={weak_best['tracking_id']}\n"


                                        f"candidate_distance={weak_dist:.3f}\n"


                                        f"candidate_conf={weak_best['conf']:.4f}\n"


                                        f"required_conf={self.SCAN_TARGET_MIN_CONF:.4f}\n"


                                        f"selected=False"


                                    )


                # ----------------------------------------------------
                # Generic 2D authoritative-target fallback.
                #
                # If no 3D candidate exists (for example Gazebo depth
                # returns the far-plane sentinel), project the
                # authoritative map target into the current RGB frame
                # using timestamp-matched TF and camera intrinsics.
                #
                # This is association-only: it never fabricates 3D
                # coordinates and never changes the Tree YOLO confidence.
                # ----------------------------------------------------
                if fresh_target is None and frame_map_x is not None and frame_map_y is not None:
                    try:
                        target_map = PointStamped()
                        target_map.header.frame_id = "map"
                        target_map.header.stamp = stamp
                        target_map.point.x = float(frame_map_x)
                        target_map.point.y = float(frame_map_y)
                        target_map.point.z = 1.2

                        map_to_odom = self.tf_buffer.lookup_transform(
                            "odom", "map", stamp
                        )
                        target_odom = tf2_geometry_msgs.do_transform_point(
                            target_map, map_to_odom
                        )

                        odom_to_camera = self.tf_buffer.lookup_transform(
                            "camera_link_optical", "odom", stamp
                        )
                        target_cam = tf2_geometry_msgs.do_transform_point(
                            target_odom, odom_to_camera
                        )

                        if target_cam.point.z > 0.05:
                            proj_x = (
                                target_cam.point.x * float(fx) /
                                target_cam.point.z + float(cx_cam)
                            )
                            proj_y = (
                                target_cam.point.y * float(fy) /
                                target_cam.point.z + float(cy_cam)
                            )

                            image_candidates = []
                            for v in valid_fresh_boxes:
                                x1, y1, x2, y2 = (
                                    float(v["x1"]), float(v["y1"]),
                                    float(v["x2"]), float(v["y2"])
                                )
                                w = max(x2 - x1, 1.0)
                                h = max(y2 - y1, 1.0)
                                inside = (
                                    x1 <= proj_x <= x2 and
                                    y1 <= proj_y <= y2
                                )

                                # Normalized distance from projected
                                # authoritative target to bbox center.
                                cx_box = (x1 + x2) * 0.5
                                cy_box = (y1 + y2) * 0.5
                                nd = math.hypot(
                                    (proj_x - cx_box) / w,
                                    (proj_y - cy_box) / h
                                )

                                # Prefer containment, then confidence,
                                # then geometric proximity.
                                if inside:
                                    image_candidates.append(
                                        (v, 0, nd)
                                    )

                            if image_candidates:
                                image_candidates.sort(
                                    key=lambda item: (
                                        item[1],
                                        -item[0]["conf"],
                                        item[2]
                                    )
                                )
                                fresh_target = image_candidates[0][0]
                                min_target_dist = image_candidates[0][2]
                                target_association_mode = "2D_PROJECTED"

                                self.log_diagnostic(
                                    f"[SCAN TARGET 2D FALLBACK]\n"
                                    f"authoritative_tree_id={scan_target_id}\n"
                                    f"projected_target=({proj_x:.1f},{proj_y:.1f})\n"
                                    f"candidate_tracking_id={fresh_target['tracking_id']}\n"
                                    f"candidate_bbox={fresh_target['x1']},{fresh_target['y1']},{fresh_target['x2']},{fresh_target['y2']}\n"
                                    f"candidate_conf={fresh_target['conf']:.4f}\n"
                                    f"normalized_center_distance={min_target_dist:.3f}\n"
                                    f"selected=True"
                                )
                            else:
                                self.log_diagnostic(
                                    f"[SCAN TARGET 2D FALLBACK]\n"
                                    f"authoritative_tree_id={scan_target_id}\n"
                                    f"projected_target=({proj_x:.1f},{proj_y:.1f})\n"
                                    f"candidates=0\n"
                                    f"selected=False"
                                )
                    except Exception as e:
                        self.log_diagnostic(
                            f"[SCAN TARGET 2D FALLBACK ERROR]\n"
                            f"authoritative_tree_id={scan_target_id}\n"
                            f"error={e}"
                        )

                    # ----------------------------------------------------
                    # Target lock:
                    #
                    # A local tracking_id may change between frames.
                    # Therefore the lock is physical, not ID-based.
                    #
                    # Once a reliable candidate has established the lock,
                    # a new candidate must still be physically consistent
                    # with the locked target before it can replace it.
                    # ----------------------------------------------------
                    with self.state_lock:
                        lock = dict(self.scan_target_lock)

                    lock_valid = (
                        lock.get('epoch') == frame_epoch and
                        lock.get('target_id') == scan_target_id and
                        lock.get('real_x') is not None and
                        lock.get('real_y') is not None
                    )

                    if (
                        fresh_target is not None
                        and lock_valid
                        and target_association_mode in ("3D_STRONG", "3D_WEAK")
                    ):
                        lock_dist = math.hypot(
                            fresh_target["real_x"] - lock["real_x"],
                            fresh_target["real_y"] - lock["real_y"]
                        )

                        if lock_dist > self.SCAN_TARGET_LOCK_DISTANCE:
                            self.log_diagnostic(
                                f"[SCAN TARGET LOCK REJECT]\n"
                                f"authoritative_tree_id={scan_target_id}\n"
                                f"candidate_tracking_id={fresh_target['tracking_id']}\n"
                                f"candidate_conf={fresh_target['conf']:.4f}\n"
                                f"lock_distance={lock_dist:.3f}\n"
                                f"max_lock_distance={self.SCAN_TARGET_LOCK_DISTANCE:.3f}\n"
                                f"selected=False"
                            )
                            fresh_target = None
                            min_target_dist = float('inf')

                    # ----------------------------------------------------
                    # Establish/update physical target lock.
                    #
                    # tracking_id is recorded only as diagnostic metadata;
                    # it is never used as target identity.
                    # ----------------------------------------------------
                    if (
                        fresh_target is not None
                        and target_association_mode in ("3D_STRONG", "3D_WEAK")
                    ):
                        with self.state_lock:
                            self.scan_target_lock = {
                                'epoch': frame_epoch,
                                'target_id': scan_target_id,
                                'tracking_id': fresh_target["tracking_id"],
                                'real_x': fresh_target["real_x"],
                                'real_y': fresh_target["real_y"],
                                'real_z': fresh_target["real_z"],
                                'conf': fresh_target["conf"],
                                'last_seen': time.time()
                            }

                        self.log_diagnostic(
                            f"[SCAN TARGET LOCK UPDATE]\n"
                            f"authoritative_tree_id={scan_target_id}\n"
                            f"tracking_id={fresh_target['tracking_id']}\n"
                            f"x={fresh_target['real_x']:.3f}\n"
                            f"y={fresh_target['real_y']:.3f}\n"
                            f"conf={fresh_target['conf']:.4f}"
                        )

                if fresh_target is not None:
                    if target_association_mode == "2D_PROJECTED":
                        self.log_diagnostic(
                            f"[SCAN TARGET MATCH]\\n"
                            f"target_frame=image\\n"
                            f"authoritative_tree_id={scan_target_id}\\n"
                            f"candidate_tracking_id={fresh_target['tracking_id']}\\n"
                            f"association=2D_PROJECTED\\n"
                            f"selected=True"
                        )
                    else:
                        self.log_diagnostic(
                            f"[SCAN TARGET SPATIAL MATCH]\\n"
                        f"authoritative_tree_id={scan_target_id}\\n"
                        f"target_map_x={frame_map_x:.3f}\\n"
                        f"target_map_y={frame_map_y:.3f}\\n"
                        f"target_odom_x={odom_target_x:.3f}\\n"
                        f"target_odom_y={odom_target_y:.3f}\\n"
                        f"candidate_tracking_id={fresh_target['tracking_id']}\\n"
                        f"candidate_x={fresh_target['real_x']:.3f}\\n"
                        f"candidate_y={fresh_target['real_y']:.3f}\\n"
                        f"candidate_distance={min_target_dist:.3f}\\n"
                            f"selected=True"
                        )

                if fresh_target is not None:
                    vote_target_id = scan_target_id
                    with self.state_lock:
                        previous_tracking_id = self.scan_target_lock.get(
                            'tracking_id'
                        )

                    self.log_diagnostic(
                        f"[TREE IDENTITY]\n"
                        f"tracking_id={fresh_target['tracking_id']}\n"
                        f"previous_tracking_id={previous_tracking_id}\n"
                        f"scan_target_id={scan_target_id}\n"
                        f"vote_target_id={vote_target_id}"
                    )
                    self.log_diagnostic(
                        f"[SCAN TARGET MATCH]\n"
                        f"target_frame=odom\n"
                        f"candidate_tracking_id={fresh_target['tracking_id']}\n"
                        f"candidate_distance={min_target_dist:.2f}\n"
                        f"selected=True"
                    )
                    self.log_diagnostic(
                        f"[SCAN ROI]\ntarget_id={scan_target_id}\nsource=FRESH\nbbox={fresh_target['x1']},{fresh_target['y1']},{fresh_target['x2']},{fresh_target['y2']}"
                    )

                    # Cache the latest valid ROI for the active scan target.
                    self.tree_roi_cache[scan_target_id] = {
                        "epoch": int(frame_epoch),
                        "bbox": [
                            fresh_target["x1"],
                            fresh_target["y1"],
                            fresh_target["x2"],
                            fresh_target["y2"]
                        ],
                        "tracking_id": fresh_target["tracking_id"],
                        "real_x": fresh_target["real_x"],
                        "real_y": fresh_target["real_y"],
                        "real_z": fresh_target["real_z"],
                        "conf": fresh_target["conf"],
                        "timestamp": time.time()
                    }

                    self.log_diagnostic(
                        f"[SCAN ROI CACHE WRITE]\n"
                        f"target_id={scan_target_id}\n"
                        f"tracking_id={fresh_target['tracking_id']}\n"
                        f"bbox={fresh_target['x1']},{fresh_target['y1']},{fresh_target['x2']},{fresh_target['y2']}\n"
                        f"conf={fresh_target['conf']:.4f}"
                    )

                    self.process_tree(
                        frame,
                        fresh_target["box_obj"],
                        fresh_target["tracking_id"],
                        vote_target_id,
                        marker_array,
                        stamp,
                        fresh_target["real_x"],
                        fresh_target["real_y"],
                        fresh_target["real_z"],
                        fresh_target["conf"],
                        frame_epoch=frame_epoch
                    )

                else:
                    cache = self.tree_roi_cache.get(scan_target_id)

                    # A cached ROI is valid only inside the same immutable
                    # inspection session.  Never reuse an ROI from an older
                    # scan of the same authoritative tree ID.
                    cache_epoch_valid = (
                        cache is not None
                        and cache.get("epoch") == int(frame_epoch)
                    )

                    cache_age = (
                        float("inf")
                        if not cache_epoch_valid
                        else time.time() - cache["timestamp"]
                    )

                    if cache_epoch_valid and cache_age <= self.MAX_SCAN_ROI_AGE:
                        # Multi-view fallback:
                        # cached pixel bbox is used only for ROI dimensions.
                        # The physical target XYZ is reprojected into the CURRENT
                        # RGB frame by make_leaf_crop() using the current timestamp.
                        cached_bbox = cache.get("bbox")
                        cached_xyz = (
                            cache.get("real_x"),
                            cache.get("real_y"),
                            cache.get("real_z")
                        )

                        if cached_bbox is not None:
                            try:
                                class _CachedBox:
                                    def __init__(self, bbox):
                                        self.xyxy = np.asarray(
                                            [bbox], dtype=np.float32
                                        )

                                cached_box = _CachedBox(cached_bbox)
                                cached_tracking_id = cache.get("tracking_id")
                                cached_conf = float(cache.get("conf", 0.0))

                                self.log_diagnostic(
                                    f"[SCAN ROI REPROJECT]\\n"
                                    f"target_id={scan_target_id}\\n"
                                    f"reason=NO_FRESH_TREE_DETECTION\\n"
                                    f"cache_age={cache_age:.2f}s\\n"
                                    f"max_age={self.MAX_SCAN_ROI_AGE:.2f}s\\n"
                                    f"cached_tracking_id={cached_tracking_id}\\n"
                                    f"cached_bbox={cached_bbox}\\n"
                                    f"cached_xyz={cached_xyz}\\n"
                                    f"target_frame=current_rgb_timestamp"
                                )

                                self.process_tree(
                                    frame,
                                    cached_box,
                                    cached_tracking_id,
                                    scan_target_id,
                                    marker_array,
                                    stamp,
                                    cached_xyz[0],
                                    cached_xyz[1],
                                    cached_xyz[2],
                                    cached_conf,
                                    frame_epoch=frame_epoch
                                )

                            except Exception as exc:
                                self.log_diagnostic(
                                    f"[SCAN ROI REPROJECT FAILED]\\n"
                                    f"target_id={scan_target_id}\\n"
                                    f"error={type(exc).__name__}: {exc}"
                                )
                        else:
                            self.log_diagnostic(
                                f"[SCAN ROI REPROJECT SKIP]\\n"
                                f"target_id={scan_target_id}\\n"
                                f"reason=CACHE_MISSING_BBOX"
                            )

                    else:
                        if odom_target_x is None:
                            reason = "TF_UNAVAILABLE_NO_CACHE"
                        elif cache is None:
                            reason = "TARGET_NOT_DETECTED_NO_CACHED_ROI"
                        else:
                            reason = "CACHED_ROI_EXPIRED"

                        self.log_diagnostic(
                            f"[SCAN ROI SKIP]\n"
                            f"target_id={scan_target_id}\n"
                            f"reason={reason}\n"
                            f"cache_age={cache_age:.2f}s\n"
                            f"max_cache_age={self.MAX_SCAN_ROI_AGE:.2f}s"
                        )
                        
        if len(marker_array.markers) > 2: 
            self.marker_pub.publish(marker_array)

    def handle_get_status(self, request, response):
        d_name = "Unknown"
        
        target_matched_id = None
        req_x, req_y = 0.0, 0.0
        min_dist = float('inf')
        target_type = "UNKNOWN"
        
        raw_req = request.tree_id
        
        try:
            # First, check if the request is directly an integer (e.g. "24")
            # which matches what inspection_server currently sends.
            if raw_req.isdigit():
                target_matched_id = int(raw_req)
                target_type = "INTEGER_DIRECT"
            else:
                # Fallback to the old coordinate format "tree_x_y"
                tx_str, ty_str = raw_req.replace("tree_", "").split("_")
                req_x, req_y = float(tx_str), float(ty_str)
                with self.state_lock:
                    tracker_snapshot = [
                        (tid, tracker_data.get('x'), tracker_data.get('y'))
                        for tid, tracker_data in self.tree_tracker.items()
                    ]

                for tid, tx, ty in tracker_snapshot:
                    if tx is None or ty is None:
                        continue
                    dist = np.sqrt((req_x - tx)**2 + (req_y - ty)**2)
                    if dist < min_dist and dist < 3.5:
                        min_dist = dist
                        target_matched_id = tid
                target_type = "COORDINATE_MATCH"
        except Exception as e:
            self.get_logger().error(f"Error parsing tree_id: {e}")
            
        self.get_logger().info(
            f"TARGET MATCH:\n"
            f"raw_target={raw_req}\n"
            f"parsed_tree_id={target_matched_id}\n"
            f"target_type={target_type}"
        )
            
        # --- SYNCHRONIZATION BARRIER ---
        # Stop accepting new scan frames first. Frames already accepted by
        # marker_queue belong to the captured scan session and must finish.
        with self.state_lock:
            self.finalizing_scan = True
            final_epoch = int(self.scan_epoch)

        # Wait until every queued frame has completed processing.
        # image_callback is blocked by finalizing_scan, so no new frame can
        # enter this scan session after the barrier starts.
        self.marker_queue.join()

            
        tree_votes = {}
        # MIN_FINAL_VOTES removed to allow evidence-based decision regardless of scan duration
        
        with self.state_lock:
            final_key = (
                (final_epoch, int(target_matched_id))
                if target_matched_id is not None
                else None
            )

            live_tree_data = (
                self.scan_disease_votes.get(final_key)
                if final_key is not None
                else None
            )

            # Immutable finalization snapshot.
            if live_tree_data is not None:
                tree_data = {
                    'epoch': live_tree_data.get('epoch'),
                    'target_id': live_tree_data.get('target_id'),
                    'counts': dict(live_tree_data.get('counts', {})),
                    'conf_sum': dict(live_tree_data.get('conf_sum', {})),
                    'total_crops': live_tree_data.get('total_crops', 0),
                    'yolo_frames': live_tree_data.get('yolo_frames', 0),
                    'valid_frames': live_tree_data.get('valid_frames', 0),
                    'rejected_frames': live_tree_data.get('rejected_frames', 0),
                    'sum_raw_conf': live_tree_data.get('sum_raw_conf', 0.0),
                    'max_raw_conf': live_tree_data.get('max_raw_conf', 0.0),
                    'max_valid_conf': live_tree_data.get('max_valid_conf', 0.0)
                }

                tree_votes = dict(tree_data['counts'])
                conf_sums = dict(tree_data['conf_sum'])
            else:
                tree_data = None
                tree_votes = {}
                conf_sums = {}

        selected_conf = 0.0
        selected_class = "None"

        if tree_data is not None:
            
            total_valid_votes = sum(tree_votes.values())
            
            reason = "none"
            selected_class = "None"
            selected_conf = 0.0
            
            # ------------------------------------------------------------
            # SCAN-LEVEL FINAL EVIDENCE POLICY
            #
            # Raw Leaf YOLO candidates may be retained at low confidence for
            # diagnostics, but final disease decisions require stronger,
            # repeated frame-level evidence.
            #
            # Policy:
            #   1) Normal confirmation: >= 2 independent frame votes AND
            #      average confidence >= 0.30.
            #   2) A single exceptionally strong frame may independently
            #      confirm a class at >= 0.50.
            #
            # This is class-agnostic and tree-ID-agnostic. It prevents a
            # repeated sequence of marginal ~0.20 detections from becoming
            # a final disease merely because it has the largest vote sum.
            # ------------------------------------------------------------
            FINAL_MIN_AVG_CONF = 0.30
            FINAL_MIN_FRAME_VOTES = 2

            final_candidates = []

            for disease, count in tree_votes.items():
                c_sum = float(conf_sums.get(disease, 0.0))
                avg_c = c_sum / count if count > 0 else 0.0

                if (
                    count >= FINAL_MIN_FRAME_VOTES
                    and avg_c >= FINAL_MIN_AVG_CONF
                ):
                    final_candidates.append(
                        (disease, count, c_sum, avg_c, "multi_frame_confirmation")
                    )
                elif count == 1 and avg_c >= 0.50:
                    final_candidates.append(
                        (disease, count, c_sum, avg_c, "single_strong_frame_confirmation")
                    )

            if final_candidates:
                # Prefer strongest average confidence, then accumulated
                # confidence, then independent frame support.
                top_disease, top_count, top_sum, top_avg, top_reason = max(
                    final_candidates,
                    key=lambda item: (item[3], item[2], item[1])
                )

                d_name = top_disease
                selected_class = top_disease
                selected_conf = top_avg
                reason = top_reason
            else:
                d_name = "Unknown"
                selected_class = "None"
                selected_conf = 0.0
                reason = "insufficient_final_evidence"
                    
            log_str = (
                f"DISEASE FINAL SELECTION:\n"
                f"tree_id={target_matched_id}\n"
                f"total_crops={tree_data.get('total_crops', 0)}\n"
                f"valid_frames={tree_data.get('valid_frames', 0)}\n"
                f"votes={conf_sums}\n"
                f"counts={tree_votes}\n"
                f"max_conf={tree_data.get('max_valid_conf', 0.0):.4f}\n"
                f"selected_class={selected_class}\n"
                f"selected_conf={selected_conf:.4f}\n"
                f"result={d_name}\n"
                f"reason={reason}\n"
            )
            self.get_logger().info(log_str)
        else:
            reason = "missing_target_or_no_votes_data"
            self.get_logger().info(
                f"DISEASE FINAL SELECTION:\n"
                f"tree_id={target_matched_id}\n"
                f"result=Unknown\n"
                f"reason={reason}\n"
            )
            
        severity = self.severity_map.get(d_name, 0.0)
        
        # Build and save the final report
        import os
        debug_dir = "/home/johny/durian_ws/debug_disease"
        os.makedirs(debug_dir, exist_ok=True)
        
        has_votes = False
        # selected_conf was already determined by the final evidence
        # selection above.  Do not overwrite it here.
        total_votes = 0
        final_count = 0
        final_avg_conf = 0.0
        
        if tree_data is not None:
            counts = dict(tree_data.get('counts', {}))
            conf_sums = dict(tree_data.get('conf_sum', {}))
            
            total_crops = tree_data.get('total_crops', 0)
            yolo_frames = tree_data.get('yolo_frames', 0)
            valid_frames = tree_data.get('valid_frames', 0)
            rejected_frames = tree_data.get('rejected_frames', 0)
            sum_raw_conf = tree_data.get('sum_raw_conf', 0.0)
            max_raw_conf = tree_data.get('max_raw_conf', 0.0)
            max_valid_conf = tree_data.get('max_valid_conf', 0.0)
            
            if counts:
                has_votes = True
                total_votes = sum(counts.values())
                final_count = counts.get(d_name, 0)
                final_c_sum = conf_sums.get(d_name, 0.0)
                final_avg_conf = final_c_sum / final_count if final_count > 0 else 0.0

            report = []
            report.append("============================================================")
            report.append("TREE DISEASE SCAN RESULT")
            report.append("============================================================")
            report.append(f"\nTree ID: {target_matched_id}\n")
            report.append(f"Requested tree coordinates: {req_x}, {req_y}")
            report.append(f"Authoritative tree ID: {target_matched_id}")
            report.append(f"Tracker distance: {min_dist:.6f}\n")
            
            report.append(f"Total crops: {total_crops}")
            report.append(f"YOLO crops with leaf detections: {yolo_frames}")
            report.append(f"Valid frames: {valid_frames}")
            report.append(f"Rejected frames: {rejected_frames}\n")
            
            report.append(f"Raw confidence sum: {sum_raw_conf:.6f}")
            report.append(f"Maximum raw confidence: {max_raw_conf:.6f}")
            report.append(f"Maximum valid confidence: {max_valid_conf:.6f}\n")
            
            report.append(f"Has disease votes: {'TRUE' if has_votes else 'FALSE'}\n")
            report.append(f"Total valid frame votes: {total_votes}\n")
            report.append("Disease statistics:\n")
            
            all_diseases = ["Leaf_Blight", "Leaf_Healthy", "Leaf_Algal", "Leaf_Colletotrichum", "Leaf_Phomopsis", "Leaf_Rhizoctonia"]
            for disease in all_diseases:
                count = counts.get(disease, 0)
                c_sum = conf_sums.get(disease, 0.0)
                avg_conf = c_sum / count if count > 0 else 0.0
                report.append(f"{disease}")
                report.append(f"Vote count: {count}")
                report.append(f"Confidence sum: {c_sum:.6f}")
                report.append(f"Average confidence: {avg_conf:.6f}\n")
                
            report.append("---")
            report.append(f"\nFINAL DISEASE: {d_name}")
            report.append(f"FINAL VOTE COUNT: {final_count}")
            report.append(f"FINAL AVERAGE CONFIDENCE: {final_avg_conf:.6f}\n")
            report.append("============================================================\n")
            
            report_str = "\n".join(report)
            
            debug_file = os.path.join(debug_dir, f"tree_{target_matched_id}_scan.txt")
            try:
                with open(debug_file, "w") as f:
                    f.write(report_str)
            except Exception as e:
                self.get_logger().error(f"Failed to write debug file: {e}")
                
            self.get_logger().info(
                f"[DISEASE SCAN COMPLETE] Tree {target_matched_id}\n"
                f"Final disease: {d_name}\n"
                f"Votes: {final_count} / {total_votes}\n"
                f"Average confidence: {final_avg_conf:.6f}\n"
                f"Debug file: {debug_file}"
            )
        else:
            self.get_logger().info(f"[DISEASE SCAN COMPLETE] Tree {target_matched_id} - No votes recorded. Final disease: {d_name}")
            
        # Keep the completed scan closed.
        # A new scan_active=True callback starts the next epoch
        # and explicitly clears finalizing_scan.
        response.durian_type = "Musang King (D197)"
        response.disease_status = d_name
        response.coverage = float(severity * 100.0)
        response.confidence = float(selected_conf * 100.0)
        
        remedy = "None"
        priority = "None"
        if d_name == "Leaf_Blight":
            remedy = "Difenoconazole"
            priority = "High"
        elif d_name == "Leaf_Algal":
            remedy = "Copper-based"
            priority = "Medium"
        elif d_name == "Leaf_Colletotrichum":
            remedy = "Azoxystrobin"
            priority = "Medium"
        elif d_name == "Leaf_Phomopsis":
            remedy = "Carbendazim"
            priority = "Low"
        elif d_name == "Leaf_Rhizoctonia":
            remedy = "Jinggangmycin"
            priority = "High"
            
        response.remedy = remedy
        response.priority = priority
        
        self.get_logger().info(f"DATABASE:\nstored disease={d_name}")
        
        if target_matched_id is not None:
            with self.state_lock:
                self.scan_disease_votes.pop(
                    (final_epoch, int(target_matched_id)),
                    None
                )
        return response

    def detect_durian_type(self, crop):
        if crop.size == 0 or self.model_fruit is None: return "Unknown"
        result = self.model_fruit(crop, verbose=False, conf=0.20)
        if len(result[0].boxes) == 0: return "Unknown"
        class_id = int(result[0].boxes.cls.cpu().numpy()[0])
        return self.durian_type.get(class_id, "Unknown")

    def make_leaf_crop(self, frame, bbox, real_x=None, real_y=None, real_z=None, stamp=None):
        height, width = frame.shape[:2]
        x1, y1, x2, y2 = map(int, bbox)

        x1 = max(0, min(width - 1, x1))
        y1 = max(0, min(height - 1, y1))
        x2 = max(x1 + 1, min(width, x2))
        y2 = max(y1 + 1, min(height, y2))

        box_w = max(1, x2 - x1)
        box_h = max(1, y2 - y1)
        aspect = box_w / float(box_h)
        bbox_area_ratio = (box_w * box_h) / float(width * height)
        border_touch_count = sum([
            x1 <= 0,
            y1 <= 0,
            x2 >= width,
            y2 >= height,
        ])
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        # Generalized Tree-bbox reliability assessment.
        # This is intentionally independent of tree ID, tracking ID,
        # scene, fixed coordinates, or disease class.
        is_unreliable = (
            box_w < 80 or
            box_h < 80 or
            aspect < 0.15 or
            aspect > 7.0 or
            bbox_area_ratio > 0.50 or
            (border_touch_count >= 2 and (aspect < 0.30 or aspect > 3.50))
        )

        fallback_used = False
        fallback_reason = "none"
        projected = False

        # Use the active physical target position whenever metric data is available.
        # Tree bbox reliability must not disable physical target projection.
        projection_eligible = (
            real_x is not None
            and real_y is not None
            and real_z is not None
            and float(real_z) > 0.05
            and hasattr(self, 'tf_buffer')
            and hasattr(self, 'camera_info_msg')
            and stamp is not None
        )

        if not is_unreliable and not projection_eligible:
            self.get_logger().info(
                f"[PROJECTION DIAG] skipped: "
                f"real_xyz=({real_x},{real_y},{real_z}) "
                f"z_valid={real_z is not None and float(real_z) > 0.05 if real_z is not None else False} "
                f"tf={hasattr(self, 'tf_buffer')} "
                f"camera_info={hasattr(self, 'camera_info_msg')} "
                f"stamp={stamp is not None}"
            )

        if projection_eligible:
            try:
                import tf2_geometry_msgs
                from geometry_msgs.msg import PointStamped
                import rclpy

                # Project the odom point using the TF corresponding to
                # the RGB frame timestamp. Never fall back to latest TF,
                # because the robot may have moved between frames.
                trans = self.tf_buffer.lookup_transform(
                    "camera_link_optical", "odom",
                    stamp
                )

                pt = PointStamped()
                pt.header.frame_id = "odom"
                pt.header.stamp = stamp
                pt.point.x = float(real_x)
                pt.point.y = float(real_y)
                pt.point.z = float(real_z)

                cam_pt = tf2_geometry_msgs.do_transform_point(pt, trans)

                if cam_pt.point.z <= 0.05:
                    self.get_logger().info(
                        f"[PROJECTION DIAG] INVALID_CAMERA_Z "
                        f"cam_z={cam_pt.point.z:.6f}"
                    )
                elif cam_pt.point.z > 0.05:
                    fx = float(self.camera_info_msg.k[0])
                    fy = float(self.camera_info_msg.k[4])
                    cx_cam = float(self.camera_info_msg.k[2])
                    cy_cam = float(self.camera_info_msg.k[5])

                    proj_x = int(cam_pt.point.x * fx / cam_pt.point.z + cx_cam)
                    proj_y = int(cam_pt.point.y * fy / cam_pt.point.z + cy_cam)

                    if not (0 <= proj_x < width and 0 <= proj_y < height):
                        self.get_logger().info(
                            f"[PROJECTION DIAG] OUTSIDE_IMAGE "
                            f"cam_xyz=({cam_pt.point.x:.3f},{cam_pt.point.y:.3f},{cam_pt.point.z:.3f}) "
                            f"proj=({proj_x},{proj_y}) image={width}x{height}"
                        )

                    if 0 <= proj_x < width and 0 <= proj_y < height:
                        # The Tree YOLO bbox dimensions are already in image pixels.
                        # Do NOT re-project bbox size with fx / depth.
                        # 3D projection is used only to obtain the physical target center.
                        context_scale = (
                            float(self.get_parameter('leaf_roi_unreliable_context_scale').value)
                            if is_unreliable
                            else float(self.get_parameter('leaf_roi_context_scale').value)
                        )
                        context_scale = max(1.0, context_scale)

                        min_roi_size = max(
                            1,
                            int(self.get_parameter('leaf_roi_min_size').value)
                        )

                        max_area_ratio = float(
                            self.get_parameter('leaf_roi_max_area_ratio').value
                        )
                        max_area_ratio = min(0.99, max(0.01, max_area_ratio))

                        target_w = int(max(box_w, 1) * context_scale)
                        target_h = int(max(box_h, 1) * context_scale)

                        # Keep the existing minimum-ROI policy consistent.
                        target_w = max(box_w, target_w, min_roi_size)
                        target_h = max(box_h, target_h, min_roi_size)

                        # Apply the same area bound to the 3D-projection path
                        # that is already used by the 2D fallback.
                        max_area = float(width * height) * max_area_ratio
                        requested_area = float(target_w) * float(target_h)

                        if requested_area > max_area:
                            shrink = (max_area / requested_area) ** 0.5
                            target_w = int(max(float(box_w), target_w * shrink))
                            target_h = int(max(float(box_h), target_h * shrink))

                        target_w = min(width, int(target_w))
                        target_h = min(height, int(target_h))

                        self.get_logger().info(
                            f"[ROI SAFETY] "
                            f"bbox_area_ratio={bbox_area_ratio:.4f} "
                            f"border_touch_count={border_touch_count} "
                            f"max_area_ratio={max_area_ratio:.4f} "
                            f"final_target={target_w}x{target_h}"
                        )
                        self.get_logger().info(
                            f"[PROJECTION FORENSIC] "
                            f"tree_bbox=({x1},{y1},{x2},{y2}) "
                            f"tree_size={box_w}x{box_h} "
                            f"real_xyz=({float(real_x):.3f},{float(real_y):.3f},{float(real_z):.3f}) "
                            f"cam_xyz=({cam_pt.point.x:.3f},{cam_pt.point.y:.3f},{cam_pt.point.z:.3f}) "
                            f"proj=({proj_x},{proj_y}) "
                            f"context_scale={context_scale:.2f} "
                            f"target_size={target_w}x{target_h} "
                            f"image={width}x{height}"
                        )

                        c_x1 = int(proj_x - target_w / 2.0)
                        c_y1 = int(proj_y - target_h / 2.0)
                        c_x2 = c_x1 + target_w
                        c_y2 = c_y1 + target_h

                        # Translate the ROI inward when it touches an image boundary.
                        if c_x1 < 0:
                            c_x1, c_x2 = 0, target_w
                        elif c_x2 > width:
                            c_x2 = width
                            c_x1 = width - target_w

                        if c_y1 < 0:
                            c_y1, c_y2 = 0, target_h
                        elif c_y2 > height:
                            c_y2 = height
                            c_y1 = height - target_h

                        c_x1, c_y1 = int(c_x1), int(c_y1)
                        c_x2, c_y2 = int(c_x2), int(c_y2)

                        projected = True
                        fallback_reason = "3d_tracking_projection"

            except Exception:
                fallback_reason = "tf_projection_failed"

        # Generic 2D fallback: derive a bounded ROI directly from the Tree bbox.
        # No fixed distance, tree ID, scene, or bbox-specific exception is used.
        if not projected:
            context_scale = (
                float(self.get_parameter('leaf_roi_unreliable_context_scale').value)
                if is_unreliable
                else float(self.get_parameter('leaf_roi_context_scale').value)
            )
            max_area_ratio = float(
                self.get_parameter('leaf_roi_max_area_ratio').value
            )

            context_scale = max(1.0, context_scale)
            max_area_ratio = min(0.99, max(0.01, max_area_ratio))

            # Start from the requested contextual ROI.
            target_w = max(float(box_w), float(box_w) * context_scale)
            target_h = max(float(box_h), float(box_h) * context_scale)

            # If the requested ROI is too large, shrink it uniformly while
            # keeping the Tree bbox fully contained.
            max_area = float(width * height) * max_area_ratio
            requested_area = target_w * target_h

            if requested_area > max_area:
                shrink = (max_area / requested_area) ** 0.5
                target_w *= shrink
                target_h *= shrink

            # The ROI must still contain the complete Tree bbox.
            target_w = max(target_w, float(box_w))
            target_h = max(target_h, float(box_h))

            cx_f = (x1 + x2) / 2.0
            cy_f = (y1 + y2) / 2.0

            c_x1 = int(round(cx_f - target_w / 2.0))
            c_y1 = int(round(cy_f - target_h / 2.0))
            c_x2 = c_x1 + int(round(target_w))
            c_y2 = c_y1 + int(round(target_h))

            # Shift the ROI back inside the image without changing its size
            # whenever possible.
            roi_w = c_x2 - c_x1
            roi_h = c_y2 - c_y1

            if c_x1 < 0:
                c_x2 -= c_x1
                c_x1 = 0
            if c_y1 < 0:
                c_y2 -= c_y1
                c_y1 = 0
            if c_x2 > width:
                shift = c_x2 - width
                c_x1 -= shift
                c_x2 = width
            if c_y2 > height:
                shift = c_y2 - height
                c_y1 -= shift
                c_y2 = height

            # Final clipping; the original bbox remains the reference target.
            c_x1 = max(0, int(c_x1))
            c_y1 = max(0, int(c_y1))
            c_x2 = min(width, int(c_x2))
            c_y2 = min(height, int(c_y2))

            fallback_used = True
            if fallback_reason == "none":
                fallback_reason = "2d_bbox_geometry"

        crop_w = c_x2 - c_x1
        crop_h = c_y2 - c_y1
        area_ratio = (crop_w * crop_h) / float(width * height)

        self.log_diagnostic(
            f"[CROP GEOMETRY]\n"
            f"tree_bbox={x1},{y1},{x2},{y2}\n"
            f"tree_size={box_w}x{box_h}\n"
            f"bbox_area_ratio={bbox_area_ratio:.4f}\n"
            f"border_touch_count={border_touch_count}\n"
            f"unreliable={is_unreliable}\n"
            f"fallback={fallback_used}\n"
            f"fallback_reason={fallback_reason}\n"
            f"crop_bbox={c_x1},{c_y1},{c_x2},{c_y2}\n"
            f"crop_size={crop_w}x{crop_h}\n"
            f"area_ratio={area_ratio:.4f}"
        )

        min_roi_size = max(
            1,
            int(self.get_parameter('leaf_roi_min_size').value)
        )

        if crop_w < min_roi_size or crop_h < min_roi_size:
            self.log_diagnostic(
                f"[CROP RESULT]\n"
                f"result=INVALID\n"
                f"reason=ROI_TOO_SMALL\n"
                f"crop_size={crop_w}x{crop_h}\n"
                f"min_roi_size={min_roi_size}"
            )
            return None, None

        max_area_ratio = float(
            self.get_parameter('leaf_roi_max_area_ratio').value
        )
        if area_ratio > max_area_ratio:
            self.log_diagnostic(
                f"[CROP RESULT]\n"
                f"result=INVALID\n"
                f"reason=ROI_TOO_LARGE\n"
                f"area_ratio={area_ratio:.4f}\n"
                f"max_area_ratio={max_area_ratio:.4f}"
            )
            return None, None

        crop = frame[c_y1:c_y2, c_x1:c_x2]

        if crop.size == 0:
            self.log_diagnostic("[CROP RESULT]\nresult=INVALID\nreason=EMPTY")
            return None, None

        crop_gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        variance = np.var(crop_gray)

        sobelx = cv2.Sobel(crop_gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(crop_gray, cv2.CV_64F, 0, 1, ksize=3)
        magnitude = cv2.magnitude(
            sobelx.astype(np.float32),
            sobely.astype(np.float32)
        )
        edge_density = np.mean(magnitude > 30)

        b_channel = crop[:, :, 0]
        g_channel = crop[:, :, 1]
        r_channel = crop[:, :, 2]

        blue_ratio = np.mean(
            (b_channel > r_channel + 20) &
            (b_channel > g_channel + 10)
        )

        green_ratio = np.mean(
            (g_channel > r_channel) &
            (g_channel > b_channel)
        )

        self.log_diagnostic(
            f"[CROP VALIDATION]\n"
            f"variance={variance:.2f}\n"
            f"edge_density={edge_density:.4f}\n"
            f"blue_ratio={blue_ratio:.4f}\n"
            f"green_ratio={green_ratio:.4f}"
        )

        if blue_ratio > 0.95 and green_ratio < 0.02:
            self.log_diagnostic("[CROP RESULT]\nresult=INVALID\nreason=SKY_DOMINANT")
            return None, None

        if variance < 50.0 or edge_density < 0.01:
            self.log_diagnostic("[CROP RESULT]\nresult=INVALID\nreason=LOW_INFORMATION")
            return None, None

        self.log_diagnostic(
            f"[CROP RESULT]\nresult=VALID\nsize={crop_w}x{crop_h}"
        )

        return crop, (c_x1, c_y1, c_x2, c_y2)

    def process_tree(self, frame, box, tracking_id, vote_target_id, marker_array, stamp, real_x, real_y, real_z, tree_conf, frame_epoch=None):
        self.debug_frame_counter += 1
        debug_id = self.debug_frame_counter
        
        height, width = frame.shape[:2]
        xyxy = box.xyxy[0].tolist() if hasattr(box.xyxy[0], 'tolist') else box.xyxy[0]
        x1, y1, x2, y2 = map(int, xyxy)
        
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "odom"
        pose_msg.header.stamp = stamp
        # Invalid depth is allowed for disease ROI processing, but a ROS
        # 3D pose/marker must never be created from None coordinates.
        if real_x is not None and real_y is not None and real_z is not None:
            pose_msg.pose.position.x = float(real_x)
            pose_msg.pose.position.y = float(real_y)
            pose_msg.pose.position.z = float(real_z)
        else:
            self.log_diagnostic(
                f"[TREE 2D ONLY] tracking_id={tracking_id} "
                f"3D marker skipped; disease ROI remains available"
            )
        if real_x is not None and real_y is not None and real_z is not None:
            pose_msg.pose.orientation.w = 1.0
            self.tree_pose_pub.publish(pose_msg)
        
        durian_type_id = "Unknown"
        d_name = "Unknown"
        severity = 0.0
        reason = "NO_CROP"
        crop_bbox_str = "None"
        crop_size_str = "None"
        leaf_detections = 0
        leaf_conf = 0.0
        
        if self.model_leaf:
            crop, crop_bbox = self.make_leaf_crop(frame, (x1, y1, x2, y2), real_x, real_y, real_z, stamp)
            
            if crop is None:
                reason = "INVALID_CROP"
            else:
                c_x1, c_y1, c_x2, c_y2 = crop_bbox
                crop_bbox_str = f"{c_x1},{c_y1},{c_x2},{c_y2}"
                crop_size_str = f"{crop.shape[1]}x{crop.shape[0]}"
                
                import cv2
                saved = cv2.imwrite(f"/home/johny/durian_ws/debug_tree_{tracking_id}_{debug_id}.jpg", crop)
                if not saved:
                    self.get_logger().error("ERROR: failed to save debug ROI")
                else:
                    self.log_diagnostic(f"ROI SAVED:\ntracking_id={tracking_id}\ndebug_id={debug_id}\nbbox={x1},{y1},{x2},{y2}\ncrop_bbox={crop_bbox_str}\ncrop_size={crop_size_str}")
                    
                durian_type_id = self.detect_durian_type(crop)
                d_name, severity, reason, leaf_detections, leaf_conf = self.detect_leaf_disease(
                    crop, tracking_id, vote_target_id, debug_id, tree_conf, frame_epoch
                )
                
        type_info = self.tree_config.get(durian_type_id, {'name': 'Unknown'})
        type_name = type_info.get('name', 'Unknown')
        
        with self.state_lock:
            old_tree = self.tree_tracker.get(tracking_id)

        if old_tree is not None:
            if (
                real_x is not None
                and real_y is not None
                and old_tree.get('x') is not None
                and old_tree.get('y') is not None
            ):
                real_x = 0.7 * old_tree['x'] + 0.3 * real_x
                real_y = 0.7 * old_tree['y'] + 0.3 * real_y
            
        disease_info = self.tree_config.get(d_name, {'name': 'Unknown', 'remedy': 'None'})
        
        if marker_array is not None:
            if real_x is not None and real_y is not None and real_z is not None:
                marker_array.markers.append(
                    self.create_tree_marker(
                        tracking_id,
                        float(real_x),
                        float(real_y),
                        severity,
                        stamp
                    )
                )
                marker_array.markers.append(
                    self.create_text_marker(
                        tracking_id,
                        float(real_x),
                        float(real_y),
                        type_name,
                        disease_info,
                        stamp
                    )
                )
            else:
                self.log_diagnostic(
                    f"[TREE 2D ONLY] tracking_id={tracking_id} "
                    f"3D markers/text skipped; disease ROI remains available"
                )
        
        self.log_diagnostic(
            f"================ VISION PIPELINE ================\n\n"
            f"frame/debug_id: {debug_id}\n"
            f"tracking_id: {tracking_id}\n"
            f"vote_target_id: {vote_target_id}\n\n"
            f"TREE YOLO:\n"
            f"confidence: {tree_conf:.4f}\n"
            f"bbox: {x1},{y1},{x2},{y2}\n\n"
            f"TREE TRACKING:\n"
            f"tracking_id: {tracking_id}\n\n"
            f"DEPTH:\n"
            f"depth: {0.0}\n"
            f"valid: True\n\n"
            f"ROI:\n"
            f"crop_bbox: {crop_bbox_str}\n"
            f"crop_size: {crop_size_str}\n\n"
            f"FRUIT YOLO:\n"
            f"result: {durian_type_id}\n\n"
            f"LEAF YOLO:\n"
            f"detections: {leaf_detections}\n\n"
            f"LEAF RESULT:\n"
            f"class: {d_name}\n"
            f"confidence: {leaf_conf:.4f}\n"
            f"reason: {reason}\n\n"
            f"=================================================="
        )

    def calibrate_colors(self):
        self.get_logger().info("Using Pure YOLO for disease detection in all environments.")

    def apply_hsv_clahe(self, img, sat_factor, use_clahe):
        import cv2
        import numpy as np
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.float32)
        hsv[:, :, 1] = hsv[:, :, 1] * sat_factor
        hsv[:, :, 1] = np.clip(hsv[:, :, 1], 0, 255)
        hsv = hsv.astype(np.uint8)
        
        if use_clahe:
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            hsv[:, :, 2] = clahe.apply(hsv[:, :, 2])
            
        enhanced = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        return enhanced

    def _get_vegetation_mask(self, img):
        import cv2
        import numpy as np
        # HSV and LAB used as general engineering thresholds for green vegetation candidate isolation
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_hsv = cv2.inRange(hsv, np.array([30, 40, 40]), np.array([105, 255, 255]))
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        mask_lab = cv2.inRange(lab, np.array([0, 0, 0]), np.array([255, 120, 255]))
        return cv2.bitwise_and(mask_hsv, mask_lab)

    def _extract_leaf_rois(self, mask, img):
        import cv2
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for c in contours:
            area = cv2.contourArea(c)
            # Permissive base filtering
            if area <= 50:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w < 10 or h < 10:
                continue
            aspect = w / float(h)
            if aspect < 0.1 or aspect > 10.0:
                continue
            
            # Very permissive solidity to avoid killing good candidates
            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            if solidity < 0.40:
                continue

            candidates.append({
                'contour': c,
                'x': x, 'y': y, 'w': w, 'h': h
            })
        return candidates

    def _pad_to_square(self, crop):
        import numpy as np
        import cv2
        h, w = crop.shape[:2]
        size = max(h, w)
        if size == 0:
            return crop, 0, 0
        y_off = (size - h) // 2
        x_off = (size - w) // 2
        bottom_off = size - h - y_off
        right_off = size - w - x_off
        
        border_pixels = np.concatenate([crop[0, :], crop[-1, :], crop[:, 0], crop[:, -1]])
        if border_pixels.size > 0:
            bg_color = np.median(border_pixels, axis=0).astype(np.uint8).tolist()
        else:
            bg_color = [128, 128, 128]
            
        padded = cv2.copyMakeBorder(crop, y_off, bottom_off, x_off, right_off, cv2.BORDER_CONSTANT, value=bg_color)
        return padded, x_off, y_off

    def _get_scan_evidence(self, epoch, target_id, create=True):
        """Return evidence for one immutable scan session/target pair.

        All callers must hold state_lock while using the returned object.
        The epoch prevents frames from a previous inspection session from
        writing into a later session with the same authoritative tree ID.
        """
        if target_id is None or epoch is None:
            return None

        key = (int(epoch), int(target_id))
        evidence = self.scan_disease_votes.get(key)

        if evidence is None and create:
            evidence = {
                'epoch': int(epoch),
                'target_id': int(target_id),
                'counts': {},
                'conf_sum': {},
                'total_crops': 0,
                'yolo_frames': 0,
                'valid_frames': 0,
                'rejected_frames': 0,
                'sum_raw_conf': 0.0,
                'max_raw_conf': 0.0,
                'max_valid_conf': 0.0
            }
            self.scan_disease_votes[key] = evidence

        return evidence

    def detect_leaf_disease(self, crop, tracking_id, vote_target_id, debug_id, tree_conf, frame_epoch=None):
        d_name = "Unknown"
        severity = 0.0
        reason = "NO_LEAF_DETECTION"
        frame_votes = {}
        # Raw YOLO candidate threshold: retain weak detections for diagnostics.
        MIN_LEAF_CONF = 0.01

        # Formal evidence threshold: only meaningful detections enter
        # NMS and temporal disease voting.
        LEAF_EVIDENCE_CONF = 0.20

        NMS_IOU = 0.40
        
        leaf_detections = 0
        leaf_conf = 0.0
        
        # P1 Fix: Make disease evidence session-scoped
        # Only accumulate votes if this is the active scan target and scanning is active
        if vote_target_id is None:
            return "Unknown", 0.0, "NOT_ACTIVE_SCAN_TARGET", 0, 0.0
            
        matched_id = vote_target_id

        # The frame session must travel explicitly from frame_context.
        # This prevents delayed ROS callbacks from being assigned to
        # the current scan epoch after the frame was originally captured.
        if frame_epoch is None:
            with self.state_lock:
                frame_epoch = self.scan_epoch

        with self.state_lock:
            evidence = self._get_scan_evidence(
                frame_epoch,
                matched_id,
                create=True
            )

            if evidence is None:
                return "Unknown", 0.0, "INVALID_SCAN_SESSION", 0, 0.0

            evidence['total_crops'] += 1

        if crop.size > 0 and self.model_leaf:
            
            # --- DIAGNOSTIC STATE INIT ---
            if not hasattr(self, 'leaf_diag_stats'):
                self.leaf_diag_stats = {}
            if matched_id not in self.leaf_diag_stats:
                self.leaf_diag_stats[matched_id] = {
                    'total_candidates': 0, 'geometry_accepted': 0, 'geometry_rejected': 0,
                    'yolo_evaluated': 0, 'no_detection': 0, 'raw_low_conf': 0,
                    'geometry_rejected_roi': 0, 'success': 0, 'mixed': 0,
                    'confs': [], 'widths': [], 'heights': [],
                    'reason_BORDER_TOUCH': 0, 'reason_AREA_RATIO': 0,
                    'reason_ABSOLUTE_SIZE': 0, 'reason_ASPECT_RATIO': 0,
                    'valid_confidence_count': 0, 'nms_count': 0
                }
            
            import os
            import cv2
            import shutil
            diag_base = "/tmp/durian_roi_diagnostic"
            os.makedirs(os.path.join(diag_base, "success"), exist_ok=True)
            os.makedirs(os.path.join(diag_base, "low_conf"), exist_ok=True)
            os.makedirs(os.path.join(diag_base, "no_detection"), exist_ok=True)
            os.makedirs(os.path.join(diag_base, "geometry_rejected"), exist_ok=True)
            os.makedirs(os.path.join(diag_base, "mixed"), exist_ok=True)

            
            height, width = crop.shape[:2]
            
            all_raw_boxes = []
            tile_id = 0
            
            raw_yolo_count = 0
            reject_border = 0
            reject_area = 0
            reject_size = 0
            reject_aspect = 0
            accepted_geometry = 0
            
            import cv2
            import numpy as np
            import os
            
            # --- FULL ROI 1024 INFERENCE ---
            prod_debug_dir = "/tmp/durian_roi_production_debug"
            os.makedirs(prod_debug_dir, exist_ok=True)
            
            cand_idx = 0
            if True:
                if True:
                    tile_crop = crop
                    
                    if tile_crop.size > 0:
                        self.leaf_diag_stats[matched_id]['yolo_evaluated'] += 1
                        
                        # Run YOLO using configurable inference scales.
                        #
                        # The scale list is a ROS parameter, so the
                        # perception pipeline contains no tree-specific,
                        # bbox-specific, or scene-specific scale logic.
                        leaf_inference_scales = self.get_parameter(
                            'leaf_inference_scales'
                        ).value

                        # Normalize and validate the configured scales.
                        try:
                            leaf_inference_scales = [
                                int(v) for v in leaf_inference_scales
                                if int(v) > 0
                            ]
                        except Exception:
                            leaf_inference_scales = []

                        # Remove duplicate sizes while preserving order.
                        leaf_inference_scales = list(dict.fromkeys(
                            leaf_inference_scales
                        ))

                        leaf_scale_results = []

                        if not leaf_inference_scales:
                            self.log_diagnostic(
                                "[LEAF MULTISCALE] "
                                "no configured inference scales; "
                                "leaf inference skipped"
                            )
                        else:
                            for leaf_imgsz in leaf_inference_scales:
                                res_try = self.model_leaf(
                                    tile_crop,
                                    verbose=False,
                                    conf=MIN_LEAF_CONF,
                                    imgsz=leaf_imgsz
                                )[0]

                                raw_count_try = len(res_try.boxes)
                                leaf_scale_results.append(
                                    (leaf_imgsz, res_try)
                                )

                                self.log_diagnostic(
                                    f"[LEAF MULTISCALE] "
                                    f"scale={leaf_imgsz} "
                                    f"raw_boxes={raw_count_try}"
                                )

                                # Evaluate every configured inference scale.
                                # The configured scales are all part of the
                                # same model-agnostic leaf inference pipeline.

                        # TRUE MULTI-SCALE MERGE
                        #
                        # Every configured inference scale has already been
                        # evaluated above. Retain ALL detections here.
                        # Do not select one scale by maximum confidence,
                        # because small objects may only be detected at
                        # another configured scale.

                        leaf_detection_records = []
                        total_multiscale_raw = 0

                        for _scale, _result in leaf_scale_results:
                            for _box in _result.boxes:
                                leaf_detection_records.append(
                                    (_scale, _result, _box)
                                )
                                total_multiscale_raw += 1

                        if leaf_scale_results:
                            # Diagnostic/compatibility field only.
                            # This value no longer controls detection selection.
                            selected_leaf_scale = leaf_scale_results[-1][0]
                            res_v26 = leaf_scale_results[-1][1]

                            self.log_diagnostic(
                                f"[LEAF MULTISCALE RESULT] "
                                f"mode=MERGED "
                                f"scales={[x[0] for x in leaf_scale_results]} "
                                f"total_raw_boxes={total_multiscale_raw}"
                            )
                        else:
                            res_v26 = None
                            selected_leaf_scale = None

                        roi_diag = {
                            'raw_box_count': 0, 'raw_boxes_conf_ge_0_20': 0, 'raw_boxes_conf_lt_0_20': 0,
                            'geometry_pass_count': 0, 'geometry_reject_count': 0
                        }
                        
                        tile_w = width
                        tile_h = height
                        tile_id = 0
                        x1, y1 = 0, 0
                        
                        for leaf_imgsz, leaf_result, box in leaf_detection_records:
                            raw_yolo_count += 1
                            cand_idx = raw_yolo_count - 1
                            roi_diag['raw_box_count'] += 1
                            
                            conf = float(box.conf[0].item())
                            cls_idx = int(box.cls[0].item())
                            raw_name = leaf_result.names[cls_idx]
                            
                            self.leaf_diag_stats[matched_id]['confs'].append(conf)
                            if conf >= 0.20:
                                roi_diag['raw_boxes_conf_ge_0_20'] += 1
                            else:
                                roi_diag['raw_boxes_conf_lt_0_20'] += 1
                            
                            # Coordinates relative to crop
                            px1, py1, px2, py2 = box.xyxy[0].tolist()
        
                            self.log_diagnostic(
                                f"[LEAF YOLO RAW XYXY]\n"
                                f"roi=FULL\n"
                                f"imgsz={leaf_imgsz}\n"
                                f"px1={px1:.2f}\n"
                                f"py1={py1:.2f}\n"
                                f"px2={px2:.2f}\n"
                                f"py2={py2:.2f}\n"
                                f"roi_w={tile_w:.2f}\n"
                                f"roi_h={tile_h:.2f}\n"
                                f"class={cls_idx}\n"
                                f"conf={conf:.4f}"
                            )
                            
                            # Coordinates are natively global crop coords
                            gx1, gy1 = px1, py1
                            gx2, gy2 = px2, py2
                            bx1, by1 = px1, py1
                            bx2, by2 = px2, py2
                            
                            local_w = px2 - px1
                            local_h = py2 - py1
                            local_area = local_w * local_h
                            
                            # Debug image limit
                            if conf >= 0.20 and cand_idx < 3: 
                                roi_debug_path = os.path.join(prod_debug_dir, f"tree{matched_id}_full_{raw_name}_{conf:.2f}.png")
                                cv2.imwrite(roi_debug_path, tile_crop)
                            
                            touch_left_crop = bx1 < 5
                            touch_right_crop = bx2 > width - 5
                            touch_top_crop = by1 < 5
                            touch_bottom_crop = by2 > height - 5
                            is_border_touch_crop = (touch_left_crop + touch_right_crop + touch_top_crop + touch_bottom_crop >= 3)
                            
                            self.log_diagnostic(
                                f"[LEAF RAW DETECT]\n"
                                f"roi=FULL\ncandidate={cand_idx}\nclass={cls_idx}\nname={raw_name}\n"
                                f"conf={conf:.4f}\nbbox={gx1:.1f},{gy1:.1f},{gx2:.1f},{gy2:.1f}\n"
                                f"local_width={local_w:.1f}\nlocal_height={local_h:.1f}\narea_ratio={local_area/max(1.0, tile_w*tile_h):.3f}\n"
                                f"border_touch={is_border_touch_crop}"
                            )
                            
                            # Filter 1. Reject boxes touching full crop borders on multiple sides
                            if is_border_touch_crop:
                                reject_border += 1
                                self.leaf_diag_stats[matched_id]['reason_BORDER_TOUCH'] += 1
                                self.log_diagnostic(f"[LEAF GEOMETRY REJECT]\nroi=FULL\nclass={cls_idx}\nreason=BORDER_TOUCH")
                                roi_diag['geometry_reject_count'] += 1
                                continue
                                
                            roi_area = float(max(1, tile_w * tile_h))
                            roi_max_dim = float(max(tile_w, tile_h))
                            
                            area_ratio = local_area / roi_area
                            width_ratio = local_w / float(tile_w)
                            height_ratio = local_h / float(tile_h)
                            
                            # Dynamic limits: smaller ROIs can be filled more densely by a single leaf.
                            # Base scale interpolation from 100px to 640px.
                            scale_factor = max(0.0, min(1.0, (roi_max_dim - 100.0) / 540.0))
                            
                            dynamic_area_limit = 0.85 - (0.40 * scale_factor) # Clamped: 0.85 (small) to 0.45 (large)
                            dynamic_size_limit = 0.95 - (0.15 * scale_factor) # Clamped: 0.95 (small) to 0.80 (large)
                            
                            self.log_diagnostic(
                                f"[LEAF DYNAMIC GEOMETRY]\n"
                                f"area_ratio={area_ratio:.3f}\n"
                                f"dynamic_area_limit={dynamic_area_limit:.3f}\n"
                                f"width_ratio={width_ratio:.3f}\n"
                                f"height_ratio={height_ratio:.3f}\n"
                                f"dynamic_size_limit={dynamic_size_limit:.3f}"
                            )

                            # Filter 2. AREA RATIO
                            if area_ratio > dynamic_area_limit:
                                reject_area += 1
                                self.leaf_diag_stats[matched_id]['reason_AREA_RATIO'] += 1
                                self.log_diagnostic(f"[LEAF GEOMETRY REJECT]\nroi=FULL\nclass={cls_idx}\nreason=AREA_RATIO")
                                roi_diag['geometry_reject_count'] += 1
                                continue
                                
                            # Filter 3. ABSOLUTE SIZE
                            if width_ratio > dynamic_size_limit or height_ratio > dynamic_size_limit:
                                reject_size += 1
                                self.leaf_diag_stats[matched_id]['reason_ABSOLUTE_SIZE'] += 1
                                self.log_diagnostic(f"[LEAF GEOMETRY REJECT]\nroi=FULL\nclass={cls_idx}\nreason=ABSOLUTE_SIZE")
                                roi_diag['geometry_reject_count'] += 1
                                continue
                                
                            aspect = local_w / max(1.0, local_h)
                            if aspect < 0.15 or aspect > 7.0:
                                reject_aspect += 1
                                self.leaf_diag_stats[matched_id]['reason_ASPECT_RATIO'] += 1
                                self.log_diagnostic(f"[LEAF GEOMETRY REJECT]\nroi=FULL\nclass={cls_idx}\nreason=ASPECT_RATIO")
                                roi_diag['geometry_reject_count'] += 1
                                continue
                                
                            accepted_geometry += 1
                            roi_diag['geometry_pass_count'] += 1
                            norm_name = raw_name.lower().replace(" ", "_")
                            d_class = "Unknown"
                            if "algal" in norm_name or "algae" in norm_name:
                                d_class = "Leaf_Algal"
                            elif "spot" in norm_name:
                                d_class = "Leaf_Spot"
                            elif "blight" in norm_name:
                                d_class = "Leaf_Blight"
                            elif "healthy" in norm_name:
                                d_class = "Leaf_Healthy"
                            elif "colletotrichum" in norm_name:
                                d_class = "Leaf_Colletotrichum"
                            elif "phomopsis" in norm_name:
                                d_class = "Leaf_Phomopsis"
                            elif "rhizoctonia" in norm_name:
                                d_class = "Leaf_Rhizoctonia"
                            
                            if d_class == "Unknown":
                                if "healthy" in norm_name: d_class = "Leaf_Healthy"
                            if "colletotrichum" in norm_name: d_class = "Leaf_Colletotrichum"
                            elif "phomopsis" in norm_name: d_class = "Leaf_Phomopsis"
                            elif "rhizoctonia" in norm_name: d_class = "Leaf_Rhizoctonia"
                            elif d_class == "Unknown":
                                # Fallback to exact match with known keys
                                for k in self.tree_config.keys():
                                    if k.lower() == norm_name:
                                        d_class = k
                                        break
            
                            all_raw_boxes.append({
                                'tile_id': tile_id,
                                'cand_idx': cand_idx,
                                'conf': conf,
                                'cls_idx': cls_idx,
                                'class_name': raw_name,
                                'norm_class': d_class,
                                'bbox': [gx1, gy1, gx2, gy2],
                                'local_bbox': [px1, py1, px2, py2]
                            })
                            
                            self.log_diagnostic(f"LEAF YOLO ACCEPTED: roi=FULL cand={cand_idx} class={cls_idx} norm_class={d_class} conf={conf:.4f}")
                            
                    # DIAGNOSTIC ROI CATEGORIZATION
                    diag_base = "/tmp/durian_roi_diagnostic"
                    if roi_diag['raw_box_count'] == 0:
                        self.leaf_diag_stats[matched_id]['no_detection'] += 1
                        cv2.imwrite(os.path.join(diag_base, f"no_detection/tree{matched_id}_full.png"), tile_crop)
                    elif roi_diag['raw_boxes_conf_ge_0_20'] == 0:
                        self.leaf_diag_stats[matched_id]['raw_low_conf'] += 1
                        cv2.imwrite(os.path.join(diag_base, f"low_conf/tree{matched_id}_full.png"), tile_crop)
                    elif roi_diag['geometry_pass_count'] == 0:
                        self.leaf_diag_stats[matched_id]['geometry_rejected_roi'] += 1
                        cv2.imwrite(os.path.join(diag_base, f"geometry_rejected/tree{matched_id}_full.png"), tile_crop)
                    elif roi_diag['geometry_pass_count'] > 0:
                        roi_passed_boxes = [b for b in all_raw_boxes if b['tile_id'] == tile_id]
                        if any(b['conf'] >= 0.20 for b in roi_passed_boxes):
                            self.leaf_diag_stats[matched_id]['success'] += 1
                            cv2.imwrite(os.path.join(diag_base, f"success/tree{matched_id}_full.png"), tile_crop)
                        else:
                            self.leaf_diag_stats[matched_id]['mixed'] += 1
                            cv2.imwrite(os.path.join(diag_base, f"mixed/tree{matched_id}_full.png"), tile_crop)
                    else:
                        self.leaf_diag_stats[matched_id]['mixed'] += 1
                        cv2.imwrite(os.path.join(diag_base, f"mixed/tree{matched_id}_full.png"), tile_crop)

            # Cumulative diagnostic accounting for this scan target.
            diag_stats = self.leaf_diag_stats[matched_id]
            diag_stats['total_candidates'] += raw_yolo_count
            diag_stats['geometry_accepted'] += accepted_geometry
            diag_stats['geometry_rejected'] += (
                raw_yolo_count - accepted_geometry
            )

            self.log_diagnostic(
                f"[LEAF GEOMETRY SUMMARY]\n"
                f"raw_yolo={raw_yolo_count}\n"
                f"geometry_accepted={accepted_geometry}\n"
                f"geometry_rejected={raw_yolo_count - accepted_geometry}"
            )
            
            # Accounting:
            # raw_yolo_count = actual YOLO model outputs
            # all_raw_boxes = geometry-accepted detections
            # confidence_valid_detections = geometry + confidence accepted
            raw_yolo_detections = raw_yolo_count
            geometry_valid_detections = len(all_raw_boxes)

            # Dynamic evidence admission threshold.
            # The threshold is derived from the selected inference result
            # for the current ROI, so it adapts to the observed environment
            # without using tree-, scene-, or model-specific rules.
            raw_conf_candidates = [
                float(box.conf[0].item())
                for _, _, box in leaf_detection_records
            ]

            if raw_conf_candidates:
                max_raw_conf = max(raw_conf_candidates)

                # Keep a sanity floor while allowing weaker environments
                # to admit evidence relative to their strongest detection.
                LEAF_EVIDENCE_CONF = max(
                    0.05,
                    min(0.20, max_raw_conf * 0.60)
                )
            else:
                LEAF_EVIDENCE_CONF = 0.20

            self.log_diagnostic(
                f"[DYNAMIC LEAF EVIDENCE THRESHOLD] "
                f"threshold={LEAF_EVIDENCE_CONF:.4f} "
                f"max_raw_conf={max_raw_conf:.4f}"
                if raw_conf_candidates
                else
                "[DYNAMIC LEAF EVIDENCE THRESHOLD] "
                "threshold=0.2000 max_raw_conf=0.0000"
            )

            confidence_valid_detections = [
                b for b in all_raw_boxes
                if b['conf'] >= LEAF_EVIDENCE_CONF
            ]
            below_confidence = (
                geometry_valid_detections - len(confidence_valid_detections)
            )

            # Per-scan confidence statistics must be committed while
            # holding the same lock used by scan lifecycle callbacks.
            raw_conf_values = [
                float(box.conf[0].item())
                for _, _, box in leaf_detection_records
            ]

            valid_conf_values = [
                float(b['conf'])
                for b in confidence_valid_detections
            ]

            with self.state_lock:
                scan_stats = self._get_scan_evidence(
                    frame_epoch,
                    matched_id,
                    create=False
                )
                if scan_stats is None:
                    return "Unknown", 0.0, "SCAN_EVIDENCE_EXPIRED", 0, 0.0

                if raw_conf_values:
                    scan_stats['sum_raw_conf'] += sum(raw_conf_values)
                    scan_stats['max_raw_conf'] = max(
                        scan_stats.get('max_raw_conf', 0.0),
                        max(raw_conf_values)
                    )

                if valid_conf_values:
                    scan_stats['max_valid_conf'] = max(
                        scan_stats.get('max_valid_conf', 0.0),
                        max(valid_conf_values)
                    )
            
            def associate_leaf_instances(detections):
                import math
                detections.sort(key=lambda x: x['conf'], reverse=True)
                retained = []
                for box in detections:
                    discard = False
                    for keep_box in retained:
                        x1_1, y1_1, x2_1, y2_1 = box['bbox']
                        x1_2, y1_2, x2_2, y2_2 = keep_box['bbox']
                        
                        area1 = max(0, x2_1 - x1_1) * max(0, y2_1 - y1_1)
                        area2 = max(0, x2_2 - x1_2) * max(0, y2_2 - y1_2)
                        min_area = min(area1, area2)
                        
                        xA = max(x1_1, x1_2)
                        yA = max(y1_1, y1_2)
                        xB = min(x2_1, x2_2)
                        yB = min(y2_1, y2_2)
                        interArea = max(0, xB - xA) * max(0, yB - yA)
                        
                        iou = interArea / float(area1 + area2 - interArea + 1e-5)
                        iom = interArea / float(min_area + 1e-5) if min_area > 0 else 0.0
                        
                        cx1, cy1 = (x1_1 + x2_1) / 2.0, (y1_1 + y2_1) / 2.0
                        cx2, cy2 = (x1_2 + x2_2) / 2.0, (y1_2 + y2_2) / 2.0
                        c_dist = math.sqrt((cx1 - cx2)**2 + (cy1 - cy2)**2)
                        norm_c_dist = c_dist / math.sqrt(min_area) if min_area > 0 else 999.0
                        
                        reason = None
                        
                        # Fix: Make NMS class-agnostic. Overlapping boxes suppress the lower confidence one,
                        # regardless of whether YOLO hallucinated multiple classes on the same physical leaf.
                        if iou > 0.40:
                            reason = "HIGH_IOU_ASSOCIATION"
                        elif iom > 0.85 and norm_c_dist < 0.50:
                            reason = "CONCENTRIC_CONTAINMENT_ASSOCIATION"
                            
                        if reason:
                            discard = True
                            self.log_diagnostic(
                                f"[ASSOCIATION SUPPRESSED]\n"
                                f"tree={matched_id}\n"
                                f"suppressed_roi={box.get('cand_idx', 'unknown')}\n"
                                f"kept_roi={keep_box.get('cand_idx', 'unknown')}\n"
                                f"suppressed_conf={box['conf']:.4f}\n"
                                f"kept_conf={keep_box['conf']:.4f}\n"
                                f"iou={iou:.3f}\niom={iom:.3f}\n"
                                f"norm_c_dist={norm_c_dist:.3f}\n"
                                f"reason={reason}"
                            )
                            break
                        else:
                            if iou > 0.01:
                                self.log_diagnostic(
                                    f"[ASSOCIATION PRESERVED]\n"
                                    f"tree={matched_id}\n"
                                    f"roi1={box.get('cand_idx', 'unknown')}\n"
                                    f"roi2={keep_box.get('cand_idx', 'unknown')}\n"
                                    f"iou={iou:.3f}\niom={iom:.3f}\n"
                                    f"reason=GENUINE_OVERLAP_PRESERVED"
                                )
                    if not discard:
                        retained.append(box)
                return retained

            nms_boxes = associate_leaf_instances(confidence_valid_detections)
                    
            duplicates_removed = len(confidence_valid_detections) - len(nms_boxes)
            final_detections = len(nms_boxes)
            
            self.get_logger().info(
                f"LEAF FILTER:\n"
                f"raw_yolo_detections={raw_yolo_detections}\n"
                f"geometry_valid_detections={geometry_valid_detections}\n"
                f"below_confidence={below_confidence}\n"
                f"confidence_valid_detections={len(confidence_valid_detections)}\n"
                f"duplicates_removed={duplicates_removed}\n"
                f"final_nms_detections={final_detections}"
            )
            
            import cv2
            debug_crop = crop.copy()
            # Raw YOLO evidence is independent of geometry acceptance.
            has_yolo_boxes = raw_yolo_detections > 0
            has_valid = len(nms_boxes) > 0
            
            frame_votes = {}
            roi_accum = {}
            for box in nms_boxes:
                conf = box['conf']
                raw_name = box['class_name']
                norm_name = str(raw_name).strip().lower().replace(" ", "_")

                if "blight" in norm_name:
                    cls_name = "Leaf_Blight"
                elif "algal" in norm_name or "algae" in norm_name:
                    cls_name = "Leaf_Algal"
                elif "colletotrichum" in norm_name or "anthracnose" in norm_name:
                    cls_name = "Leaf_Colletotrichum"
                elif "phomopsis" in norm_name:
                    cls_name = "Leaf_Phomopsis"
                elif "rhizoctonia" in norm_name:
                    cls_name = "Leaf_Rhizoctonia"
                elif "healthy" in norm_name:
                    cls_name = "Leaf_Healthy"
                else:
                    cls_name = raw_name
                
                # Keep frame_votes for the immediate per-frame visual result logging
                if conf > frame_votes.get(cls_name, 0.0):
                    frame_votes[cls_name] = conf
                    
                # New ROI-level accumulation
                if cls_name not in roi_accum:
                    roi_accum[cls_name] = {'count': 0, 'conf_sum': 0.0}
                roi_accum[cls_name]['count'] += 1
                roi_accum[cls_name]['conf_sum'] += conf
                
                gx1, gy1, gx2, gy2 = map(int, box['bbox'])
                cv2.rectangle(debug_crop, (gx1, gy1), (gx2, gy2), (0, 255, 0), 2)
                cv2.putText(debug_crop, f"{cls_name} {conf:.2f}", (gx1, max(10, gy1 - 5)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            
            cv2.imwrite(f"/home/johny/durian_ws/debug_leaf_{matched_id}_{debug_id}.jpg", debug_crop)
            
            vote_target_id = matched_id

            # Atomic frame accounting.
            # yolo_frames / valid_frames / rejected_frames and the
            # frame-level class evidence must belong to exactly the same
            # evidence object and session.
            with self.state_lock:
                evidence = self._get_scan_evidence(
                    frame_epoch,
                    matched_id,
                    create=False
                )

                if evidence is None:
                    return "Unknown", 0.0, "SCAN_EVIDENCE_EXPIRED", 0, 0.0

                if has_yolo_boxes:
                    evidence['yolo_frames'] += 1

                if has_valid:
                    evidence['valid_frames'] += 1
                elif has_yolo_boxes:
                    evidence['rejected_frames'] += 1

                if has_valid:
                    for d_class, frame_conf in frame_votes.items():
                        evidence['counts'][d_class] = (
                            evidence['counts'].get(d_class, 0) + 1
                        )
                        evidence['conf_sum'][d_class] = (
                            evidence['conf_sum'].get(d_class, 0.0)
                            + float(frame_conf)
                        )
                
            leaf_detections = len(nms_boxes)

            if not has_valid:
                d_name = "Unknown"
                reason = (
                    "NO_LEAF_DETECTION"
                    if raw_yolo_detections == 0
                    else "NO_VALID_LEAF_EVIDENCE"
                )
                leaf_conf = 0.0
            else:
                # 1. Accumulate FRAME-LEVEL evidence.
                # Each class can contribute at most one vote per frame.
                # Use the strongest retained detection for that class,
                # rather than allowing multiple boxes from the same frame
                # to accumulate disproportionate evidence.
                # Frame-level evidence was already committed atomically
                # above together with valid_frames.
                
                evidence_str = ", ".join(
                    [f"{k}: count={v['count']} sum={v['conf_sum']:.4f}"
                     for k, v in roi_accum.items()]
                )

                # Snapshot the committed session evidence under the lock.
                # Never log a live mutable evidence dictionary outside it.
                with self.state_lock:
                    evidence_snapshot = self._get_scan_evidence(
                        frame_epoch,
                        matched_id,
                        create=False
                    )

                    if evidence_snapshot is None:
                        return "Unknown", 0.0, "SCAN_EVIDENCE_EXPIRED", 0, 0.0

                    snapshot_counts = dict(evidence_snapshot.get('counts', {}))
                    snapshot_conf_sums = dict(
                        evidence_snapshot.get('conf_sum', {})
                    )

                self.log_diagnostic(
                    f"[FRAME EVIDENCE RETAINED]\n"
                    f"tree_id={matched_id}\n"
                    f"epoch={int(frame_epoch)}\n"
                    f"retained_classes={evidence_str}\n"
                    f"scan_counts={snapshot_counts}\n"
                    f"scan_conf_sums={snapshot_conf_sums}"
                )

                # 2. Determine primary disease ONLY for the immediate per-frame visual result.
                # This is NOT the scan-level aggregation decision.
                top_d = max(frame_votes.items(), key=lambda item: item[1])[0]
                vote_weight = frame_votes[top_d]
                
                if "Healthy" in top_d:
                    reason = "HEALTHY_DETECTED"
                else:
                    reason = "DISEASE_DETECTED"
                    
                d_name = top_d
                leaf_conf = vote_weight
                severity = self.severity_map.get(d_name, 0.0)
                
                self.log_diagnostic(
                    f"[FRAME PRIMARY RESULT]\n"
                    f"tree_id={matched_id}\n"
                    f"primary_class={d_name}\n"
                    f"primary_conf={leaf_conf:.4f}"
                )

        self.log_diagnostic(
            f"[LEAF RESULT]\n"
            f"result={d_name}\n"
            f"reason={reason}\n"
            f"raw_yolo_detections={raw_yolo_detections}\n"
            f"geometry_valid_detections={geometry_valid_detections}\n"
            f"final_nms_detections={len(nms_boxes)}"
        )
        
        self.log_diagnostic(
            f"[VISION PIPELINE END]\n"
            f"tree_id={matched_id}\n"
            f"tree_yolo_conf={tree_conf:.4f}\n"
            f"tree_tracking_status={'EXISTING' if matched_id in self.tree_tracker else 'NEW'}\n"
            f"roi={'VALID' if crop is not None and crop.size > 0 else 'INVALID'}\n"
            f"leaf_raw={raw_yolo_detections}\n"
            f"leaf_geometry_valid={geometry_valid_detections}\n"
            f"leaf_valid={len(nms_boxes)}\n"
            f"disease={d_name}\n"
            f"reason={reason}"
        )
        
        # --- DIAGNOSTIC SUMMARY PRINT ---
        if hasattr(self, 'leaf_diag_stats') and matched_id in self.leaf_diag_stats:
            stats = self.leaf_diag_stats[matched_id]
            # geometry_accepted / geometry_rejected are cumulative.
            # Per-frame final counts remain separate.
            stats['valid_confidence_count'] = len(confidence_valid_detections)
            stats['nms_count'] = len(nms_boxes)
            
            y_eval = stats['yolo_evaluated']
            if y_eval > 0:
                yolo_success_rate = stats['success'] / y_eval
                yolo_nodet_rate = stats['no_detection'] / y_eval
                yolo_lowconf_rate = stats['raw_low_conf'] / y_eval
                geom_reject_rate = stats['geometry_rejected_roi'] / y_eval
            else:
                yolo_success_rate = yolo_nodet_rate = yolo_lowconf_rate = geom_reject_rate = 0.0
                
            w_min = min(stats['widths']) if stats['widths'] else 0
            w_max = max(stats['widths']) if stats['widths'] else 0
            w_mean = sum(stats['widths'])/len(stats['widths']) if stats['widths'] else 0
            
            c_min = min(stats['confs']) if stats['confs'] else 0
            c_max = max(stats['confs']) if stats['confs'] else 0
            
            confs_sorted = sorted(stats['confs'])
            if confs_sorted:
                mid = len(confs_sorted) // 2
                c_median = (confs_sorted[mid] + confs_sorted[~mid]) / 2.0
            else:
                c_median = 0
                
            c_mean = sum(stats['confs'])/len(stats['confs']) if stats['confs'] else 0

            self.get_logger().info(f"""
===== LEAF ROOT CAUSE DIAGNOSTIC =====

Tree ID: {matched_id}

ROI candidates:
  total: {stats['total_candidates']}
  geometry accepted: {stats['geometry_accepted']}
  geometry rejected: {stats['geometry_rejected']}

YOLO:
  evaluated: {y_eval}
  detections >= 0.20: {stats['success'] + stats['geometry_rejected_roi'] + stats['mixed']}
  detections < 0.20: {stats['raw_low_conf']}
  no detection: {stats['no_detection']}

YOLO Rates:
  success rate: {yolo_success_rate:.4f}
  no-detection rate: {yolo_nodet_rate:.4f}
  raw-low-conf rate: {yolo_lowconf_rate:.4f}
  geometry-reject rate: {geom_reject_rate:.4f}

Confidence:
  mean: {c_mean:.4f}
  median: {c_median:.4f}
  max: {c_max:.4f}

ROI dimensions:
  min: {w_min}
  max: {w_max}
  mean: {w_mean:.2f}

Geometry rejection:
  BORDER_TOUCH: {stats['reason_BORDER_TOUCH']}
  AREA_RATIO: {stats['reason_AREA_RATIO']}
  ABSOLUTE_SIZE: {stats['reason_ABSOLUTE_SIZE']}
  ASPECT_RATIO: {stats['reason_ASPECT_RATIO']}

Final:
  valid detections: {stats['valid_confidence_count']}
  NMS detections: {stats['nms_count']}
  disease result: {d_name}
=======================================
""")

        return d_name, severity, reason, leaf_detections, leaf_conf

    def _snapshot_scan_evidence(self, epoch, target_id):
        """Atomically copy one completed scan evidence object."""
        if target_id is None:
            return None

        with self.state_lock:
            evidence = self._get_scan_evidence(
                epoch,
                target_id,
                create=False
            )

            if evidence is None:
                return None

            return {
                'epoch': evidence.get('epoch'),
                'target_id': evidence.get('target_id'),
                'counts': dict(evidence.get('counts', {})),
                'conf_sum': dict(evidence.get('conf_sum', {})),
                'total_crops': int(evidence.get('total_crops', 0)),
                'yolo_frames': int(evidence.get('yolo_frames', 0)),
                'valid_frames': int(evidence.get('valid_frames', 0)),
                'rejected_frames': int(evidence.get('rejected_frames', 0)),
                'sum_raw_conf': float(evidence.get('sum_raw_conf', 0.0)),
                'max_raw_conf': float(evidence.get('max_raw_conf', 0.0)),
                'max_valid_conf': float(evidence.get('max_valid_conf', 0.0))
            }

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
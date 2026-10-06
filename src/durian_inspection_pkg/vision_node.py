#!/usr/bin/env python3
"""Lightweight synchronized RGB-D capture node with no inference code."""

import hashlib
import json
import os
import threading
import time
import uuid
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
import tf2_ros

from observation_store import connect


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _transform_dict(transform):
    if transform is None:
        return None
    t = transform.transform.translation
    q = transform.transform.rotation
    return {
        "header": {"frame_id": transform.header.frame_id, "stamp": _stamp_seconds(transform.header.stamp)},
        "child_frame_id": transform.child_frame_id,
        "translation": {"x": t.x, "y": t.y, "z": t.z},
        "rotation": {"x": q.x, "y": q.y, "z": q.z, "w": q.w},
    }


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.declare_parameter("capture_root", "/home/johny/durian_ws/captured_images")
        self.declare_parameter("database_path", "/home/johny/durian_ws/durian_inspection.db")
        self.declare_parameter("capture_interval_sec", 0.1)
        self.declare_parameter("rgb_depth_tolerance_sec", 0.15)
        self.declare_parameter("camera_frame", "camera_link_optical")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("tf_lookup_timeout_sec", 0.3)
        self.declare_parameter("tf_latest_fallback_max_age_sec", 1.5)

        self.capture_root = os.path.abspath(str(self.get_parameter("capture_root").value))
        self.db_path = os.path.abspath(str(self.get_parameter("database_path").value))
        os.makedirs(self.capture_root, exist_ok=True)
        with connect(self.db_path):
            pass

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.lock = threading.RLock()
        self.depth_buffer = deque(maxlen=60)
        self.camera_info = None
        self.inspection_id = None
        self.tree_id = None
        self.target_x = None
        self.target_y = None
        self.viewpoint_id = None
        self.viewpoint_index = None
        self.capture_active = False
        self.capture_epoch = 0
        self.last_capture_wall = 0.0
        self.accepted = 0
        self.rejected = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        session_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        callbacks = ReentrantCallbackGroup()
        self.create_subscription(CameraInfo, "/camera/camera_info", self.camera_info_callback, sensor_qos, callback_group=callbacks)
        self.create_subscription(Image, "/camera/depth/image_raw", self.depth_callback, sensor_qos, callback_group=callbacks)
        self.create_subscription(Image, "/camera/image_raw", self.image_callback, sensor_qos, callback_group=callbacks)
        self.create_subscription(Point, "/scan_target_tree", self.scan_target_callback, 10, callback_group=callbacks)
        self.create_subscription(String, "/scan_viewpoint", self.scan_viewpoint_callback, 10, callback_group=callbacks)
        self.create_subscription(Bool, "/scan_active", self.scan_active_callback, 10, callback_group=callbacks)
        self.create_subscription(String, "/inspection_id", self.inspection_id_callback, session_qos, callback_group=callbacks)
        self.get_logger().info(f"Capture-only vision node ready: {self.capture_root}")

    def inspection_id_callback(self, msg):
        with self.lock:
            self.inspection_id = msg.data.strip() or None

    def scan_target_callback(self, msg):
        with self.lock:
            self.target_x = float(msg.x)
            self.target_y = float(msg.y)
            new_tree_id = int(msg.z) if msg.z >= 0 else None
            if new_tree_id != self.tree_id:
                self.viewpoint_id = None
                self.viewpoint_index = None
            self.tree_id = new_tree_id

    def scan_viewpoint_callback(self, msg):
        try:
            payload = json.loads(msg.data)
            viewpoint_id = str(payload["viewpoint_id"])
            viewpoint_index = int(payload["viewpoint_index"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Invalid scan viewpoint metadata: {exc}")
            return
        with self.lock:
            self.viewpoint_id = viewpoint_id
            self.viewpoint_index = viewpoint_index

    def scan_active_callback(self, msg):
        with self.lock:
            self.capture_active = bool(msg.data)
            if self.capture_active:
                self.capture_epoch += 1
                self.accepted = 0
                self.rejected = 0
                self.last_capture_wall = 0.0
                if not self.inspection_id:
                    self.inspection_id = time.strftime("inspection_%Y%m%d_%H%M%S")
                self.get_logger().info(f"CAPTURE START inspection={self.inspection_id} tree={self.tree_id}")
            else:
                self.get_logger().info(f"CAPTURE STOP tree={self.tree_id} accepted={self.accepted} rejected={self.rejected}")
                self.viewpoint_id = None
                self.viewpoint_index = None

    def camera_info_callback(self, msg):
        with self.lock:
            self.camera_info = msg

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self.lock:
                self.depth_buffer.append((_stamp_seconds(msg.header.stamp), msg, depth.copy()))
        except Exception as exc:
            self.get_logger().error(f"Depth conversion failed: {exc}")

    def image_callback(self, msg):
        now = time.monotonic()
        with self.lock:
            if (
                not self.capture_active or self.tree_id is None
                or self.viewpoint_id is None or self.viewpoint_index is None
                or self.camera_info is None
            ):
                return
            interval = float(self.get_parameter("capture_interval_sec").value)
            if now - self.last_capture_wall < interval:
                return
            rgb_t = _stamp_seconds(msg.header.stamp)
            best = min(self.depth_buffer, key=lambda item: abs(item[0] - rgb_t), default=None)
            tolerance = float(self.get_parameter("rgb_depth_tolerance_sec").value)
            if best is None or abs(best[0] - rgb_t) > tolerance:
                self.rejected += 1
                return
            self.last_capture_wall = now
            context = {
                "inspection_id": self.inspection_id,
                "tree_id": self.tree_id,
                "target_x": self.target_x,
                "target_y": self.target_y,
                "capture_epoch": self.capture_epoch,
                "viewpoint_id": self.viewpoint_id,
                "viewpoint_index": self.viewpoint_index,
                "camera_info": self.camera_info,
                "depth_stamp": best[1].header.stamp,
            }
            depth = best[2].copy()

        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._persist_observation(msg, rgb, depth, context)
            with self.lock:
                self.accepted += 1
        except Exception as exc:
            with self.lock:
                self.rejected += 1
            self.get_logger().error(f"Capture persistence failed: {exc}")

    def _lookup_capture_transform(self, target, source, stamp):
        """Prefer the image timestamp, then allow a bounded stationary fallback."""
        requested_sec = float(stamp.nanoseconds) * 1e-9
        try:
            timeout = Duration(
                seconds=float(self.get_parameter("tf_lookup_timeout_sec").value)
            )
            transform = self.tf_buffer.lookup_transform(target, source, stamp, timeout)
            return transform, {"mode": "exact", "age_sec": 0.0}
        except Exception as exact_exc:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target, source, rclpy.time.Time(), timeout
                )
                transform_sec = _stamp_seconds(transform.header.stamp)
                age_sec = abs(requested_sec - transform_sec)
                maximum_age = float(
                    self.get_parameter("tf_latest_fallback_max_age_sec").value
                )
                if transform_sec <= 0.0 or age_sec > maximum_age:
                    raise RuntimeError(
                        f"latest transform age {age_sec:.3f}s exceeds "
                        f"configured {maximum_age:.3f}s"
                    )
                self.get_logger().warning(
                    "Capture TF used bounded latest fallback "
                    f"{source}->{target}: age={age_sec:.3f}s",
                    throttle_duration_sec=2.0,
                )
                return transform, {"mode": "bounded_latest", "age_sec": age_sec}
            except Exception as fallback_exc:
                self.get_logger().warning(
                    f"Capture TF unavailable {source}->{target}: exact={exact_exc}; "
                    f"fallback={fallback_exc}",
                    throttle_duration_sec=2.0,
                )
                return None, {
                    "mode": "unavailable",
                    "age_sec": None,
                    "error": type(fallback_exc).__name__,
                }

    def _lookup(self, target, source, stamp):
        """Compatibility wrapper used by focused tests and diagnostics."""
        transform, _ = self._lookup_capture_transform(target, source, stamp)
        return transform

    def _persist_observation(self, rgb_msg, rgb, depth, context):
        rgb_t = _stamp_seconds(rgb_msg.header.stamp)
        depth_t = _stamp_seconds(context["depth_stamp"])
        stamp = rclpy.time.Time.from_msg(rgb_msg.header.stamp)
        camera_frame = str(self.get_parameter("camera_frame").value)
        odom_frame = str(self.get_parameter("odom_frame").value)
        map_frame = str(self.get_parameter("map_frame").value)
        robot_frame = str(self.get_parameter("robot_frame").value)

        transform_requests = {
            "odom_from_camera": (odom_frame, camera_frame),
            "odom_from_map": (odom_frame, map_frame),
            "map_from_robot": (map_frame, robot_frame),
        }
        transforms = {}
        transform_provenance = {}
        for name, (target, source) in transform_requests.items():
            transform, provenance = self._lookup_capture_transform(
                target, source, stamp
            )
            transforms[name] = _transform_dict(transform)
            transform_provenance[name] = provenance
        missing = [name for name, value in transforms.items() if value is None]
        if missing:
            raise RuntimeError(
                "required capture transform unavailable; observation rejected: "
                + ",".join(missing)
            )
        rgb_hash = hashlib.sha256(np.ascontiguousarray(rgb).tobytes()).hexdigest()
        depth_hash = hashlib.sha256(np.ascontiguousarray(depth).tobytes()).hexdigest()
        observation_hash = hashlib.sha256((rgb_hash + depth_hash).encode("ascii")).hexdigest()
        context_material = {
            "tree_id": context["tree_id"],
            "viewpoint_id": context["viewpoint_id"],
            "viewpoint_index": context["viewpoint_index"],
            "target_map": {"x": context["target_x"], "y": context["target_y"], "z": 1.2},
            "camera_k": list(context["camera_info"].k),
            "transforms": transforms,
            "transform_provenance": transform_provenance,
        }
        context_hash = hashlib.sha256(
            json.dumps(context_material, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
        observation_id = f"{int(rgb_t)}_{int((rgb_t % 1) * 1e9):09d}_{uuid.uuid4().hex[:8]}"
        tree_dir = os.path.join(self.capture_root, str(context["inspection_id"]), str(context["tree_id"]))
        os.makedirs(tree_dir, exist_ok=True)
        rgb_path = os.path.join(tree_dir, f"{observation_id}_rgb.png")
        depth_path = os.path.join(tree_dir, f"{observation_id}_depth.tiff")
        metadata_path = os.path.join(tree_dir, f"{observation_id}.json")

        camera = context["camera_info"]
        metadata = {
            "observation_id": observation_id,
            "inspection_id": context["inspection_id"],
            "tree_id": context["tree_id"],
            "viewpoint_id": context["viewpoint_id"],
            "viewpoint_index": context["viewpoint_index"],
            "target_map": {"x": context["target_x"], "y": context["target_y"], "z": 1.2},
            "rgb": {"timestamp": rgb_t, "frame_id": rgb_msg.header.frame_id, "encoding": "bgr8", "sha256": rgb_hash},
            "depth": {"timestamp": depth_t, "encoding": str(depth.dtype), "sha256": depth_hash},
            "camera_info": {
                "frame_id": camera.header.frame_id, "width": camera.width, "height": camera.height,
                "distortion_model": camera.distortion_model,
                "d": list(camera.d), "k": list(camera.k), "r": list(camera.r), "p": list(camera.p),
            },
            "transforms": transforms,
            "transform_provenance": transform_provenance,
            "observation_sha256": observation_hash,
            "context_sha256": context_hash,
        }

        tmp_rgb, tmp_depth, tmp_meta = f"{rgb_path}.tmp.png", f"{depth_path}.tmp.tiff", f"{metadata_path}.tmp"
        if not cv2.imwrite(tmp_rgb, rgb):
            raise IOError(f"Could not write {tmp_rgb}")
        if not cv2.imwrite(tmp_depth, depth):
            raise IOError(f"Could not write {tmp_depth}")
        with open(tmp_meta, "w", encoding="utf-8") as stream:
            json.dump(metadata, stream, indent=2, sort_keys=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp_rgb, rgb_path)
        os.replace(tmp_depth, depth_path)
        os.replace(tmp_meta, metadata_path)

        with connect(self.db_path) as conn:
            conn.execute(
                """INSERT INTO captured_observations (
                    observation_id, inspection_id, tree_id, viewpoint_id,
                    viewpoint_index, rgb_path, depth_path,
                    metadata_path, rgb_sha256, depth_sha256, observation_sha256,
                    context_sha256, rgb_timestamp, depth_timestamp, status
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 'CAPTURED')""",
                (observation_id, context["inspection_id"], context["tree_id"],
                 context["viewpoint_id"], context["viewpoint_index"], rgb_path,
                 depth_path, metadata_path, rgb_hash, depth_hash, observation_hash,
                 context_hash, rgb_t, depth_t),
            )


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

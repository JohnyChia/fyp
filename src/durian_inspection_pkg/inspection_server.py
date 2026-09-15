#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, Point, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from nav2_msgs.action import NavigateToPose
from durian_message.srv import InspectionControl, TreeStatus
from action_msgs.msg import GoalStatus
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
import numpy as np
import math
import os
import xml.etree.ElementTree as ET
import time
import threading
import sqlite3
import datetime
import struct
import uuid
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
from std_srvs.srv import Trigger
from observation_store import connect


class InspectionServer(Node):
    def __init__(self):
        super().__init__("inspection_server")
        self.data_lock = threading.Lock()
        self.has_authoritative_trees = False

        self.cb_group = ReentrantCallbackGroup()

        self.service = self.create_service(
            InspectionControl,
            "control_inspection",
            self.command_callback,
            callback_group=self.cb_group
        )

        static_map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, "/static_map", self.cloud_callback,
            qos_profile=static_map_qos, callback_group=self.cb_group
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10, callback_group=self.cb_group
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.target_pub = self.create_publisher(Point, '/locked_target_tree', 10)
        from std_msgs.msg import Bool, Int32
        self.scan_active_pub = self.create_publisher(Bool, "/scan_active", 10)
        self.scan_target_pub = self.create_publisher(Point, "/scan_target_tree", 10)
        inspection_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.inspection_id_pub = self.create_publisher(String, "/inspection_id", inspection_qos)
        self._last_target_pub = 0.0

        self.nav_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose", callback_group=self.cb_group
        )
        self.vision_client = self.create_client(TreeStatus, 'get_tree_status', callback_group=self.cb_group)
        self.processing_client = self.create_client(
            Trigger, 'process_captured_images', callback_group=self.cb_group
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_server_connected = False
        self.nav_check_timer = self.create_timer(1.0, self.check_nav_server_callback, callback_group=self.cb_group)

        self.db_path = "/home/johny/durian_ws/durian_inspection.db"
        self.init_database()
        self.current_inspection_id = None
        self.processing_future = None

        self.robot_x, self.robot_y, self.robot_yaw = 0.0, 0.0, 0.0
        self.robot_vx = 0.0
        self.robot_vy = 0.0
        self.robot_vtheta = 0.0, 0.0, 0.0
        self.initial_inspect_x = 0.0
        self.initial_inspect_y = 0.0

        self.trees = []
        self.completed_trees = []
        self.current_tree = None
        self.last_cluster_time = 0.0

        self.last_yaw = 0.0
        self.total_rotated_angle = 0.0
        self.scanning = False
        self.scan_start_time = None
        self.scan_duration = 0.0
        self.returning_home = False
        self.global_tree_id = 0

        self.declare_parameter('world_path', '')
        self.world_path = os.path.expanduser(
            str(self.get_parameter('world_path').value)
        )
        self.authoritative_trees = []
        self.load_authoritative_trees_from_world()

        self.TREE_MATCH_RADIUS = 1.0
        self.COMPLETED_TREE_RADIUS = 1.0
        self.inspection_finished = False
        self.locked_target_tree = None

        self.recovery_step = 0
        self.recovery_start = 0
        self.recovery_escape_angle = 0.0

        self.home_x, self.home_y, self.home_yaw = 0.0, 0.0, 0.0

        self.state = "IDLE"
        self.running = False
        self.nav_retry = 0
        self.max_retry = 3

        self.nav_goal_handle = None
        self.nav_goal_seq = 0
        self.nav_active = False
        self.nav_result = None
        self.nav_start_time = None
        self.nav_timeout = 180.0
        self.goal_pose = None
        self._cached_tree_res = None

        self.load_completed()

        self.load_trees_from_db()

        self.state_thread = threading.Thread(target=self.state_machine_loop, daemon=True)
        self.state_thread.start()

        self.get_logger().info("Durian Inspection Server connected.")

    def check_nav_server_callback(self):
        if not self.nav_server_connected:
            if self.nav_client.server_is_ready():
                self.nav_server_connected = True
                self.get_logger().info("Nav2 Action Server connected successfully!")
                self.nav_check_timer.cancel()
            else:
                self.get_logger().warn("Waiting for Nav2 Action Server response...", throttle_duration_sec=10)

    def init_database(self):
        try:
            with connect(self.db_path):
                pass
        except Exception as e:
            self.get_logger().error(f"database connected failed : {e}")

    def load_completed(self):
        try:
            conn = sqlite3.connect(self.db_path, timeout=10.0)
            cursor = conn.cursor()
            cursor.execute("SELECT tree_x, tree_y FROM inspection_log WHERE status='completed'")
            rows = cursor.fetchall()
            conn.close()

            with self.data_lock:
                self.completed_trees = [(r[0], r[1]) for r in rows]
            self.get_logger().info(f"completed inspection {len(rows)} tree")
        except Exception as e:
            self.get_logger().error(f"loading data failed : {e}")

    def command_callback(self, request, response):
        if not self.nav_server_connected and request.command == "start":
            self.get_logger().warn("Execution rejected: Nav2 system is not fully ready yet!")
            response.success = False
            return response

        if request.command == "start":
            self.stop_robot()
            self.cancel_navigation()

            pose_tuple = self.get_robot_pose_in_map()
            retry_count = 0
            while (pose_tuple is None or pose_tuple[0] is None) and retry_count < 20:
                self.get_logger().info("Waiting for TF to capture valid initial position...")
                time.sleep(0.5)
                pose_tuple = self.get_robot_pose_in_map()
                retry_count += 1

            if pose_tuple is None or pose_tuple[0] is None:
                self.get_logger().error("Failed to get TF for initial position after 10s. Aborting START.")
                response.success = False
                return response

            rx, ry, map_yaw = pose_tuple
            inspection_id = f"inspection_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"

            with self.data_lock:
                self.current_inspection_id = inspection_id
                self.processing_future = None
                self.inspection_finished = False
                self.returning_home = False
                self.return_home_retry = 0
                self.home_x, self.home_y, self.home_yaw = rx, ry, map_yaw
                self.initial_inspect_x = rx
                self.initial_inspect_y = ry
                self.running = True
                self.state = "FIND_TREE"
                self.completed_trees.clear()
                self.current_tree = None
                self.locked_target_tree = None
                self.nav_goal_handle = None
                self.nav_goal_seq = 0
                self.nav_active = False
                self.nav_result = None

            inspection_msg = String()
            inspection_msg.data = inspection_id
            self.inspection_id_pub.publish(inspection_msg)
            self.get_logger().info("INSPECTION START")
            self.get_logger().info(f"INSPECTION ID: {inspection_id}")
            self.get_logger().info(f"INITIAL POSE:\nx={self.initial_inspect_x:.2f}\ny={self.initial_inspect_y:.2f}\nyaw={self.home_yaw:.2f}")
            response.success = True
        elif request.command == "stop":
            self.get_logger().info("Received GUI [STOP/CANCEL] command - Interrupting task and returning home immediately...")
            self.stop_robot()
            self.cancel_navigation()

            with self.data_lock:
                self.running = True
                self.returning_home = True
                self.state = "RETURN_HOME"
                self.nav_result = None
                self.nav_active = False
                self.nav_retry = 0
                self.recovery_step = 0
                self.scanning = False
                self.current_tree = None
                self.locked_target_tree = None

            response.success = True
        else:
            response.success = False
        return response

    def odom_callback(self, msg):
        with self.data_lock:
            old_yaw = self.robot_yaw
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny = 2 * (q.w * q.z + q.x * q.y)
            cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny, cosy)
            self.robot_vx = msg.twist.twist.linear.x
            self.robot_vy = msg.twist.twist.linear.y
            self.robot_vtheta = msg.twist.twist.angular.z

            if self.scanning:
                diff = self.robot_yaw - old_yaw
                while diff > math.pi: diff -= 2 * math.pi
                while diff < -math.pi: diff += 2 * math.pi
                self.total_rotated_angle += abs(diff)

    def cloud_callback(self, msg):
        now = time.time()
        if now - self.last_cluster_time < 2.0:
            return
        self.last_cluster_time = now

        try:
            points = []
            for p in pc2.read_points(msg, field_names=["x", "y", "z", "rgb"], skip_nans=True):
                x, y, z, rgb = p
                if 0.1 < z < 3.0:
                    points.append([x, y, z])

            if len(points) < 20:
                return

            points = np.array(points)
            voxel_size = 0.15
            voxel_points = {
                (int(p[0]/voxel_size), int(p[1]/voxel_size), int(p[2]/voxel_size)): p
                for p in points
            }
            filtered = np.array(list(voxel_points.values()))

            clustering = DBSCAN(eps=0.3, min_samples=10).fit(filtered[:, :2])
            labels = clustering.labels_
            local_trees = []

            for label in set(labels):
                if label == -1:
                    continue
                cluster = filtered[labels == label]
                if len(cluster) < 20:
                    continue

                cx = float(np.mean(cluster[:, 0]))
                cy = float(np.mean(cluster[:, 1]))
                cz = float(np.max(cluster[:, 2]))

                if cz > 1.5:
                    local_trees.append((cx, cy, cz, self.global_tree_id))
                    self.global_tree_id += 1

            with self.data_lock:
                if len(local_trees) > 0:
                    if not self.has_authoritative_trees:
                        self.get_logger().info(
                            f"Point cloud received ({len(local_trees)} trees). "
                            f"Keeping DB inspection inventory ({len(self.trees)} trees); "
                            f"using cloud only for spatial update."
                        )
                        self.has_authoritative_trees = True

                for lx, ly, lz, _ in local_trees:
                    found = False
                    for i, (tx, ty, tz, tid) in enumerate(self.trees):
                        if math.hypot(lx - tx, ly - ty) < self.TREE_MATCH_RADIUS:
                            is_locked = False
                            if self.locked_target_tree:
                                l_tx, l_ty, _, l_tid = self.locked_target_tree
                                if math.hypot(tx - l_tx, ty - l_ty) < 1.0:
                                    is_locked = True
                            if not self.has_authoritative_trees and not is_locked:
                                self.trees[i] = (0.7 * tx + 0.3 * lx, 0.7 * ty + 0.3 * ly, lz, tid)
                            found = True
                            break
                    if not self.has_authoritative_trees and not found:
                        self.trees.append((lx, ly, lz, self.global_tree_id))
                        self.global_tree_id += 1

            self.get_logger().info(f"Detected trees in cloud: {len(local_trees)}, Total known trees: {len(self.trees)}")
        except Exception as e:
            self.get_logger().error(f"Point cloud processing error: {e}")

    def get_robot_pose_in_map(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            self._last_rx = t.transform.translation.x
            self._last_ry = t.transform.translation.y
            q = t.transform.rotation
            siny = 2 * (q.w * q.z + q.x * q.y)
            cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
            self._last_ryaw = math.atan2(siny, cosy)
            self._last_tf_success_time = time.time()
            return self._last_rx, self._last_ry, self._last_ryaw
        except Exception as e:
            pass
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            self._last_rx = t.transform.translation.x
            self._last_ry = t.transform.translation.y
            q = t.transform.rotation
            siny = 2 * (q.w * q.z + q.x * q.y)
            cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
            self._last_ryaw = math.atan2(siny, cosy)
            self._last_tf_success_time = time.time()
            return self._last_rx, self._last_ry, self._last_ryaw
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            if hasattr(self, '_last_rx'):
                return self._last_rx, self._last_ry, getattr(self, '_last_ryaw', 0.0)
            return None, None, None

    def load_authoritative_trees_from_world(self):
        """Load stable tree IDs and map poses from the active Gazebo world."""
        world_path = os.path.expanduser(str(self.world_path))

        if not world_path or not os.path.isfile(world_path):
            self.get_logger().warn(
                f"AUTHORITATIVE WORLD NOT FOUND: {world_path}"
            )
            return

        try:
            root = ET.parse(world_path).getroot()
            inventory = []

            for model in root.iter("model"):
                name = model.get("name", "")

                if not name.startswith("durian_tree_"):
                    continue

                suffix = name.rsplit("_", 1)[-1]
                if not suffix.isdigit():
                    continue

                pose = model.find("pose")
                if pose is None or not pose.text:
                    continue

                values = pose.text.split()
                if len(values) < 3:
                    continue

                tree_id = int(suffix)
                x, y, z = map(float, values[:3])
                inventory.append((x, y, z, tree_id))

            inventory.sort(key=lambda item: item[3])

            with self.data_lock:
                self.authoritative_trees = list(inventory)
                self.trees = list(inventory)
                self.has_authoritative_trees = bool(inventory)

                if inventory:
                    self.global_tree_id = max(
                        item[3] for item in inventory
                    ) + 1

            self.get_logger().info(
                f"AUTHORITATIVE WORLD INVENTORY: {len(inventory)} trees"
            )

            for x, y, z, tree_id in inventory:
                self.get_logger().info(
                    f"WORLD TREE: id={tree_id} x={x:.3f} y={y:.3f} z={z:.3f}"
                )
        except Exception as e:
            self.get_logger().error(
                f"AUTHORITATIVE WORLD PARSE FAILED: {e}"
            )

    def load_trees_from_db(self):
        """从 durian_data.db 聚类提取树木位置，仅作为无 authoritative inventory 时的冷启动备用方案。"""
        if self.has_authoritative_trees:
            self.get_logger().info(
                "AUTHORITATIVE INVENTORY ACTIVE: skipping DB tree identity preload."
            )
            return
        try:
            conn = sqlite3.connect("/home/johny/durian_ws/durian_data.db", timeout=10.0)
            cursor = conn.cursor()

            cursor.execute(
                "SELECT x, y, z FROM point_clouds WHERE rowid % 20 = 0 "
                "AND z BETWEEN 0.1 AND 3.0 AND x BETWEEN -200 AND 200 AND y BETWEEN -200 AND 200"
            )
            rows = cursor.fetchall()
            conn.close()

            if not rows:
                self.get_logger().warn("DB 预加载：数据库中无点云数据")
                return

            points = np.array(rows)

            clustering = DBSCAN(eps=0.8, min_samples=10).fit(points[:, :2])
            labels = clustering.labels_
            local_trees = []
            for label in set(labels):
                if label == -1:
                    continue
                cluster = points[labels == label]
                if len(cluster) < 10:
                    continue
                cx = float(np.mean(cluster[:, 0]))
                cy = float(np.mean(cluster[:, 1]))
                cz = float(np.max(cluster[:, 2]))

                if cz > 0.3:
                    local_trees.append((cx, cy, cz, self.global_tree_id))
                    self.global_tree_id += 1
                    self.get_logger().info(f"  DB 树木: ({cx:.2f}, {cy:.2f}), 最高z={cz:.2f}, 点数={len(cluster)}")

            with self.data_lock:
                if not self.has_authoritative_trees and local_trees:
                    self.trees = local_trees
            if local_trees:
                self.get_logger().info(f"从 DB 预加载了 {len(local_trees)} 棵树（冷启动）")
        except Exception as e:
            self.get_logger().error(f"DB 预加载树木失败: {e}")

    def find_nearest_tree(self):
        nearest, min_dist = None, float("inf")

        pose_tuple = self.get_robot_pose_in_map()
        if pose_tuple is None or pose_tuple[0] is None:
            self.get_logger().warn("FIND_TREE: Waiting for valid TF pose before selecting tree...")
            return "TF_NOT_READY", float("inf")

        rx, ry, _ = pose_tuple
        with self.data_lock:
            current_trees = list(self.trees)
            done_trees = list(self.completed_trees)

        self.get_logger().info(f"TREE SELECTION:\nrobot_x={rx:.2f}\nrobot_y={ry:.2f}\ncandidate_count={len(current_trees)}")

        if self.current_tree and self.state == "NAVIGATING":
            nearest = self.current_tree
            min_dist = math.hypot(rx - nearest[0], ry - nearest[1])
            self.get_logger().info(f"ROUTE SELECT:\ncurrent=({rx:.2f},{ry:.2f})\ncandidates={len(current_trees)}\nselected=({nearest[0]:.2f},{nearest[1]:.2f})\ndistance={min_dist:.2f}\nreason=locked target")
        else:
            candidates = []
            for tree in current_trees:
                tx, ty, _, tid = tree
                inspected = any(math.hypot(tx - dx, ty - dy) < self.COMPLETED_TREE_RADIUS for dx, dy in done_trees)
                if not inspected:
                    dist = math.hypot(rx - tx, ry - ty)
                    candidates.append((dist, tid, tree))

            if candidates:
                candidates.sort(key=lambda x: (x[0], x[1]))
                min_dist, _, nearest = candidates[0]
                self.get_logger().info(f"ROUTE SELECT:\ncurrent=({rx:.2f},{ry:.2f})\ncandidates={len(candidates)}\nselected=({nearest[0]:.2f},{nearest[1]:.2f})\ndistance={min_dist:.2f}\nreason=nearest distance with stable tie-break")

        if nearest:
            self.get_logger().info(f"SELECTED NEAREST:\ntree_id={nearest[3]}\nx={nearest[0]:.2f}\ny={nearest[1]:.2f}\ndistance={min_dist:.2f}")

        return nearest, min_dist

    def state_machine_loop(self):
        last_logged_state = None
        while rclpy.ok():
            with self.data_lock:
                running = self.running
                current_state = self.state
                locked_target = self.locked_target_tree

            if locked_target:
                if time.time() - getattr(self, '_last_target_pub', 0) > 0.1:
                    pt = Point()
                    pt.x = float(locked_target[0])
                    pt.y = float(locked_target[1])
                    pt.z = float(locked_target[3])
                    self.target_pub.publish(pt)

                    self._last_target_pub = time.time()
            else:
                if time.time() - getattr(self, '_last_target_pub', 0) > 0.1:
                    pt = Point()
                    pt.x = 0.0
                    pt.y = 0.0
                    pt.z = -1.0
                    self.target_pub.publish(pt)

                    self._last_target_pub = time.time()

            if not running:
                if last_logged_state != "IDLE":
                    self.get_logger().info("State: IDLE (waiting for command)")
                    last_logged_state = "IDLE"
                time.sleep(0.2)
                continue

            if current_state != last_logged_state:
                self.get_logger().info(f"State transition: {last_logged_state} → {current_state}")
                last_logged_state = current_state

            if current_state == "FIND_TREE":
                if getattr(self, 'last_cluster_time', 0.0) == 0.0:
                    time.sleep(0.5)
                    continue
                tree, dist = self.find_nearest_tree()
                with self.data_lock:
                    if tree == "TF_NOT_READY":
                        time.sleep(0.5)
                        continue
                    elif tree is None:
                        if len(self.trees) == 0:
                            time.sleep(0.5)
                        else:
                            self.get_logger().info("ALL TREES COMPLETED")
                            self.returning_home = True
                            self.state = "RETURN_HOME"
                    else:
                        self.current_tree = tree
                        self.locked_target_tree = tree
                        self.nav_retry = 0
                        self.state = "CALCULATE_GOAL"
                        if not self.completed_trees:
                            self.get_logger().info(f"FIRST TARGET:\nx={tree[0]:.2f}\ny={tree[1]:.2f}\n\nTARGET LOCK:\ntree_id={tree[3]}\ntree_x={tree[0]:.2f}\ntree_y={tree[1]:.2f}")
                        else:
                            self.get_logger().info("FIND TREE:\ntree_x={0:.2f}\ntree_y={1:.2f}\n\nTARGET LOCK:\ntree_id={2}\ntree_x={0:.2f}\ntree_y={1:.2f}".format(tree[0], tree[1], tree[3]))
            elif current_state == "CALCULATE_GOAL":
                self._cached_tree_res = None
                with self.data_lock:
                    if self.locked_target_tree is None:
                        self.state = "FIND_TREE"
                        continue
                    tx, ty, _, tid = self.locked_target_tree

                pose_tuple = self.get_robot_pose_in_map()
                if pose_tuple is None or pose_tuple[0] is None:
                    time.sleep(0.1)
                    continue
                rx, ry, _ = pose_tuple

                actual_dist = math.hypot(tx - rx, ty - ry)
                INSPECTION_DISTANCE = 1.5
                MINIMUM_SAFE_DISTANCE = 1.0

                if actual_dist < MINIMUM_SAFE_DISTANCE:
                    self.get_logger().warn(f"EMERGENCY: Robot too close to tree! dist={actual_dist:.2f} < {MINIMUM_SAFE_DISTANCE}. Triggering recovery.")
                    with self.data_lock:
                        self.state = "RECOVERY"
                        self.recovery_step = 0
                        self.recovery_start = time.time()
                        self.recovery_escape_angle = math.atan2(ry - ty, rx - tx)
                    continue

                dx = tx - rx
                dy = ty - ry
                gx = tx - (dx / actual_dist) * INSPECTION_DISTANCE
                gy = ty - (dy / actual_dist) * INSPECTION_DISTANCE

                theta = math.atan2(ty - ry, tx - rx)

                with self.data_lock:
                    self.nav_debug = {
                        'tree_x': tx, 'tree_y': ty,
                        'start_x': rx, 'start_y': ry, 'start_yaw': self.robot_yaw,
                        'goal_x': gx, 'goal_y': gy, 'stop_dist': INSPECTION_DISTANCE
                    }
                    self.nav_debug_written = False

                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x, pose.pose.position.y = gx, gy
                pose.pose.orientation.z = math.sin(theta/2)
                pose.pose.orientation.w = math.cos(theta/2)

                with self.data_lock:
                    self.goal_pose = pose
                    self.state = "NAVIGATING"
            elif current_state == "NAVIGATING":
                with self.data_lock:
                    nav_active = self.nav_active
                    nav_result = self.nav_result
                    nav_start_time = self.nav_start_time

                if not nav_active and nav_result is None:
                    if abs(self.robot_vx) > 0.05 or abs(self.robot_vy) > 0.05 or abs(self.robot_vtheta) > 0.1:
                        time.sleep(0.1)
                        continue

                    self.get_logger().info(f"NAVIGATION START:\nx={self.goal_pose.pose.position.x:.2f}\ny={self.goal_pose.pose.position.y:.2f}")
                    self.send_nav_goal()

                if nav_active:
                    with self.data_lock:
                        if self.locked_target_tree:
                            tx, ty, _, _ = self.locked_target_tree
                            dist = math.hypot(tx - self.robot_x, ty - self.robot_y)
                            if dist < 0.8:
                                self.get_logger().error(f"EMERGENCY WATCHDOG: dist={dist:.2f} < 0.8m. Canceling Nav2 goal.")
                                self.navigation_recovery()
                                continue

                if nav_active and nav_start_time and (time.time() - nav_start_time > self.nav_timeout):
                    self.navigation_recovery()

                if nav_result in ["failed", "rejected", "canceled"]:
                    self.get_logger().info(f"NAVIGATION RESULT:\nsuccess=False")
                    self.navigation_recovery()
                elif nav_result == "success":
                    self.get_logger().info(f"NAVIGATION RESULT:\nsuccess=True")
                    with self.data_lock:
                        self.nav_result = None
                        self.state = "VERIFY_STOP"
                        self.verify_timer = time.time()

                time.sleep(0.1)
            elif current_state == "VERIFY_STOP":
                if abs(self.robot_vx) < 0.02 and abs(self.robot_vy) < 0.02 and abs(self.robot_vtheta) < 0.05:
                    self.get_logger().info("VERIFY_STOP: Robot is stationary.")
                    with self.data_lock:
                        self.state = "VERIFY_SAFE_DISTANCE"
                elif time.time() - getattr(self, 'verify_timer', time.time()) > 5.0:
                    self.get_logger().warn("VERIFY_STOP: Timeout waiting for zero velocity. Proceeding to safe distance check.")
                    with self.data_lock:
                        self.state = "VERIFY_SAFE_DISTANCE"
                else:
                    time.sleep(0.1)
            elif current_state == "VERIFY_SAFE_DISTANCE":
                pose_tuple = self.get_robot_pose_in_map()
                if pose_tuple is None or pose_tuple[0] is None:
                    time.sleep(0.1)
                    continue
                rx, ry, _ = pose_tuple

                with self.data_lock:
                    if not self.locked_target_tree:
                        self.state = "FIND_TREE"
                        continue
                    tx, ty, _, tid = self.locked_target_tree

                actual_dist = math.hypot(tx - rx, ty - ry)
                if actual_dist < 1.0:
                    self.get_logger().error(f"VERIFY_SAFE_DISTANCE FAILED: dist={actual_dist:.2f} < 1.0m. Triggering recovery.")
                    with self.data_lock:
                        self.state = "RECOVERY"
                        self.recovery_step = 0
                        self.recovery_start = time.time()
                        self.recovery_escape_angle = math.atan2(ry - ty, rx - tx)
                else:
                    self.get_logger().info(f"VERIFY_SAFE_DISTANCE SUCCESS: dist={actual_dist:.2f} >= 1.0m. Proceeding to ALIGNING.")
                    with self.data_lock:
                        self.state = "ALIGNING"
            elif current_state == "ALIGNING":
                with self.data_lock:
                    if not self.locked_target_tree:
                        self.state = "FIND_TREE"
                        continue
                    tx, ty, _, tid = self.locked_target_tree

                pose_tuple = self.get_robot_pose_in_map()
                if pose_tuple is None or pose_tuple[0] is None:
                    time.sleep(0.1)
                    continue

                if time.time() - getattr(self, '_last_tf_success_time', 0.0) > 2.0:
                    self.get_logger().error("TF STALE for > 2.0s in ALIGNING! Aborting.")
                    self.stop_robot()
                    with self.data_lock:
                        self.scan_duration = 0.0
                        self.state = "SAVE_RESULT"
                    time.sleep(0.05)
                    continue
                rx, ry, map_yaw = pose_tuple

                t = Twist()
                target_yaw = math.atan2(ty - ry, tx - rx)
                err_yaw = target_yaw - map_yaw
                while err_yaw > math.pi: err_yaw -= 2*math.pi
                while err_yaw < -math.pi: err_yaw += 2*math.pi

                if abs(err_yaw) > 0.15:
                    t.angular.z = 0.3 if err_yaw > 0 else -0.3
                    self.cmd_pub.publish(t)
                else:
                    self.stop_robot()
                    with self.data_lock:
                        self.state = "VERIFY_SAFE_DISTANCE_AGAIN"

                time.sleep(0.1)
            elif current_state == "VERIFY_SAFE_DISTANCE_AGAIN":
                pose_tuple = self.get_robot_pose_in_map()
                if pose_tuple is None or pose_tuple[0] is None:
                    time.sleep(0.1)
                    continue
                rx, ry, _ = pose_tuple

                with self.data_lock:
                    if not self.locked_target_tree:
                        self.state = "FIND_TREE"
                        continue
                    tx, ty, _, tid = self.locked_target_tree

                actual_dist = math.hypot(tx - rx, ty - ry)
                if actual_dist < 1.0:
                    self.get_logger().error(f"VERIFY_SAFE_DISTANCE_AGAIN FAILED: dist={actual_dist:.2f} < 1.0m after rotation. Triggering recovery.")
                    with self.data_lock:
                        self.state = "RECOVERY"
                        self.recovery_step = 0
                        self.recovery_start = time.time()
                        self.recovery_escape_angle = math.atan2(ry - ty, rx - tx)
                else:
                    self.get_logger().info(f"VERIFY_SAFE_DISTANCE_AGAIN SUCCESS: dist={actual_dist:.2f} >= 1.0m. Proceeding to SCANNING.")
                    with self.data_lock:
                        self._cached_tree_res = None
                        self.scanning = True
                        self.total_rotated_angle = 0.0
                        self.scan_start_time = time.time()
                        self.state = "SCANNING"
                        self.scan_debug_start = {
                            'x': self.robot_x, 'y': self.robot_y, 'yaw': self.robot_yaw,
                            'tx': tx, 'ty': ty, 'tid': tid
                        }

                    target_msg = Point()
                    target_msg.x = float(tx)
                    target_msg.y = float(ty)
                    target_msg.z = float(tid)
                    self.scan_target_pub.publish(target_msg)

                    msg = Bool()
                    msg.data = True
                    self.scan_active_pub.publish(msg)

                    self.get_logger().info(f"SCAN START:\ntree_id={tid}")
            elif current_state == "SCANNING":
                self.stop_robot()

                self.get_robot_pose_in_map()

                if time.time() - getattr(self, '_last_tf_success_time', 0.0) > 2.0:
                    self.get_logger().error("TF STALE for > 2.0s during SCANNING! Aborting.")
                    _sa_msg = Bool()
                    _sa_msg.data = False
                    self.scan_active_pub.publish(_sa_msg)
                    with self.data_lock:
                        self.scanning = False
                        self.state = "SAVE_RESULT"
                    time.sleep(0.05)
                    continue

                scan_time_elapsed = time.time() - self.scan_start_time if getattr(self, 'scan_start_time', None) else 0.0

                if scan_time_elapsed > 10.0:
                    self.get_logger().info("SCAN_COMPLETE: 10 seconds stationary scan finished.")

                    _sa_msg = Bool()
                    _sa_msg.data = False
                    self.scan_active_pub.publish(_sa_msg)

                    with self.data_lock:
                        self.scanning = False
                        self.scan_duration = scan_time_elapsed
                        self.state = "SAVE_RESULT"

                time.sleep(0.1)
            elif current_state == "SAVE_RESULT":
                with self.data_lock:
                    tree = self.locked_target_tree
                    nav_retry = self.nav_retry
                    scan_duration = self.scan_duration

                pose_tuple = self.get_robot_pose_in_map()

                if tree:
                    tx, ty, _, tid = tree

                    if (
                        pose_tuple is not None
                        and pose_tuple[0] is not None
                        and pose_tuple[1] is not None
                    ):
                        rx, ry, _ = pose_tuple
                        inspection_distance = math.hypot(
                            float(rx) - float(tx),
                            float(ry) - float(ty)
                        )
                    else:
                        inspection_distance = None
                        self.get_logger().warn(
                            "SAVE_RESULT: valid map pose unavailable; "
                            "inspection_log.distance will be NULL"
                        )

                    disease_name = None
                    coverage = 0.0
                    confidence = 0.0
                    priority = "None"
                    remedy = "None"

                    try:
                        self.get_logger().info(
                            f"FINAL TREE:\n"
                            f"tree_x={tx:.2f}\n"
                            f"tree_y={ty:.2f}\n"
                            f"status=processing\n"
                            f"confidence={confidence:.2f}\n"
                            f"coverage={coverage:.2f}"
                        )
                        self.get_logger().info("SAVE_RESULT")
                        self.get_logger().info(
                            f"SQLITE INSERT:\n"
                            f"x={tx:.2f}\n"
                            f"y={ty:.2f}\n"
                            f"status=processing\n"
                            f"confidence={confidence:.2f}\n"
                            f"coverage={coverage:.2f}"
                        )
                        conn = sqlite3.connect(self.db_path, timeout=10.0)
                        cursor = conn.cursor()
                        import datetime
                        cursor.execute(
                            """INSERT INTO inspection_log (
                                tree_x, tree_y, distance, scan_time, status,
                                timestamp, disease_name, coverage, confidence,
                                priority, remedy, inspection_id, tree_id,
                                processing_error, cache_hit
                            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, NULL, 0)""",
                            (tx, ty, inspection_distance, float(scan_duration),
                             "processing", datetime.datetime.now(), disease_name,
                             coverage, confidence, priority, remedy,
                             self.current_inspection_id, int(tid))
                        )
                        conn.commit()
                        conn.close()
                        self.get_logger().info("TREE LABELLED processing")
                    except Exception as e:
                        self.get_logger().error(f"DB update err: {e}")

                    with self.data_lock:
                        if not any(math.hypot(tx - dx, ty - dy) < self.COMPLETED_TREE_RADIUS for dx, dy in self.completed_trees):
                            self.completed_trees.append((tx, ty))

                        self.get_logger().info("TARGET UNLOCKED")
                        self.locked_target_tree = None
                        self.current_tree = None
                        self.get_logger().info("NEXT TREE")
                        self.state = "FIND_TREE"
                time.sleep(0.05)
            elif current_state == "RETURN_HOME":
                with self.data_lock:
                    nav_active = self.nav_active
                    nav_result = self.nav_result
                    nav_start_time = self.nav_start_time
                if not nav_active and nav_result is None:
                    self.get_logger().info(f"RETURN HOME:\nx={self.home_x:.2f}\ny={self.home_y:.2f}\nyaw={self.home_yaw:.2f}")
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = self.home_x
                    pose.pose.position.y = self.home_y
                    pose.pose.orientation.z = math.sin(self.home_yaw/2)
                    pose.pose.orientation.w = math.cos(self.home_yaw/2)
                    with self.data_lock:
                        self.goal_pose = pose
                    self.send_nav_goal()
                if nav_active and nav_start_time and (time.time() - nav_start_time > self.nav_timeout):
                    self.stop_robot()
                    self.cancel_navigation()
                    with self.data_lock:
                        self.nav_result = "failed"

                if nav_result in ["failed", "rejected", "canceled"]:
                    with self.data_lock:
                        self.return_home_retry = getattr(self, "return_home_retry", 0) + 1
                        retry = self.return_home_retry
                        self.nav_result = None
                        self.nav_active = False
                    if retry <= 3:
                        self.get_logger().warn(
                            f"RETURN HOME NAVIGATION FAILED: retry {retry}/3 "
                            f"to x={self.home_x:.2f}, y={self.home_y:.2f}"
                        )
                        time.sleep(1.0)
                    else:
                        self.get_logger().error("RETURN HOME FAILED: Navigation failed after 3 attempts.")
                        with self.data_lock:
                            self.running = False
                            self.inspection_finished = True
                            self.state = "IDLE"
                elif nav_result == "success":
                    pose_tuple = self.get_robot_pose_in_map()
                    if pose_tuple and pose_tuple[0] is not None:
                        rx, ry, ryaw = pose_tuple
                        pos_err = math.hypot(rx - self.home_x, ry - self.home_y)
                        yaw_err = ryaw - self.home_yaw
                        while yaw_err > math.pi: yaw_err -= 2*math.pi
                        while yaw_err < -math.pi: yaw_err += 2*math.pi

                        if pos_err < 0.5 and abs(yaw_err) < 0.3:
                            self.get_logger().info(f"RETURN HOME SUCCESS (pos_err={pos_err:.2f}m, yaw_err={abs(yaw_err):.2f}rad)")
                            self.get_logger().info("CAPTURE COMPLETE; DEFERRED PROCESSING NEXT")
                        else:
                            self.get_logger().error(f"RETURN HOME FAILED: Nav succeeded but robot is out of tolerance! (pos_err={pos_err:.2f}m, yaw_err={abs(yaw_err):.2f}rad)")
                    else:
                        self.get_logger().error("RETURN HOME FAILED: Could not fetch final TF.")

                    with self.data_lock:
                        self.nav_result = None
                        self.nav_active = False
                        self.state = "TRIGGER_PROCESSING"
                time.sleep(0.1)
            elif current_state == "TRIGGER_PROCESSING":
                if not self.processing_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().warn(
                        "Waiting for deferred processing service...",
                        throttle_duration_sec=5.0,
                    )
                    time.sleep(0.2)
                    continue
                with self.data_lock:
                    if self.processing_future is None:
                        self.get_logger().info("ROBOT HOME: starting deferred processing")
                        self.processing_future = self.processing_client.call_async(Trigger.Request())
                    self.state = "WAIT_PROCESSING"
            elif current_state == "WAIT_PROCESSING":
                with self.data_lock:
                    future = self.processing_future
                if future is None or not future.done():
                    time.sleep(0.2)
                    continue
                try:
                    result = future.result()
                    if result is None or not result.success:
                        message = result.message if result is not None else "no response"
                        self.get_logger().error(f"DEFERRED PROCESSING FAILED: {message}")
                    else:
                        self.get_logger().info(f"DEFERRED PROCESSING COMPLETE: {result.message}")
                except Exception as exc:
                    self.get_logger().error(f"DEFERRED PROCESSING SERVICE ERROR: {exc}")
                with self.data_lock:
                    self.running = False
                    self.inspection_finished = True
                    self.processing_future = None
                    self.state = "IDLE"
                time.sleep(0.1)
            elif current_state == "RECOVERY":
                with self.data_lock:
                    step = self.recovery_step
                    start = self.recovery_start
                if step == 0:
                    self.get_logger().info("RECOVERY: Backing up")
                    t = Twist()
                    t.linear.x = -0.2
                    self.cmd_pub.publish(t)
                    if time.time() - start > 2.0:
                        self.stop_robot()
                        with self.data_lock:
                            self.recovery_step = 1
                            self.recovery_start = time.time()
                elif step == 1:
                    self.get_logger().info("RECOVERY: Rotating")
                    t = Twist()
                    t.angular.z = 0.4
                    self.cmd_pub.publish(t)
                    if time.time() - start > 2.5:
                        self.stop_robot()
                        with self.data_lock:
                            self.state = "FIND_TREE"
                time.sleep(0.1)
            else:
                time.sleep(0.1)

    def send_nav_goal(self):
        with self.data_lock:
            self.nav_active = True
            self.nav_result = None
            goal_pose = self.goal_pose
            self.nav_goal_seq += 1
            current_seq = self.nav_goal_seq

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.nav_response_callback(f, current_seq))
        with self.data_lock:
            self.nav_start_time = time.time()

    def nav_response_callback(self, future, seq):
        try:
            goal_handle = future.result()
            with self.data_lock:
                if seq != self.nav_goal_seq:
                    return
                if not goal_handle.accepted:
                    self.nav_result = "rejected"
                    self.nav_active = False
                    return
                self.nav_goal_handle = goal_handle
            res_future = goal_handle.get_result_async()
            res_future.add_done_callback(lambda f: self.nav_result_callback(f, seq))
        except Exception as e:
            self.get_logger().error(f"Async response exception when sending Goal: {e}")
            with self.data_lock:
                if seq == self.nav_goal_seq:
                    self.nav_result = "failed"
                    self.nav_active = False

    def nav_result_callback(self, future, seq):
        try:
            with self.data_lock:
                if seq != self.nav_goal_seq:
                    return
            status = future.result().status
            with self.data_lock:
                returning = self.returning_home
                current_state = self.state

            if current_state == "RETURN_HOME":
                with self.data_lock:
                    self.nav_active = False
                    if status == GoalStatus.STATUS_SUCCEEDED:
                        self.nav_result = "success"
                    else:
                        self.nav_result = "failed"
                return

            if status == GoalStatus.STATUS_SUCCEEDED:
                with self.data_lock:
                    self.nav_active = False
                    if returning:
                        self.nav_result = "success"
                    else:
                        self.nav_result = "success"
            else:
                with self.data_lock:
                    if self.running and current_state == "NAVIGATING":
                        self.nav_result = "failed"
                    else:
                        self.nav_result = None
                    self.nav_active = False
        except Exception as e:
            self.get_logger().error(f"Nav2 callback error: {e}")
            with self.data_lock:
                self.nav_active = False
                self.nav_result = None

    def navigation_recovery(self):
        self.stop_robot()
        self.cancel_navigation()

        with self.data_lock:
            self.nav_retry += 1
            self.get_logger().warn(f"Starting smart vector recovery attempt {self.nav_retry}")

            if self.current_tree:
                tx, ty, _, _ = self.current_tree
                rx, ry = self.robot_x, self.robot_y
                self.recovery_escape_angle = math.atan2(ry - ty, rx - tx)
            else:
                self.recovery_escape_angle = self.robot_yaw + math.pi

            self.recovery_step = 0
            self.recovery_start = time.time()
            self.nav_active = False
            self.nav_result = None
            self.state = "RECOVERY"

    def cancel_navigation(self):
        with self.data_lock:
            goal_handle = self.nav_goal_handle
        if goal_handle:
            try:
                goal_handle.cancel_goal_async()
                self.get_logger().info("Nav2 task cancellation request sent")
            except Exception as e:
                self.get_logger().error(f"Exception when cancelling navigation: {e}")
        with self.data_lock:
            self.nav_active = False

    def align_to_target(self):
        with self.data_lock:
            if not self.current_tree: return True
            tx, ty, _, _ = self.current_tree
            rx, ry, ryaw = self.robot_x, self.robot_y, self.robot_yaw

        dist = math.hypot(tx - rx, ty - ry)
        if dist < 0.8:
            self.get_logger().error(f"Safety violation: Distance to tree {dist:.2f}m is unsafe during ALIGN. Aborting alignment!")
            self.stop_robot()
            with self.data_lock:
                self.state = "RECOVERY"
                self.recovery_step = 0
                self.recovery_start = time.time()
                self.recovery_escape_angle = math.atan2(ry - ty, rx - tx)
            return False

        target_yaw = math.atan2(ty - ry, tx - rx)
        err = target_yaw - ryaw
        while err > math.pi: err -= 2.0 * math.pi
        while err < -math.pi: err += 2.0 * math.pi
        if abs(err) < 0.05:
            self.stop_robot()
            return True
        t = Twist()
        t.angular.z = 0.3 if err > 0 else -0.3
        self.cmd_pub.publish(t)
        return False

    def _fetch_tree_status(self, tid: int):
        """
        Synchronously query VisionNode's get_tree_status service for the given
        authoritative tree ID.

        Must be called from the state_machine_loop thread and OUTSIDE any
        data_lock critical section.

        Returns a TreeStatus.Response on success, or None on:
          - service unavailable (waits 2 s)
          - future does not complete within 3 s
          - future raises an exception
          - future result is None
        """
        try:
            temp_node = rclpy.create_node(f'temp_tree_status_client_{tid}_{int(time.time())}')
            client = temp_node.create_client(TreeStatus, 'get_tree_status')

            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    f"[_fetch_tree_status] get_tree_status service unavailable for tid={tid}"
                )
                temp_node.destroy_node()
                return None

            req = TreeStatus.Request()
            req.tree_id = str(tid)
            self.get_logger().info(f"TREE STATUS REQUEST:\ntree_id={tid}")

            future = client.call_async(req)
            rclpy.spin_until_future_complete(temp_node, future, timeout_sec=30.0)

            res = None
            if future.done():
                try:
                    res = future.result()
                    if res is None:
                        self.get_logger().warn(
                            f"[_fetch_tree_status] future.result() is None for tid={tid}"
                        )
                except Exception as e:
                    self.get_logger().error(
                        f"[_fetch_tree_status] future.result() raised for tid={tid}: {e}"
                    )
            else:
                self.get_logger().warn(
                    f"[_fetch_tree_status] future timed out (>3 s) for tid={tid}"
                )

            temp_node.destroy_node()
            return res
        except Exception as e:
            self.get_logger().error(f"[_fetch_tree_status] unexpected error tid={tid}: {e}")
            return None

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = InspectionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

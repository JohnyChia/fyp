#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
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
import json
import struct
import uuid
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
from std_srvs.srv import Trigger
from observation_store import connect
from leaf_deployment_config import load_deployment_config
from multi_view_inspection import (
    MultiViewPolicy,
    NavigationSafetyPolicy,
    recovery_motion_command,
)
from localization_stability import (
    LocalizationGuardPolicy,
    PoseSample,
    StationaryStabilityGate,
    angular_distance,
    evaluate_pose_continuity,
)


MISSION_STATUS_VERSION = 1
MISSION_STATES = frozenset({
    "READY",
    "INSPECTING",
    "RETURNING_HOME",
    "PROCESSING",
    "COMPLETED",
    "CANCELED",
    "FAILED",
})
TERMINAL_MISSION_STATES = frozenset({"COMPLETED", "CANCELED", "FAILED"})


def navigation_watchdog_violation(distance, minimum_distance):
    """Return whether navigation entered the tree safety exclusion radius."""
    return float(distance) < float(minimum_distance)


def recovery_completion_state(resume_state, has_locked_target, has_viewpoints):
    """Resume an interrupted view when its target context is still valid."""
    if resume_state == "CALCULATE_GOAL" and has_locked_target and has_viewpoints:
        return "CALCULATE_GOAL"
    return "FIND_TREE"


def inventory_gate_state(tree_count, wait_started, timeout_sec, now=None):
    """Return ready/waiting/timed_out for the bounded inventory fallback gate."""
    if tree_count > 0:
        return "ready"
    if wait_started is None:
        return "waiting"
    current = time.monotonic() if now is None else now
    return (
        "timed_out"
        if current - wait_started >= max(0.0, timeout_sec)
        else "waiting"
    )


def home_pose_within_tolerance(
    robot_x, robot_y, robot_yaw, home_x, home_y, home_yaw,
    position_tolerance=0.5, yaw_tolerance=0.3,
):
    """Check the final mapped pose rather than trusting Nav2 success alone."""
    position_error = math.hypot(robot_x - home_x, robot_y - home_y)
    yaw_error = (robot_yaw - home_yaw + math.pi) % (2.0 * math.pi) - math.pi
    return (
        position_error < position_tolerance
        and abs(yaw_error) < yaw_tolerance,
        position_error,
        abs(yaw_error),
    )


def build_mission_status_payload(
    state, inspection_id=None, terminal=False, success=False, reason=""
):
    """Build the versioned public mission-status contract."""
    if state not in MISSION_STATES:
        raise ValueError(f"Unsupported mission state: {state}")
    if bool(terminal) != (state in TERMINAL_MISSION_STATES):
        raise ValueError("terminal flag does not match mission state")
    if bool(success) != (state == "COMPLETED"):
        raise ValueError("success flag does not match mission state")
    return {
        "version": MISSION_STATUS_VERSION,
        "state": state,
        "inspection_id": inspection_id or "",
        "terminal": bool(terminal),
        "success": bool(success),
        "reason": reason or "",
    }


def serialize_mission_status(
    state, inspection_id=None, terminal=False, success=False, reason=""
):
    """Serialize mission status deterministically for transition de-duplication."""
    payload = build_mission_status_payload(
        state, inspection_id, terminal, success, reason
    )
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def processing_terminal_status(cancel_requested, processing_succeeded, reason=""):
    """Map deferred-processing outcome to the authoritative terminal state."""
    if not processing_succeeded:
        return "FAILED", True, False, reason or "deferred_processing_failed"
    if cancel_requested:
        return "CANCELED", True, False, reason or "cancel_requested"
    return "COMPLETED", True, True, ""


def mission_status_qos():
    """QoS used by both status publisher and late-joining subscribers."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        depth=1,
    )


class InspectionServer(Node):
    def __init__(self):
        super().__init__("inspection_server")
        # Several state transitions call helpers which also protect shared state.
        # RLock is a defensive backstop; normal paths still release the lock
        # before publishing, cancelling actions, or entering recovery.
        self.data_lock = threading.RLock()
        self.has_authoritative_trees = False
        self.tree_inventory_source = "none"

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
        self.scan_viewpoint_pub = self.create_publisher(String, "/scan_viewpoint", 10)
        inspection_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.inspection_id_pub = self.create_publisher(String, "/inspection_id", inspection_qos)
        self.inspection_status_pub = self.create_publisher(
            String, "/inspection_status", mission_status_qos()
        )
        self._last_mission_status = None
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
        self.robot_vtheta = 0.0
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
        self.declare_parameter('tree_inventory_timeout_sec', 30.0)
        self.declare_parameter('auto_shutdown_after_completion', False)
        self.declare_parameter('leaf_deployment_config_path', '')
        self.declare_parameter('navigation_only_validation', False)
        self.world_path = os.path.expanduser(
            str(self.get_parameter('world_path').value)
        )
        self.tree_inventory_timeout_sec = max(
            0.0,
            float(self.get_parameter('tree_inventory_timeout_sec').value),
        )
        self.auto_shutdown_after_completion = bool(
            self.get_parameter('auto_shutdown_after_completion').value
        )
        self.navigation_only_validation = bool(
            self.get_parameter('navigation_only_validation').value
        )
        deployment_config = load_deployment_config(
            str(self.get_parameter('leaf_deployment_config_path').value)
        )
        self.multi_view_policy = MultiViewPolicy.from_mapping(
            deployment_config['inspection_policy']
        )
        self.navigation_safety = NavigationSafetyPolicy.from_mapping(
            deployment_config['inspection_policy']['navigation_safety']
        )
        self.localization_guard = LocalizationGuardPolicy.from_mapping(
            deployment_config['localization_guard']
        )
        self.localization_gate = StationaryStabilityGate(self.localization_guard)
        self._last_global_pose_sample = None
        self._last_pose_sample_was_stationary = None
        self._localization_failure_reason = None
        self._last_tf_failure_started = None
        self._scan_pose_anchor = None
        self._scan_max_translation_drift = 0.0
        self._scan_max_yaw_drift = 0.0
        self.current_viewpoints = ()
        self.current_viewpoint_index = 0
        self.tree_scan_duration = 0.0
        self.viewpoint_capture_succeeded = False
        self._completion_shutdown_scheduled = False
        self.inventory_wait_started = None
        self.authoritative_trees = []
        self.load_authoritative_trees_from_world()

        self.TREE_MATCH_RADIUS = 1.0
        self.COMPLETED_TREE_RADIUS = 1.0
        self.inspection_finished = False
        self.cancel_requested = False
        self.locked_target_tree = None

        self.recovery_step = 0
        self.recovery_start = 0
        self.recovery_escape_angle = 0.0
        self.recovery_resume_state = "FIND_TREE"

        self.home_x, self.home_y, self.home_yaw = 0.0, 0.0, 0.0

        self.state = "IDLE"
        self.running = False
        self.nav_retry = 0
        self.max_retry = self.navigation_safety.max_recovery_attempts_per_leg

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

        self._publish_mission_status("READY")

        self.state_thread = threading.Thread(target=self.state_machine_loop, daemon=True)
        self.state_thread.start()

        self.get_logger().info("Durian Inspection Server connected.")
        self.get_logger().info(
            "LOCALIZATION AUTHORITY: "
            f"source={self.localization_guard.source} "
            f"global_frame={self.localization_guard.global_frame} "
            f"odom_frame={self.localization_guard.odom_frame} "
            "map_to_odom=static simulation-only"
        )

    def _publish_mission_status(
        self, state, terminal=False, success=False, reason=""
    ):
        """Publish only when the public mission status meaningfully changes."""
        serialized = serialize_mission_status(
            state,
            self.current_inspection_id,
            terminal,
            success,
            reason,
        )
        if serialized == self._last_mission_status:
            return False

        msg = String()
        msg.data = serialized
        self.inspection_status_pub.publish(msg)
        self._last_mission_status = serialized
        self.get_logger().info(f"MISSION STATUS: {serialized}")
        return True

    def _schedule_terminal_shutdown(self, terminal_state):
        """Exit after any terminal result so launch stops every child process."""
        if (
            not self.auto_shutdown_after_completion
            or self._completion_shutdown_scheduled
        ):
            return
        self._completion_shutdown_scheduled = True
        if terminal_state == "COMPLETED":
            self.get_logger().info(
                "ALL DATA STORED: automatic system shutdown in 2 seconds"
            )
        else:
            self.get_logger().info(
                f"MISSION {terminal_state}: automatic system shutdown in 2 seconds"
            )

        def request_shutdown():
            if rclpy.ok():
                self.get_logger().info(
                    f"MISSION {terminal_state}: requesting ROS launch shutdown"
                )
                rclpy.shutdown()

        shutdown_timer = threading.Timer(2.0, request_shutdown)
        shutdown_timer.daemon = True
        shutdown_timer.start()

    def _schedule_completion_shutdown(self):
        """Backward-compatible successful-terminal wrapper."""
        self._schedule_terminal_shutdown("COMPLETED")

    def _mark_incomplete_results_failed(self, reason):
        """Close only this mission's unfinished result rows without deleting evidence."""
        if not self.current_inspection_id:
            return
        try:
            with connect(self.db_path) as conn:
                result_cursor = conn.execute(
                    """UPDATE inspection_log
                       SET status='failed', processing_error=?,
                           completed_at=CURRENT_TIMESTAMP
                       WHERE inspection_id=? AND status='processing'""",
                    (str(reason), self.current_inspection_id),
                )
                observation_cursor = conn.execute(
                    """UPDATE captured_observations
                       SET status='ABORTED_MISSION', processing_error=?
                       WHERE inspection_id=?
                         AND status IN ('CAPTURED', 'PROCESSING')""",
                    (str(reason), self.current_inspection_id),
                )
            if result_cursor.rowcount:
                self.get_logger().warn(
                    f"Marked {result_cursor.rowcount} unfinished result row(s) failed: {reason}"
                )
            if observation_cursor.rowcount:
                self.get_logger().warn(
                    f"Marked {observation_cursor.rowcount} unprocessed observation(s) "
                    f"ABORTED_MISSION: {reason}"
                )
        except Exception as exc:
            self.get_logger().error(
                f"Could not close unfinished result rows after mission failure: {exc}"
            )

    def _finish_failed_mission(self, reason):
        """Publish and persist a fail-closed terminal mission outcome."""
        self.stop_robot()
        self.cancel_navigation()
        with self.data_lock:
            self.running = False
            self.inspection_finished = True
            self.nav_active = False
            self.nav_result = None
            self.state = "IDLE"
        self._mark_incomplete_results_failed(reason)
        self._publish_mission_status(
            "FAILED", terminal=True, success=False, reason=reason
        )
        self._schedule_terminal_shutdown("FAILED")

    def check_nav_server_callback(self):
        if not self.nav_server_connected:
            if self.nav_client.server_is_ready():
                self.nav_server_connected = True
                self.get_logger().info("Nav2 Action Server connected successfully!")
                self.nav_check_timer.cancel()
            else:
                self.get_logger().warn("Waiting for Nav2 Action Server response...", throttle_duration_sec=10)

    def _fail_no_tree_inventory(self):
        """Terminate a started mission when all inventory sources stay empty."""
        self._finish_failed_mission("NO_TREE_INVENTORY")
        self.get_logger().error(
            "INSPECTION FAILED: no world, database, or /static_map "
            "tree inventory before the configured timeout."
        )

    def init_database(self):
        try:
            with connect(self.db_path):
                pass
        except Exception as e:
            self.get_logger().error(f"database initialization failed: {e}")
            raise RuntimeError(
                f"inspection server cannot start without its evidence database: {e}"
            ) from e

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

            self._last_global_pose_sample = None
            self._last_pose_sample_was_stationary = None
            self._localization_failure_reason = None
            self._last_tf_failure_started = None
            self.localization_gate.reset()

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
            prefix = "navigation_validation" if self.navigation_only_validation else "inspection"
            inspection_id = f"{prefix}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"

            with self.data_lock:
                self.current_inspection_id = inspection_id
                self.processing_future = None
                self.inspection_finished = False
                self.cancel_requested = False
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
                self.inventory_wait_started = time.monotonic()
                self.current_viewpoints = ()
                self.current_viewpoint_index = 0
                self.tree_scan_duration = 0.0
                self.viewpoint_capture_succeeded = False

            inspection_msg = String()
            inspection_msg.data = inspection_id
            self.inspection_id_pub.publish(inspection_msg)
            self._publish_mission_status("INSPECTING")
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
                self.cancel_requested = True
                self.returning_home = True
                self.state = "RETURN_HOME"
                self.nav_result = None
                self.nav_active = False
                self.nav_retry = 0
                self.recovery_step = 0
                self.scanning = False
                self.current_tree = None
                self.locked_target_tree = None

            self._publish_mission_status(
                "RETURNING_HOME", reason="cancel_requested"
            )
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

        with self.data_lock:
            if self.trees:
                return

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
                if local_trees and not self.trees:
                    self.trees = list(local_trees)
                    self.has_authoritative_trees = True
                    self.tree_inventory_source = "static_map"

            self.get_logger().info(f"Detected trees in cloud: {len(local_trees)}, Total known trees: {len(self.trees)}")
        except Exception as e:
            self.get_logger().error(f"Point cloud processing error: {e}")

    def get_robot_pose_in_map(self):
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.localization_guard.global_frame, "base_footprint", rclpy.time.Time()
            )
        except Exception as exc:
            now = time.monotonic()
            if self._last_tf_failure_started is None:
                self._last_tf_failure_started = now
            self.get_logger().warn(
                f"LOCALIZATION TF unavailable: {exc}", throttle_duration_sec=2.0
            )
            if (
                self.running
                and now - self._last_tf_failure_started
                >= self.localization_guard.tf_unavailable_timeout_sec
            ):
                self._localization_failure_reason = "LOCALIZATION_TF_UNAVAILABLE"
            return None, None, None

        self._last_tf_failure_started = None
        rx = transform.transform.translation.x
        ry = transform.transform.translation.y
        q = transform.transform.rotation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        # Pose continuity must use the timestamp of the pose source.  This
        # node runs with use_sim_time, so comparing Gazebo pose movement to
        # wall-clock time can falsely classify normal batched simulator
        # updates as instantaneous jumps when WSL/Gazebo is briefly delayed.
        stamp = transform.header.stamp
        source_timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if source_timestamp <= 0.0:
            source_timestamp = self.get_clock().now().nanoseconds * 1e-9
        sample = PoseSample(source_timestamp, rx, ry, yaw)
        linear_speed = math.hypot(self.robot_vx, self.robot_vy)
        result = evaluate_pose_continuity(
            self._last_global_pose_sample,
            sample,
            linear_speed,
            self.robot_vtheta,
            self.localization_guard,
            self._last_pose_sample_was_stationary,
        )
        self.get_logger().debug(
            "LOCALIZATION SAMPLE: "
            f"source={self.localization_guard.source} t={sample.timestamp_sec:.3f} "
            f"x={rx:.3f} y={ry:.3f} yaw={yaw:.3f} "
            f"delta_translation={result.translation_delta_m:.3f} "
            f"delta_yaw={result.yaw_delta_rad:.3f} "
            f"elapsed={result.elapsed_sec:.3f} "
            f"allowed_translation={result.allowed_translation_m:.3f} "
            f"allowed_yaw={result.allowed_yaw_rad:.3f} "
            f"stationary={result.stationary} accepted={result.accepted}"
        )
        if not result.accepted:
            self.get_logger().error(
                "LOCALIZATION_POSE_JUMP: "
                f"source={self.localization_guard.source} "
                f"delta_translation={result.translation_delta_m:.3f}m "
                f"delta_yaw={result.yaw_delta_rad:.3f}rad "
                f"elapsed={result.elapsed_sec:.3f}s "
                f"allowed_translation={result.allowed_translation_m:.3f}m "
                f"allowed_yaw={result.allowed_yaw_rad:.3f}rad "
                f"stationary={result.stationary}"
            )
            self._localization_failure_reason = result.reason
            return None, None, None
        self._last_global_pose_sample = sample
        self._last_pose_sample_was_stationary = result.stationary
        self._last_rx, self._last_ry, self._last_ryaw = rx, ry, yaw
        self._last_tf_success_time = time.monotonic()
        return rx, ry, yaw

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
                    self.tree_inventory_source = "world"

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
                    self.has_authoritative_trees = True
                    self.tree_inventory_source = "database"
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

            # Sample continuously, including while an asynchronous Nav2 goal
            # is active. This prevents a legitimate full navigation leg from
            # appearing as one stationary pose step at arrival and ensures TF
            # loss/jumps are detected during motion rather than at the next
            # state transition.
            self.get_robot_pose_in_map()

            if self._localization_failure_reason:
                reason = self._localization_failure_reason
                self.get_logger().error(
                    f"MISSION FAIL-CLOSED: localization guard reason={reason}"
                )
                self._finish_failed_mission(reason)
                continue

            if current_state != last_logged_state:
                self.get_logger().info(f"State transition: {last_logged_state} → {current_state}")
                last_logged_state = current_state

            if current_state == "FIND_TREE":
                with self.data_lock:
                    tree_count = len(self.trees)
                    wait_started = self.inventory_wait_started
                gate_state = inventory_gate_state(
                    tree_count,
                    wait_started,
                    self.tree_inventory_timeout_sec,
                )
                if gate_state == "waiting":
                    time.sleep(0.5)
                    continue
                if gate_state == "timed_out":
                    self._fail_no_tree_inventory()
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
                            self._publish_mission_status("RETURNING_HOME")
                    else:
                        self.current_tree = tree
                        self.locked_target_tree = tree
                        self.current_viewpoints = self.multi_view_policy.viewpoints(
                            tree[0], tree[1]
                        )
                        self.current_viewpoint_index = 0
                        self.tree_scan_duration = 0.0
                        self.viewpoint_capture_succeeded = False
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
                minimum_safe_distance = self.navigation_safety.goal_min_distance_m

                if actual_dist < minimum_safe_distance:
                    self.get_logger().warn(
                        "EMERGENCY: Robot too close to tree! "
                        f"dist={actual_dist:.2f} < {minimum_safe_distance:.2f}. "
                        "Triggering bounded recovery."
                    )
                    self.navigation_recovery(resume_state="CALCULATE_GOAL")
                    continue

                if not self.current_viewpoints:
                    raise RuntimeError("multi-view policy produced no viewpoints")
                viewpoint = self.current_viewpoints[self.current_viewpoint_index]
                gx, gy, theta = viewpoint.x, viewpoint.y, viewpoint.yaw
                self.get_logger().info(
                    "NAVIGATION LEG: "
                    f"tree_id={tid} viewpoint_id={viewpoint.viewpoint_id} "
                    f"current_pose=({rx:.3f},{ry:.3f}) "
                    f"goal_pose=({gx:.3f},{gy:.3f},{theta:.3f}) "
                    f"goal_distance={math.hypot(gx-rx, gy-ry):.3f} "
                    f"localization_source={self.localization_guard.source} "
                    f"global_frame={self.localization_guard.global_frame} "
                    f"recovery_count={self.nav_retry} localization_guard=good"
                )

                with self.data_lock:
                    self.nav_debug = {
                        'tree_x': tx, 'tree_y': ty,
                        'start_x': rx, 'start_y': ry, 'start_yaw': self.robot_yaw,
                        'goal_x': gx, 'goal_y': gy,
                        'stop_dist': self.multi_view_policy.radius_m,
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

                watchdog_distance = None
                if nav_active:
                    with self.data_lock:
                        if self.locked_target_tree:
                            tx, ty, _, _ = self.locked_target_tree
                            watchdog_distance = math.hypot(
                                tx - self.robot_x, ty - self.robot_y
                            )
                if (
                    watchdog_distance is not None
                    and navigation_watchdog_violation(
                        watchdog_distance,
                        self.navigation_safety.watchdog_min_distance_m,
                    )
                ):
                    self.get_logger().error(
                        "EMERGENCY WATCHDOG: "
                        f"dist={watchdog_distance:.2f} < "
                        f"{self.navigation_safety.watchdog_min_distance_m:.2f}m. "
                        "Canceling Nav2 goal."
                    )
                    self.navigation_recovery(resume_state="CALCULATE_GOAL")
                    continue

                if nav_active and nav_start_time and (time.time() - nav_start_time > self.nav_timeout):
                    self.navigation_recovery(resume_state="CALCULATE_GOAL")

                if nav_result in ["failed", "rejected", "canceled"]:
                    self.get_logger().info(f"NAVIGATION RESULT:\nsuccess=False")
                    self.navigation_recovery(resume_state="CALCULATE_GOAL")
                elif nav_result == "success":
                    self.get_logger().info(f"NAVIGATION RESULT:\nsuccess=True")
                    with self.data_lock:
                        # The retry budget applies to one navigation leg. A
                        # successful leg must not penalize the next viewpoint.
                        self.nav_retry = 0
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
                minimum_safe_distance = self.navigation_safety.goal_min_distance_m
                if actual_dist < minimum_safe_distance:
                    self.get_logger().error(
                        "VERIFY_SAFE_DISTANCE FAILED: "
                        f"dist={actual_dist:.2f} < {minimum_safe_distance:.2f}m. "
                        "Triggering bounded recovery."
                    )
                    self.navigation_recovery(resume_state="CALCULATE_GOAL")
                else:
                    self.get_logger().info(
                        "VERIFY_SAFE_DISTANCE SUCCESS: "
                        f"dist={actual_dist:.2f} >= {minimum_safe_distance:.2f}m. "
                        "Proceeding to ALIGNING."
                    )
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

                if time.monotonic() - getattr(self, '_last_tf_success_time', 0.0) > self.localization_guard.tf_unavailable_timeout_sec:
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
                minimum_safe_distance = self.navigation_safety.goal_min_distance_m
                if actual_dist < minimum_safe_distance:
                    self.get_logger().error(
                        "VERIFY_SAFE_DISTANCE_AGAIN FAILED: "
                        f"dist={actual_dist:.2f} < {minimum_safe_distance:.2f}m "
                        "after rotation. Triggering bounded recovery."
                    )
                    self.navigation_recovery(resume_state="CALCULATE_GOAL")
                else:
                    self.get_logger().info(
                        "VERIFY_SAFE_DISTANCE_AGAIN SUCCESS: "
                        f"dist={actual_dist:.2f} >= {minimum_safe_distance:.2f}m. "
                        "Proceeding to localization settling."
                    )
                    with self.data_lock:
                        self._cached_tree_res = None
                        self.viewpoint_capture_succeeded = False
                        self.localization_gate.reset()
                        self.localization_settle_started = time.monotonic()
                        self.state = "LOCALIZATION_SETTLE"

            elif current_state == "LOCALIZATION_SETTLE":
                self.stop_robot()
                pose_tuple = self.get_robot_pose_in_map()
                if pose_tuple is None or pose_tuple[0] is None:
                    time.sleep(0.1)
                    continue
                sample = self._last_global_pose_sample
                if self.localization_gate.observe(
                    sample,
                    math.hypot(self.robot_vx, self.robot_vy),
                    self.robot_vtheta,
                ):
                    rx, ry, ryaw = pose_tuple
                    with self.data_lock:
                        if not self.locked_target_tree:
                            self.state = "FIND_TREE"
                            continue
                        tx, ty, _, tid = self.locked_target_tree
                        self.scanning = True
                        self.total_rotated_angle = 0.0
                        self.scan_start_time = time.monotonic()
                        self._scan_pose_anchor = sample
                        self._scan_max_translation_drift = 0.0
                        self._scan_max_yaw_drift = 0.0
                        self.state = "SCANNING"

                    target_msg = Point()
                    target_msg.x = float(tx)
                    target_msg.y = float(ty)
                    target_msg.z = float(tid)
                    self.scan_target_pub.publish(target_msg)

                    viewpoint = self.current_viewpoints[self.current_viewpoint_index]
                    viewpoint_msg = String()
                    viewpoint_msg.data = json.dumps({
                        "viewpoint_id": viewpoint.viewpoint_id,
                        "viewpoint_index": viewpoint.index,
                        "camera_goal_pose": {
                            "x": viewpoint.x,
                            "y": viewpoint.y,
                            "yaw": viewpoint.yaw,
                        },
                        "target_tree": {"id": int(tid), "x": float(tx), "y": float(ty)},
                    }, sort_keys=True, separators=(",", ":"))
                    self.scan_viewpoint_pub.publish(viewpoint_msg)

                    msg = Bool()
                    msg.data = True
                    self.scan_active_pub.publish(msg)

                    self.get_logger().info(
                        f"SCAN START:\ntree_id={tid}\n"
                        f"viewpoint_id={viewpoint.viewpoint_id}\n"
                        f"viewpoint_index={viewpoint.index}\n"
                        f"pose=({rx:.3f},{ry:.3f},{ryaw:.3f})\n"
                        "stationary=true localization_guard=stable"
                    )
                elif (
                    time.monotonic() - self.localization_settle_started
                    >= self.localization_guard.settling_timeout_sec
                ):
                    self._localization_failure_reason = "LOCALIZATION_NOT_STABLE"
                time.sleep(0.1)
            elif current_state == "SCANNING":
                self.stop_robot()

                pose_tuple = self.get_robot_pose_in_map()
                if pose_tuple is not None and pose_tuple[0] is not None and self._scan_pose_anchor:
                    rx, ry, ryaw = pose_tuple
                    drift = math.hypot(rx - self._scan_pose_anchor.x, ry - self._scan_pose_anchor.y)
                    yaw_drift = angular_distance(self._scan_pose_anchor.yaw, ryaw)
                    self._scan_max_translation_drift = max(self._scan_max_translation_drift, drift)
                    self._scan_max_yaw_drift = max(self._scan_max_yaw_drift, yaw_drift)
                    if (
                        drift > self.localization_guard.stationary_translation_drift_m
                        or yaw_drift > self.localization_guard.stationary_yaw_drift_rad
                    ):
                        self._localization_failure_reason = "LOCALIZATION_POSE_JUMP"
                        continue

                if time.monotonic() - getattr(self, '_last_tf_success_time', 0.0) > self.localization_guard.tf_unavailable_timeout_sec:
                    self.get_logger().error("TF STALE for > 2.0s during SCANNING! Aborting.")
                    _sa_msg = Bool()
                    _sa_msg.data = False
                    self.scan_active_pub.publish(_sa_msg)
                    with self.data_lock:
                        self.scanning = False
                        self.state = "SAVE_RESULT"
                    time.sleep(0.05)
                    continue

                scan_time_elapsed = time.monotonic() - self.scan_start_time if getattr(self, 'scan_start_time', None) else 0.0

                if scan_time_elapsed > self.multi_view_policy.capture_duration_sec:
                    self.get_logger().info(
                        "SCAN_COMPLETE: configured stationary viewpoint capture finished. "
                        f"pose_drift={self._scan_max_translation_drift:.3f}m "
                        f"yaw_drift={self._scan_max_yaw_drift:.3f}rad"
                    )

                    _sa_msg = Bool()
                    _sa_msg.data = False
                    self.scan_active_pub.publish(_sa_msg)

                    with self.data_lock:
                        self.scanning = False
                        self.scan_duration = scan_time_elapsed
                        self.tree_scan_duration += scan_time_elapsed
                        self.viewpoint_capture_succeeded = True
                        self.state = "SAVE_RESULT"

                time.sleep(0.1)
            elif current_state == "SAVE_RESULT":
                with self.data_lock:
                    tree = self.locked_target_tree
                    nav_retry = self.nav_retry
                    scan_duration = self.tree_scan_duration

                pose_tuple = self.get_robot_pose_in_map()

                if tree:
                    tx, ty, _, tid = tree
                    if not self.viewpoint_capture_succeeded:
                        self.get_logger().error(
                            f"VIEWPOINT FAILED: tree_id={tid} "
                            f"index={self.current_viewpoint_index}; tree not completed"
                        )
                        self._finish_failed_mission(
                            "required_viewpoint_capture_failed"
                        )
                        time.sleep(0.05)
                        continue
                    viewpoint = self.current_viewpoints[self.current_viewpoint_index]
                    self.get_logger().info(
                        f"VIEWPOINT COMPLETE: tree_id={tid} "
                        f"viewpoint_id={viewpoint.viewpoint_id} "
                        f"index={viewpoint.index}"
                    )
                    if self.current_viewpoint_index + 1 < len(self.current_viewpoints):
                        with self.data_lock:
                            self.current_viewpoint_index += 1
                            self.viewpoint_capture_succeeded = False
                            self.nav_result = None
                            self.nav_active = False
                            self.state = "CALCULATE_GOAL"
                        time.sleep(0.05)
                        continue

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

                    if self.navigation_only_validation:
                        self.get_logger().info(
                            f"NAVIGATION VALIDATION TREE COMPLETE: tree_id={tid} "
                            f"views={len(self.current_viewpoints)}; no DB result written"
                        )
                        with self.data_lock:
                            if not any(
                                math.hypot(tx - dx, ty - dy) < self.COMPLETED_TREE_RADIUS
                                for dx, dy in self.completed_trees
                            ):
                                self.completed_trees.append((tx, ty))
                            self.locked_target_tree = None
                            self.current_tree = None
                            self.current_viewpoints = ()
                            self.current_viewpoint_index = 0
                            self.tree_scan_duration = 0.0
                            self.viewpoint_capture_succeeded = False
                            self.state = "FIND_TREE"
                        time.sleep(0.05)
                        continue

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
                        self.current_viewpoints = ()
                        self.current_viewpoint_index = 0
                        self.tree_scan_duration = 0.0
                        self.viewpoint_capture_succeeded = False
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
                        self._finish_failed_mission(
                            "return_home_navigation_failed"
                        )
                elif nav_result == "success":
                    pose_tuple = self.get_robot_pose_in_map()
                    if pose_tuple and pose_tuple[0] is not None:
                        rx, ry, ryaw = pose_tuple
                        in_tolerance, pos_err, yaw_err = home_pose_within_tolerance(
                            rx, ry, ryaw,
                            self.home_x, self.home_y, self.home_yaw,
                        )

                        if in_tolerance:
                            self.get_logger().info(f"RETURN HOME SUCCESS (pos_err={pos_err:.2f}m, yaw_err={yaw_err:.2f}rad)")
                            if self.navigation_only_validation:
                                self.get_logger().info(
                                    "NAVIGATION-ONLY VALIDATION COMPLETE: all trees, "
                                    "viewpoints, and return home passed"
                                )
                            else:
                                self.get_logger().info("CAPTURE COMPLETE; DEFERRED PROCESSING NEXT")
                        else:
                            self.get_logger().error(f"RETURN HOME FAILED: Nav succeeded but robot is out of tolerance! (pos_err={pos_err:.2f}m, yaw_err={yaw_err:.2f}rad)")
                    else:
                        in_tolerance = False
                        self.get_logger().error("RETURN HOME FAILED: Could not fetch final TF.")

                    with self.data_lock:
                        self.nav_active = False
                        if in_tolerance:
                            self.nav_result = None
                            if self.navigation_only_validation:
                                self.running = False
                                self.inspection_finished = True
                                self.state = "IDLE"
                            else:
                                self.state = "TRIGGER_PROCESSING"
                        else:
                            # Feed the existing bounded return-home retry path.
                            self.nav_result = "failed"
                    if in_tolerance and self.navigation_only_validation:
                        self._publish_mission_status(
                            "COMPLETED", terminal=True, success=True
                        )
                        self._schedule_terminal_shutdown("COMPLETED")
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
                self._publish_mission_status("PROCESSING")
            elif current_state == "WAIT_PROCESSING":
                with self.data_lock:
                    future = self.processing_future
                if future is None or not future.done():
                    time.sleep(0.2)
                    continue
                processing_succeeded = False
                failure_reason = ""
                try:
                    result = future.result()
                    if result is None or not result.success:
                        message = result.message if result is not None else "no response"
                        self.get_logger().error(f"DEFERRED PROCESSING FAILED: {message}")
                        failure_reason = message
                    else:
                        self.get_logger().info(f"DEFERRED PROCESSING COMPLETE: {result.message}")
                        processing_succeeded = True
                except Exception as exc:
                    self.get_logger().error(f"DEFERRED PROCESSING SERVICE ERROR: {exc}")
                    failure_reason = str(exc)
                with self.data_lock:
                    cancel_requested = self.cancel_requested
                    self.running = False
                    self.inspection_finished = True
                    self.processing_future = None
                    self.state = "IDLE"
                terminal_state = processing_terminal_status(
                    cancel_requested, processing_succeeded, failure_reason
                )
                self._publish_mission_status(
                    terminal_state[0],
                    terminal=terminal_state[1],
                    success=terminal_state[2],
                    reason=terminal_state[3],
                )
                if terminal_state[0] == "FAILED":
                    self._mark_incomplete_results_failed(terminal_state[3])
                self._schedule_terminal_shutdown(terminal_state[0])
                time.sleep(0.1)
            elif current_state == "RECOVERY":
                with self.data_lock:
                    start = self.recovery_start
                    target = self.locked_target_tree
                    resume_state = self.recovery_resume_state
                pose_tuple = self.get_robot_pose_in_map()
                if target is None or pose_tuple is None or pose_tuple[0] is None:
                    self.stop_robot()
                    with self.data_lock:
                        self.state = "FIND_TREE"
                    time.sleep(0.1)
                    continue

                rx, ry, ryaw = pose_tuple
                tx, ty, _, _ = target
                linear, angular, clearance_reached = recovery_motion_command(
                    rx, ry, ryaw, tx, ty, self.navigation_safety
                )
                if clearance_reached:
                    self.stop_robot()
                    with self.data_lock:
                        self.state = recovery_completion_state(
                            resume_state,
                            self.locked_target_tree is not None,
                            bool(self.current_viewpoints),
                        )
                        resumed_state = self.state
                    self.get_logger().info(
                        "RECOVERY COMPLETE: safe clearance reached; resuming state="
                        f"{resumed_state} viewpoint_index={self.current_viewpoint_index}"
                    )
                elif time.time() - start >= self.navigation_safety.recovery_timeout_sec:
                    self.get_logger().error(
                        "RECOVERY TIMEOUT: safe clearance not reached; "
                        "consuming another bounded attempt"
                    )
                    self.navigation_recovery(resume_state=resume_state)
                else:
                    command = Twist()
                    command.linear.x = linear
                    command.angular.z = angular
                    self.cmd_pub.publish(command)
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

    def navigation_recovery(self, resume_state="CALCULATE_GOAL"):
        self.stop_robot()
        self.cancel_navigation()

        with self.data_lock:
            self.nav_retry += 1
            self.get_logger().warn(f"Starting smart vector recovery attempt {self.nav_retry}")

            if self.nav_retry > self.max_retry:
                exhausted = True
            else:
                exhausted = False

            if not exhausted and self.current_tree:
                tx, ty, _, _ = self.current_tree
                rx, ry = self.robot_x, self.robot_y
                self.recovery_escape_angle = math.atan2(ry - ty, rx - tx)
            elif not exhausted:
                self.recovery_escape_angle = self.robot_yaw + math.pi

            if not exhausted:
                self.recovery_step = 0
                self.recovery_start = time.time()
                self.recovery_resume_state = resume_state
                self.nav_active = False
                self.nav_result = None
                self.state = "RECOVERY"

        if exhausted:
            self.get_logger().error(
                f"NAVIGATION FAILED: recovery attempts exceeded {self.max_retry}"
            )
            self._finish_failed_mission("navigation_retry_exhausted")

    def cancel_navigation(self):
        with self.data_lock:
            goal_handle = self.nav_goal_handle
            # Invalidate every callback belonging to the canceled goal. Without
            # this, a late success/cancel result can overwrite recovery state.
            self.nav_goal_seq += 1
            self.nav_goal_handle = None
            self.nav_active = False
            self.nav_result = None
        if goal_handle:
            try:
                goal_handle.cancel_goal_async()
                self.get_logger().info("Nav2 task cancellation request sent")
            except Exception as e:
                self.get_logger().error(f"Exception when cancelling navigation: {e}")

    def align_to_target(self):
        with self.data_lock:
            if not self.current_tree: return True
            tx, ty, _, _ = self.current_tree
            rx, ry, ryaw = self.robot_x, self.robot_y, self.robot_yaw

        dist = math.hypot(tx - rx, ty - ry)
        if dist < self.navigation_safety.watchdog_min_distance_m:
            self.get_logger().error(f"Safety violation: Distance to tree {dist:.2f}m is unsafe during ALIGN. Aborting alignment!")
            self.stop_robot()
            self.navigation_recovery(resume_state="CALCULATE_GOAL")
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from nav2_msgs.action import NavigateToPose
from durian_message.srv import InspectionControl
from action_msgs.msg import GoalStatus
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
import numpy as np
import math
import time
import threading
import sqlite3
import datetime
import struct

class InspectionServer(Node):
    def __init__(self):
        super().__init__("inspection_server")
        
        self.data_lock = threading.Lock()
        self.cb_group = ReentrantCallbackGroup()

        self.service = self.create_service(
            InspectionControl,
            "control_inspection",
            self.command_callback,
            callback_group=self.cb_group
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2, "/static_map", self.cloud_callback, 10, callback_group=self.cb_group
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10, callback_group=self.cb_group
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.nav_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose", callback_group=self.cb_group
        )

        self.nav_server_connected = False
        self.nav_check_timer = self.create_timer(1.0, self.check_nav_server_callback, callback_group=self.cb_group)

        self.db_path = "/home/johny/durian_ws/durian_inspection.db"
        self.init_database()

        self.robot_x, self.robot_y, self.robot_yaw = 0.0, 0.0, 0.0
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
        self.inspection_finished = False
        
        # 智能脱困相关变量
        self.recovery_step = 0
        self.recovery_start = 0
        self.recovery_escape_angle = 0.0  
        
        self.home_x, self.home_y, self.home_yaw = 0.0, 0.0, 0.0

        self.state = "IDLE"
        self.running = False
        self.nav_retry = 0
        self.max_retry = 3  
        
        self.nav_goal_handle = None
        self.nav_active = False
        self.nav_result = None
        self.nav_start_time = None
        self.nav_timeout = 60.0
        self.goal_pose = None

        self.load_completed()

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
            conn = sqlite3.connect(self.db_path, timeout=10.0)
            cursor = conn.cursor()
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS inspection_log(
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    tree_x REAL, tree_y REAL, distance REAL,
                    scan_time REAL, status TEXT, retry INTEGER, timestamp DATETIME
                )
            """)
            conn.commit()
            conn.close()
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
            self.get_logger().info("Received GUI [START] command - Beginning inspection")
            with self.data_lock:
                self.inspection_finished = False
                self.home_x, self.home_y, self.home_yaw = self.robot_x, self.robot_y, self.robot_yaw
                self.initial_inspect_x = self.robot_x
                self.initial_inspect_y = self.robot_y
                self.running = True
                self.state = "FIND_TREE"
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
                
            self.send_nav_goal()
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
                rgb_int = struct.unpack('I', struct.pack('f', rgb))[0]
                r = (rgb_int >> 16) & 255
                g = (rgb_int >> 8) & 255
                b = rgb_int & 255

                if r < 60 and g < 60 and b < 60:
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

            clustering = DBSCAN(eps=0.5, min_samples=10).fit(filtered[:, :2])
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
                local_trees.append((cx, cy, cz))

            with self.data_lock:
                self.trees = local_trees

            self.get_logger().info(f"Detected trees: {len(local_trees)}")

        except Exception as e:
            self.get_logger().error(f"Point cloud processing error: {e}")

    def find_nearest_tree(self):
        nearest, min_dist = None, float("inf")
        with self.data_lock:
            current_trees = list(self.trees)
            done_trees = list(self.completed_trees)
            rx, ry = self.robot_x, self.robot_y

        for tree in current_trees:
            tx, ty, _ = tree
            if any(math.sqrt((tx - dx)**2 + (ty - dy)**2) < 1.0 for dx, dy in done_trees):
                continue
            
            dist = math.sqrt((rx - tx)**2 + (ry - ty)**2)
            if dist < min_dist:
                min_dist = dist
                nearest = tree
        return nearest, min_dist

    def state_machine_loop(self):
        while rclpy.ok():
            with self.data_lock:
                running = self.running
                current_state = self.state

            if not running:
                time.sleep(0.2)
                continue

            if current_state == "FIND_TREE":
                tree, dist = self.find_nearest_tree()
                with self.data_lock:
                    if tree is None:
                        self.get_logger().info("All target trees in the area have been inspected! Preparing to return home.")
                        self.returning_home = True
                        self.state = "RETURN_HOME"
                    else:
                        self.current_tree = tree
                        self.nav_retry = 0  
                        self.state = "CALCULATE_GOAL"
                time.sleep(0.5)

            elif current_state == "CALCULATE_GOAL":
                with self.data_lock:
                    if self.current_tree is None:
                        self.state = "FIND_TREE"
                        continue
                    tx, ty, _ = self.current_tree
                    rx, ry = self.robot_x, self.robot_y

                theta = math.atan2(ty - ry, tx - rx)
                STOP_DISTANCE = 1.5
                gx = tx - math.cos(theta) * STOP_DISTANCE
                gy = ty - math.sin(theta) * STOP_DISTANCE
                
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
                    self.send_nav_goal()

                if nav_active and nav_start_time and (time.time() - nav_start_time > self.nav_timeout):
                    self.get_logger().warn("Navigation timeout, triggering recovery mechanism!")
                    self.navigation_recovery()

                if nav_result == "failed":
                    self.get_logger().warn("Navigation result feedback failed, triggering recovery mechanism!")
                    self.navigation_recovery()

                time.sleep(0.1)

            elif current_state == "RECOVERY":
                t = Twist()
                with self.data_lock:
                    elapsed = time.time() - self.recovery_start
                    step = self.recovery_step
                    escape_yaw = self.recovery_escape_angle
                    current_yaw = self.robot_yaw

                if step == 0:
                    self.get_logger().info("Recovery: Moving backward away from tree based on vector")
                    t.linear.x = -0.15
                    
                    err_yaw = escape_yaw - current_yaw
                    while err_yaw > math.pi: err_yaw -= 2.0 * math.pi
                    while err_yaw < -math.pi: err_yaw += 2.0 * math.pi
                    t.angular.z = 0.3 if err_yaw > 0 else -0.3

                    if elapsed > 2.5:
                        with self.data_lock:
                            self.recovery_step = 1
                            self.recovery_start = time.time()

                elif step == 1:
                    self.get_logger().info("Recovery: Adjusting lateral clearance")
                    t.linear.x = -0.10
                    t.angular.z = 0.0
                    if elapsed > 1.5:
                        self.stop_robot()
                        with self.data_lock:
                            self.nav_active = False
                            self.nav_result = None
                            
                            if self.nav_retry > self.max_retry:
                                self.get_logger().warn("Recovery failed too many times, skipping this tree and searching for next tree.")
                                self.current_tree = None
                                self.state = "FIND_TREE"
                            else:
                                self.get_logger().info(f"Smart recovery finished, re-calculating path to goal (Retry: {self.nav_retry})")
                                self.state = "CALCULATE_GOAL"

                self.cmd_pub.publish(t)
                time.sleep(0.05)

            elif current_state == "ALIGN":
                with self.data_lock:
                    if not self.current_tree:
                        self.state = "FIND_TREE"
                        continue
                    tx, ty, _ = self.current_tree
                    rx, ry, ryaw = self.robot_x, self.robot_y, self.robot_yaw

                actual_dist = math.sqrt((rx - tx)**2 + (ry - ty)**2)
                TARGET_DISTANCE = 1.5
                DIST_TOL = 0.15

                t = Twist()
                if actual_dist > TARGET_DISTANCE + DIST_TOL:
                    target_yaw = math.atan2(ty - ry, tx - rx)
                    err_yaw = target_yaw - ryaw
                    while err_yaw > math.pi: err_yaw -= 2*math.pi
                    while err_yaw < -math.pi: err_yaw += 2*math.pi

                    if abs(err_yaw) > 0.1:
                        t.angular.z = 0.2 if err_yaw > 0 else -0.2
                        t.linear.x = 0.0
                    else:
                        t.linear.x = 0.03
                        t.angular.z = 0.0

                    self.cmd_pub.publish(t)
                    time.sleep(0.05)
                    continue
                
                if self.align_to_target():
                    with self.data_lock:
                        self.state = "SCANNING"
                        self.total_rotated_angle = 0.0
                        self.scan_start_time = time.time()
                        self.scanning = True
                time.sleep(0.05)

            elif current_state == "SCANNING":
                t = Twist()
                t.linear.x = 0.0  
                t.angular.z = 0.3
                self.cmd_pub.publish(t)
                
                with self.data_lock:
                    scan_time_elapsed = time.time() - self.scan_start_time if self.scan_start_time else 0.0
                    total_rotated = self.total_rotated_angle

                if total_rotated >= (2 * math.pi - 0.2) or scan_time_elapsed > 60.0:
                    self.stop_robot()
                    with self.data_lock:
                        self.scanning = False
                        self.scan_duration = scan_time_elapsed
                        self.state = "SAVE_RESULT"
                time.sleep(0.05)

            elif current_state == "SAVE_RESULT":
                with self.data_lock:
                    tree = self.current_tree
                    nav_retry = self.nav_retry
                    scan_duration = self.scan_duration
                    rx, ry = self.robot_x, self.robot_y

                if tree:
                    tx, ty, _ = tree
                    try:
                        conn = sqlite3.connect(self.db_path, timeout=10.0)
                        cursor = conn.cursor()
                        cursor.execute(
                            "INSERT INTO inspection_log (tree_x, tree_y, distance, scan_time, status, retry, timestamp) VALUES (?, ?, ?, ?, ?, ?, ?)",
                            (tx, ty, math.sqrt((rx - tx)**2 + (ry - ty)**2), scan_duration, "completed", nav_retry, datetime.datetime.now())
                        )
                        conn.commit()
                        conn.close()
                        self.get_logger().info(f"Successfully wrote completion marker to database: Tree ({tx:.2f}, {ty:.2f})")
                    except Exception as e:
                        self.get_logger().error(f"Failed to save to database: {e}")

                    with self.data_lock:
                        self.completed_trees.append((tx, ty))
                        self.scanning = False
                        self.current_tree = None
                        self.nav_result = None
                        self.nav_active = False

                    next_tree, _ = self.find_nearest_tree()
                    with self.data_lock:
                        if next_tree is not None:
                            self.get_logger().info(f"Target switched: Moving to next tree ({next_tree[0]:.2f}, {next_tree[1]:.2f})")
                            self.state = "FIND_TREE"
                        else:
                            self.get_logger().info("All trees inspection completed, starting to return to initial position")
                            self.returning_home = True
                            self.state = "RETURN_HOME"
                time.sleep(1.0)

            elif current_state == "RETURN_HOME":
                with self.data_lock:
                    nav_active = self.nav_active
                    nav_result = self.nav_result
                    hx, hy, hyaw = self.home_x, self.home_y, self.home_yaw

                if not nav_active and nav_result is None:
                    self.get_logger().info(f"Driving towards home point: ({hx:.2f}, {hy:.2f})")
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x, pose.pose.position.y = hx, hy
                    pose.pose.orientation.z = math.sin(hyaw/2)
                    pose.pose.orientation.w = math.cos(hyaw/2)
                    
                    with self.data_lock:
                        self.goal_pose = pose
                        self.returning_home = True
                    self.send_nav_goal()
                
                with self.data_lock:
                    current_nav_result = self.nav_result

                if current_nav_result == "success":
                    with self.data_lock:
                        self.inspection_finished = True
                        self.current_tree = None
                        self.nav_result = None
                        self.nav_retry = 0
                        self.running = False
                        self.state = "IDLE"
                        self.returning_home = False
                    self.stop_robot()
                    self.get_logger().info("Robot has safely returned to the initial point, state switched back to IDLE.")
                elif current_nav_result in ["failed", "rejected"]:
                    self.get_logger().warn("Return navigation blocked, attempting to resend return goal...")
                    with self.data_lock:
                        self.nav_result = None   
                        self.nav_active = False
                time.sleep(0.1)

    def send_nav_goal(self):
        with self.data_lock:
            self.nav_active = True
            self.nav_result = None
            goal_pose = self.goal_pose

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.nav_response_callback)
        with self.data_lock:
            self.nav_start_time = time.time()

    def nav_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                with self.data_lock:
                    self.nav_result = "rejected"
                    self.nav_active = False
                return
            with self.data_lock:
                self.nav_goal_handle = goal_handle
            res_future = goal_handle.get_result_async()
            res_future.add_done_callback(self.nav_result_callback)
        except Exception as e:
            self.get_logger().error(f"Async response exception when sending Goal: {e}")
            with self.data_lock:
                self.nav_result = "failed"
                self.nav_active = False

    def nav_result_callback(self, future):
        try:
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
                        self.nav_result = None
                        self.state = "ALIGN"  # 导航成功后直接进入对准状态，替代原先无逻辑的 APPROACH_TREE
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
                tx, ty, _ = self.current_tree
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
            tx, ty, _ = self.current_tree
            rx, ry, ryaw = self.robot_x, self.robot_y, self.robot_yaw

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
        rclpy.shutdown()

if __name__ == "__main__":
    main()
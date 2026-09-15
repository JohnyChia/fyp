#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
import time
import json
import datetime
import tf2_ros
import uuid
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
import math
import hashlib
import threading
from gazebo_msgs.srv import SetEntityState

CLASS_MAP = {
    'algal': 0, 'blight': 1, 'colletotrichum': 2, 'healthy': 3,
    'phomopsis': 4, 'rhizoctonia': 5
}
DEPTH_OCCLUSION_TOLERANCE_M = 0.2
ALPHA_SAMPLE_STEP = 2


def resolve_material_class(mat_name):
    if not mat_name:
        return "UNKNOWN"
    parts = mat_name.split('_')
    if len(parts) >= 2 and parts[0] == 'leaf':
        candidate = parts[1]
        if candidate in CLASS_MAP:
            return candidate
    return "UNKNOWN"


def get_alpha_samples(texture_path):
    img = cv2.imread(texture_path, cv2.IMREAD_UNCHANGED)
    if img is None or img.shape[2] < 4:
        return []
    alpha = img[:,:,3]
    y_indices, x_indices = np.where(alpha > 0)
    mask = (x_indices % ALPHA_SAMPLE_STEP == 0) & (y_indices % ALPHA_SAMPLE_STEP == 0)
    u_vals = x_indices[mask]
    v_vals = y_indices[mask]
    h, w = alpha.shape
    nx = (u_vals / w) - 0.5
    ny = 0.5 - (v_vals / h)
    return list(zip(nx, ny))


class GazeboCollector(Node):
    def __init__(self):
        super().__init__('gazebo_collector')

        self.declare_parameter('output_dir', '/home/johny/durian_ws/experiment/outputs/current_world_gazebo/runs')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('world_path', '/home/johny/durian_ws/experiment/domain_gap/b0/durian_farm_scale_1x.world')
        self.declare_parameter('texture_path', '/home/johny/durian_ws/src/durian_inspection_pkg/config/materials/textures')

        self.declare_parameter('radii', [0.8, 1.1, 1.4])
        self.declare_parameter('num_angles', 8)
        self.declare_parameter('robot_z', 0.1)
        self.declare_parameter('settle_time', 2.0)
        self.declare_parameter('min_frame_interval', 0.5)
        self.declare_parameter('rgb_depth_sync_tolerance', 0.02)
        self.declare_parameter('pose_stability_tolerance', 0.01)
        self.declare_parameter('min_visible_leaves', 3)
        self.declare_parameter('min_bbox_px', 16)
        self.declare_parameter('preferred_bbox_px', 32)

        out_dir_base = self.get_parameter('output_dir').value
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.world_path = self.get_parameter('world_path').value
        self.texture_path = self.get_parameter('texture_path').value

        self.radii = self.get_parameter('radii').value
        self.num_angles = self.get_parameter('num_angles').value
        self.robot_z = self.get_parameter('robot_z').value
        self.settle_time = self.get_parameter('settle_time').value
        self.min_frame_interval = self.get_parameter('min_frame_interval').value
        self.rgb_depth_sync_tolerance = self.get_parameter('rgb_depth_sync_tolerance').value
        self.pose_stability_tolerance = self.get_parameter('pose_stability_tolerance').value
        self.min_visible_leaves = self.get_parameter('min_visible_leaves').value
        self.min_bbox_px = self.get_parameter('min_bbox_px').value
        self.preferred_bbox_px = self.get_parameter('preferred_bbox_px').value

        run_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(out_dir_base, f"collection_{run_time}")
        self.rgb_dir = os.path.join(self.run_dir, "images")
        self.depth_dir = os.path.join(self.run_dir, "depth")
        self.labels_dir = os.path.join(self.run_dir, "labels")
        self.meta_dir = os.path.join(self.run_dir, "metadata")

        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        os.makedirs(self.labels_dir, exist_ok=True)
        os.makedirs(self.meta_dir, exist_ok=True)

        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.set_entity_client = self.create_client(SetEntityState, '/set_entity_state')

        self.inspection_run_id = str(uuid.uuid4())

        self.latest_rgb = None
        self.latest_depth = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.img_w = None
        self.img_h = None
        self.camera_frame_id = None

        self.stats = {
            'total_viewpoints_attempted': 0,
            'accepted_frames': 0,
            'rejected_frames': 0,
            'duplicate_frames': 0,
            'frames_without_gt': 0,
            'total_gt_labels': 0,
            'trees_discovered': set(),
            'class_counts': {c: 0 for c in CLASS_MAP.keys()},
            'gt_projection_failures': 0,
            'tf_failures': 0,
            'camera_info_failures': 0,
            'frames_per_tree': {},
            'gt_labels_per_tree': {},
            'bbox_widths': [],
            'bbox_heights': []
        }

        self.seen_hashes = set()

        self.create_subscription(Image, self.image_topic, self.rgb_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.cam_info_cb, 10)

        self.trees = self.parse_world(self.world_path)
        self.tex_cache = {}

        self.collection_complete = False
        self.create_timer(1.0, self.check_shutdown_status)

        self.get_logger().info(f"Gazebo Collector Started. Output: {self.run_dir}")
        self.get_logger().info(f"Loaded {len(self.trees)} trees from world.")

        self.collection_thread = threading.Thread(target=self.run_auto_collection)
        self.collection_thread.start()

    def check_shutdown_status(self):
        if self.collection_complete:
            self.on_shutdown()
            raise SystemExit

    def parse_world(self, world_path):
        trees = {}
        if not world_path or not os.path.exists(world_path):
            self.get_logger().error(f"World path not found: {world_path}")
            return trees

        tree = ET.parse(world_path)
        root = tree.getroot()
        world = root.find('world')

        for model in world.findall('model'):
            name = model.get('name')
            if not name.startswith('durian_tree'):
                continue

            pose_node = model.find('pose')
            if pose_node is None: continue
            px, py, pz, r, p, y = map(float, pose_node.text.split())
            T_w_t = np.eye(4)
            T_w_t[:3, :3] = R.from_euler('xyz', [r, p, y]).as_matrix()
            T_w_t[:3, 3] = [px, py, pz]

            leaves = []
            for link in model.findall('link'):
                lpose_node = link.find('pose')
                T_m_l = np.eye(4)
                if lpose_node is not None:
                    lx, ly, lz, lr, lp, l_yaw = map(float, lpose_node.text.split())
                    T_m_l[:3, :3] = R.from_euler('xyz', [lr, lp, l_yaw]).as_matrix()
                    T_m_l[:3, 3] = [lx, ly, lz]

                for visual in link.findall('visual'):
                    vname = visual.get('name')
                    if '_leaf_' not in vname:
                        continue

                    mat_name = ""
                    mat = visual.find('material')
                    if mat is not None:
                        script = mat.find('script')
                        if script is not None:
                            name_node = script.find('name')
                            if name_node is not None and name_node.text:
                                mat_name = name_node.text

                    cls_name = resolve_material_class(mat_name)

                    vpose = visual.find('pose')
                    T_l_v = np.eye(4)
                    if vpose is not None:
                        vx, vy, vz, vr, vp, v_yaw = map(float, vpose.text.split())
                        T_l_v[:3, :3] = R.from_euler('xyz', [vr, vp, v_yaw]).as_matrix()
                        T_l_v[:3, 3] = [vx, vy, vz]

                    T_m_v = T_m_l @ T_l_v

                    geom = visual.find('geometry')
                    plane = geom.find('plane')
                    size = plane.find('size').text
                    w_leaf, h_leaf = map(float, size.split())

                    if cls_name != "UNKNOWN":
                        texture_file = os.path.join(self.texture_path, f"leaf_{cls_name}.png")
                        if not os.path.exists(texture_file):
                            texture_file = os.path.join(self.texture_path, f"leaf_{cls_name}_v1.png")
                    else:
                        texture_file = None

                    leaves.append({
                        'id': f"{name}/{vname}",
                        'T_t_l': T_m_v,
                        'w': w_leaf,
                        'h': h_leaf,
                        'class': CLASS_MAP[cls_name] if cls_name != "UNKNOWN" else "UNKNOWN",
                        'class_name': cls_name,
                        'mat_name': mat_name,
                        'tex': texture_file
                    })

            trees[name] = {'T_w_t': T_w_t, 'leaves': leaves}
            self.stats['trees_discovered'].add(name)

        return trees

    def rgb_cb(self, msg):
        self.latest_rgb = msg

    def depth_cb(self, msg):
        self.latest_depth = msg

    def cam_info_cb(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.img_w = msg.width
            self.img_h = msg.height
            self.camera_frame_id = msg.header.frame_id

    def teleport_robot(self, x, y, z, yaw):
        while not self.set_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /set_entity_state service...')

        req = SetEntityState.Request()
        req.state.name = 'durian_bot'
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)

        q = R.from_euler('xyz', [0, 0, yaw]).as_quat()
        req.state.pose.orientation.x = float(q[0])
        req.state.pose.orientation.y = float(q[1])
        req.state.pose.orientation.z = float(q[2])
        req.state.pose.orientation.w = float(q[3])

        future = self.set_entity_client.call_async(req)

        while not future.done():
            time.sleep(0.1)
        return future.result()

    def run_auto_collection(self):
        self.get_logger().info("Waiting for camera_info...")
        while self.fx is None:
            time.sleep(0.5)

        for tree_name, tree in self.trees.items():
            tx = tree['T_w_t'][0, 3]
            ty = tree['T_w_t'][1, 3]
            self.get_logger().info(f"Targeting {tree_name} at ({tx:.2f}, {ty:.2f})")

            for r in self.radii:
                for angle_idx in range(self.num_angles):
                    angle = angle_idx * (2 * math.pi / self.num_angles)
                    rx = tx + r * math.cos(angle)
                    ry = ty + r * math.sin(angle)
                    yaw = math.atan2(ty - ry, tx - rx)

                    vp_id = f"vp_{tree_name}_r{r:.2f}_a{angle_idx:02d}"
                    self.stats['total_viewpoints_attempted'] += 1

                    diag = {
                        "teleport_target": (rx, ry),
                        "actual_pose": None,
                        "tf_timestamp": None,
                        "pose_stability_error": -1.0,
                        "stability_sample_count": 0,
                        "rgb_timestamp": None,
                        "depth_timestamp": None,
                        "rgb_depth_delta": None,
                        "result": "UNKNOWN"
                    }

                    self.teleport_robot(rx, ry, self.robot_z, yaw)
                    time.sleep(self.settle_time)

                    stable = False
                    start_time = time.time()
                    consecutive_valid = 0
                    last_pose = None
                    last_tf_stamp = None
                    err = -1.0

                    while time.time() - start_time < 5.0:
                        try:
                            t_base = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())

                            ex = t_base.transform.translation.x
                            ey = t_base.transform.translation.y
                            tf_stamp = t_base.header.stamp.sec + t_base.header.stamp.nanosec * 1e-9

                            if last_tf_stamp is not None and tf_stamp <= last_tf_stamp:
                                time.sleep(0.1)
                                continue

                            if last_pose is not None:
                                err = math.hypot(ex - last_pose[0], ey - last_pose[1])
                                if err <= self.pose_stability_tolerance:
                                    consecutive_valid += 1
                                else:
                                    consecutive_valid = 0
                            else:
                                consecutive_valid = 1

                            last_pose = (ex, ey)
                            last_tf_stamp = tf_stamp

                            diag["actual_pose"] = (ex, ey)
                            diag["tf_timestamp"] = tf_stamp
                            diag["pose_stability_error"] = err
                            diag["stability_sample_count"] = consecutive_valid

                            if consecutive_valid >= 3:
                                stable = True
                                break
                        except tf2_ros.ExtrapolationException:
                            diag["result"] = "TF_EXTRAPOLATION_WAIT"
                        except Exception as e:
                            pass

                        time.sleep(0.2)

                    if not stable:
                        if diag["result"] == "UNKNOWN":
                            diag["result"] = "POSE_UNSTABLE" if last_pose else "TF_TIMEOUT"
                        self.get_logger().warn(f"[{vp_id}] {diag['result']}. Diag: {diag}")
                        self.stats['rejected_frames'] += 1
                        continue

                    synced = False
                    start_time = time.time()
                    rgb_msg = None
                    depth_msg = None
                    while time.time() - start_time < 5.0:
                        if self.latest_rgb and self.latest_depth:
                            t_r = self.latest_rgb.header.stamp.sec + self.latest_rgb.header.stamp.nanosec * 1e-9
                            t_d = self.latest_depth.header.stamp.sec + self.latest_depth.header.stamp.nanosec * 1e-9
                            delta = abs(t_r - t_d)

                            diag["rgb_timestamp"] = t_r
                            diag["depth_timestamp"] = t_d
                            diag["rgb_depth_delta"] = delta

                            if delta < self.rgb_depth_sync_tolerance:
                                rgb_msg = self.latest_rgb
                                depth_msg = self.latest_depth
                                synced = True
                                break
                        time.sleep(0.1)

                    if not synced:
                        diag["result"] = "RGB_DEPTH_SYNC_TIMEOUT"
                        self.get_logger().warn(f"[{vp_id}] {diag['result']}. Diag: {diag}")
                        self.stats['rejected_frames'] += 1
                        continue

                    res_status = self.process_viewpoint(vp_id, tree_name, rgb_msg, depth_msg, rx, ry, yaw)
                    diag["result"] = res_status
                    self.get_logger().info(f"[{vp_id}] {res_status}. Diag: {diag}")

        self.collection_complete = True

    def process_viewpoint(self, vp_id, tree_name, rgb_msg, depth_msg, rx, ry, yaw):
        time_stamp = rgb_msg.header.stamp
        try:
            t_base = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            t_cam = self.tf_buffer.lookup_transform("map", self.camera_frame_id, rclpy.time.Time())
        except Exception as e:
            self.stats['tf_failures'] += 1
            self.stats['rejected_frames'] += 1
            return "TF_FAILURE_PROJECTION"

        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            img_hash = hashlib.md5(cv_rgb.tobytes()).hexdigest()
            if img_hash in self.seen_hashes:
                self.stats['duplicate_frames'] += 1
                self.stats['rejected_frames'] += 1
                return "REJECTED_DUPLICATE"

            T_w_cam = np.eye(4)
            T_w_cam[:3, :3] = R.from_quat([t_cam.transform.rotation.x, t_cam.transform.rotation.y, t_cam.transform.rotation.z, t_cam.transform.rotation.w]).as_matrix()
            T_w_cam[:3, 3] = [t_cam.transform.translation.x, t_cam.transform.translation.y, t_cam.transform.translation.z]
            T_cam_w = np.linalg.inv(T_w_cam)

            labels = []
            frame_gt_count = 0
            widths = []
            heights = []

            tree = self.trees[tree_name]
            T_w_t = tree['T_w_t']

            for leaf in tree['leaves']:
                if leaf['class'] == "UNKNOWN":
                    continue

                T_w_l = T_w_t @ leaf['T_t_l']
                T_c_l = T_cam_w @ T_w_l

                if leaf['tex'] not in self.tex_cache:
                    self.tex_cache[leaf['tex']] = get_alpha_samples(leaf['tex'])
                alpha_pts = self.tex_cache[leaf['tex']]

                w = leaf['w']
                h = leaf['h']

                visible_pts = []
                for nx, ny in alpha_pts:
                    lx = nx * w
                    ly = ny * h
                    pt_l = np.array([lx, ly, 0, 1.0])
                    pt_c = T_c_l @ pt_l
                    if pt_c[2] <= 0: continue

                    u = int((self.fx * pt_c[0] / pt_c[2]) + self.cx)
                    v = int((self.fy * pt_c[1] / pt_c[2]) + self.cy)

                    if 0 <= u < self.img_w and 0 <= v < self.img_h:
                        d_measured = cv_depth[v, u]
                        if np.isnan(d_measured) or np.isinf(d_measured) or d_measured == 0:
                            visible_pts.append((u, v))
                        elif pt_c[2] < d_measured + DEPTH_OCCLUSION_TOLERANCE_M:
                            visible_pts.append((u, v))

                if len(visible_pts) > 0:
                    us = [p[0] for p in visible_pts]
                    vs = [p[1] for p in visible_pts]

                    u_min_box, u_max_box = max(0, min(us)), min(self.img_w - 1, max(us))
                    v_min_box, v_max_box = max(0, min(vs)), min(self.img_h - 1, max(vs))

                    x_c = (u_min_box + u_max_box) / 2.0 / self.img_w
                    y_c = (v_min_box + v_max_box) / 2.0 / self.img_h
                    bw = (u_max_box - u_min_box) / self.img_w
                    bh = (v_max_box - v_min_box) / self.img_h

                    px_w = u_max_box - u_min_box
                    px_h = v_max_box - v_min_box

                    if bw > 0 and bh > 0 and 0 <= x_c <= 1 and 0 <= y_c <= 1:
                        labels.append(f"{leaf['class']} {x_c:.6f} {y_c:.6f} {bw:.6f} {bh:.6f}")
                        frame_gt_count += 1
                        widths.append(px_w)
                        heights.append(px_h)
                        self.stats['class_counts'][leaf['class_name']] += 1

            if frame_gt_count == 0:
                self.stats['frames_without_gt'] += 1
                self.stats['rejected_frames'] += 1
                return "REJECTED_NO_VISIBLE"

            if frame_gt_count < self.min_visible_leaves:
                self.stats['rejected_frames'] += 1
                return "REJECTED_MIN_LEAVES"

            if all(pw < self.min_bbox_px and ph < self.min_bbox_px for pw, ph in zip(widths, heights)):
                self.stats['rejected_frames'] += 1
                return "REJECTED_SMALL_BBOX"

            self.seen_hashes.add(img_hash)

            self.stats['accepted_frames'] += 1
            self.stats['total_gt_labels'] += frame_gt_count
            self.stats['bbox_widths'].extend(widths)
            self.stats['bbox_heights'].extend(heights)

            if tree_name not in self.stats['frames_per_tree']:
                self.stats['frames_per_tree'][tree_name] = 0
            if tree_name not in self.stats['gt_labels_per_tree']:
                self.stats['gt_labels_per_tree'][tree_name] = 0
            self.stats['frames_per_tree'][tree_name] += 1
            self.stats['gt_labels_per_tree'][tree_name] += frame_gt_count

            rgb_path = os.path.join(self.rgb_dir, f"{vp_id}.png")
            cv2.imwrite(rgb_path, cv_rgb)

            depth_path = os.path.join(self.depth_dir, f"{vp_id}.tiff")
            cv2.imwrite(depth_path, cv_depth)

            label_path = os.path.join(self.labels_dir, f"{vp_id}.txt")
            with open(label_path, 'w') as f:
                f.write("\n".join(labels))

            t_r = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
            t_d = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9

            robot_q = [t_base.transform.rotation.x, t_base.transform.rotation.y, t_base.transform.rotation.z, t_base.transform.rotation.w]

            meta = {
                "viewpoint_id": vp_id,
                "target_tree": tree_name,
                "target_tree_position": {
                    "x": tree['T_w_t'][0, 3],
                    "y": tree['T_w_t'][1, 3],
                    "z": tree['T_w_t'][2, 3]
                },
                "robot_world_position": {
                    "x": t_base.transform.translation.x,
                    "y": t_base.transform.translation.y,
                    "z": t_base.transform.translation.z
                },
                "robot_world_orientation": {
                    "x": robot_q[0], "y": robot_q[1], "z": robot_q[2], "w": robot_q[3]
                },
                "rgb_timestamp": t_r,
                "depth_timestamp": t_d,
                "gazebo_pose_timestamp": time.time(),
                "rgb_depth_time_delta": abs(t_r - t_d),
                "pose_capture_time_delta": abs(time.time() - t_r),
                "pose_stability_error": math.hypot(t_base.transform.translation.x - rx, t_base.transform.translation.y - ry),
                "visible_leaf_count": frame_gt_count,
                "gt_label_count": frame_gt_count,
                "min_bbox_width_px": float(np.min(widths)) if widths else 0.0,
                "median_bbox_width_px": float(np.median(widths)) if widths else 0.0,
                "max_bbox_width_px": float(np.max(widths)) if widths else 0.0,
                "min_bbox_height_px": float(np.min(heights)) if heights else 0.0,
                "median_bbox_height_px": float(np.median(heights)) if heights else 0.0,
                "max_bbox_height_px": float(np.max(heights)) if heights else 0.0,
                "quality_pass": True
            }

            meta_path = os.path.join(self.meta_dir, f"{vp_id}.json")
            with open(meta_path, 'w') as f:
                json.dump(meta, f, indent=4)

            return f"ACCEPTED ({frame_gt_count} labels)"
        except Exception as e:
            self.stats['gt_projection_failures'] += 1
            self.stats['rejected_frames'] += 1
            return f"PROCESS_FAILURE: {e}"

    def on_shutdown(self):
        summary = {
            "Trees discovered": len(self.stats['trees_discovered']),
            "Total viewpoints attempted": self.stats['total_viewpoints_attempted'],
            "Accepted frames": self.stats['accepted_frames'],
            "Rejected frames": self.stats['rejected_frames'],
            "Duplicate frames": self.stats['duplicate_frames'],
            "Frames without GT": self.stats['frames_without_gt'],
            "GT projection failures": self.stats['gt_projection_failures'],
            "TF failures": self.stats['tf_failures'],
            "Camera info failures": self.stats['camera_info_failures'],
            "Per tree": {},
            "BBox statistics": {},
            "Class counts": self.stats['class_counts']
        }

        for t in sorted(self.stats['trees_discovered']):
            summary["Per tree"][t] = {
                "accepted": self.stats['frames_per_tree'].get(t, 0),
                "GT labels": self.stats['gt_labels_per_tree'].get(t, 0)
            }

        if self.stats['bbox_widths']:
            summary["BBox statistics"] = {
                "min_width": float(np.min(self.stats['bbox_widths'])),
                "median_width": float(np.median(self.stats['bbox_widths'])),
                "max_width": float(np.max(self.stats['bbox_widths'])),
                "min_height": float(np.min(self.stats['bbox_heights'])),
                "median_height": float(np.median(self.stats['bbox_heights'])),
                "max_height": float(np.max(self.stats['bbox_heights']))
            }

        print("\n\n=== COLLECTION SUMMARY ===")
        print(json.dumps(summary, indent=4))
        print("==========================\n")

        with open(os.path.join(self.run_dir, "collection_summary.json"), 'w') as f:
            json.dump(summary, f, indent=4)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

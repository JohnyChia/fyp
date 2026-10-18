#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from durian_message.srv import InspectionControl
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import threading
import time
import sys
import cv2
import logging
import io
import os
import pandas as pd
from fpdf import FPDF
from flask import Flask, Response, request, jsonify, send_file

flask_app = Flask(__name__)
ros_node_ref = None

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

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
RELATIVE_EVIDENCE_SEMANTICS = "relative_class_area_evidence_percent_v1"


def mission_status_qos():
    """Match the server's reliable transient-local status publisher."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        depth=1,
    )


def parse_mission_status(serialized):
    """Validate and normalize the versioned ROS mission-status payload."""
    payload = json.loads(serialized)
    if not isinstance(payload, dict):
        raise ValueError("mission status must be a JSON object")
    if payload.get("version") != MISSION_STATUS_VERSION:
        raise ValueError("unsupported mission status version")

    state = payload.get("state")
    if state not in MISSION_STATES:
        raise ValueError(f"unsupported mission state: {state}")
    terminal = payload.get("terminal")
    success = payload.get("success")
    inspection_id = payload.get("inspection_id")
    reason = payload.get("reason")
    if not isinstance(terminal, bool) or not isinstance(success, bool):
        raise ValueError("terminal and success must be booleans")
    if not isinstance(inspection_id, str) or not isinstance(reason, str):
        raise ValueError("inspection_id and reason must be strings")
    if terminal != (state in TERMINAL_MISSION_STATES):
        raise ValueError("terminal flag does not match mission state")
    if success != (state == "COMPLETED"):
        raise ValueError("success flag does not match mission state")

    return {
        "version": MISSION_STATUS_VERSION,
        "state": state,
        "inspection_id": inspection_id,
        "terminal": terminal,
        "success": success,
        "reason": reason,
    }


def generate_frames():
    last_encoded_frame = None
    while True:
        if ros_node_ref is None or getattr(ros_node_ref, 'latest_frame', None) is None:
            time.sleep(0.1)
            continue

        frame = ros_node_ref.latest_frame
        if frame is last_encoded_frame:
            time.sleep(0.03)
            continue

        last_encoded_frame = frame
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        if not ret:
            continue

        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.05)


@flask_app.route('/')
def index():
    return jsonify({"status": "Durian AI Inspection Server running", "version": "2.0"})


@flask_app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@flask_app.route('/api/status')
def api_status():
    if ros_node_ref is None:
        return jsonify({
            "service_ready": False,
            "current_status": "Starting...",
            "inspection_finished": False,
            "inspection_state": "UNKNOWN",
            "inspection_id": "",
            "inspection_succeeded": False,
            "failure_reason": "",
        })

    is_ready = ros_node_ref.client.service_is_ready() and getattr(ros_node_ref, 'nav2_ready', False)
    mission = ros_node_ref.ui_app.get_mission_snapshot()

    return jsonify({
        "service_ready": is_ready,
        "current_status": getattr(ros_node_ref.ui_app, 'current_status_text', "Running") if ros_node_ref.ui_app else "Running",
        "inspection_finished": mission["inspection_finished"],
        "inspection_state": mission["inspection_state"],
        "inspection_id": mission["inspection_id"],
        "inspection_succeeded": mission["inspection_succeeded"],
        "failure_reason": mission["failure_reason"],
    })


@flask_app.route('/api/command', methods=['POST'])
def api_command():
    if ros_node_ref is None:
        return jsonify({"success": False, "error": "Node not ready"})

    data = request.json
    cmd = data.get("cmd")

    if cmd == "start":
        is_ready = ros_node_ref.client.service_is_ready() and getattr(ros_node_ref, 'nav2_ready', False)
        if not is_ready:
            return jsonify({"success": False, "error": "Nav2 not ready yet, please wait"})
        success = ros_node_ref.send_command_sync("start")
        return jsonify({"success": success})
    elif cmd == "stop":
        success = ros_node_ref.send_command_sync("stop")
        return jsonify({"success": success})

    return jsonify({"success": False, "error": "Invalid command"})


@flask_app.route('/api/history', methods=['GET'])
def api_history():
    import sqlite3
    db_path = "/home/johny/durian_ws/durian_inspection.db"
    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        cursor.execute("""SELECT id, tree_x, tree_y, distance, scan_time, status,
                                 timestamp, disease_name, coverage, confidence, priority, remedy,
                                 tree_id, cache_hit
                          FROM inspection_log ORDER BY id DESC""")
        rows = cursor.fetchall()
        conn.close()

        history_list = []
        for r in rows:
            history_list.append({
                "id": r[0],
                "tree_x": r[1],
                "tree_y": r[2],
                "distance": r[3],
                "scan_time": r[4],
                "status": r[5],
                "timestamp": r[6],
                "disease_name": r[7],
                # Keep `coverage` for existing clients. It is relative class-area
                # evidence, not biological disease severity or canopy prevalence.
                "coverage": r[8],
                "relative_detection_evidence_percent": r[8],
                "coverage_semantics": RELATIVE_EVIDENCE_SEMANTICS,
                "confidence": r[9],
                "priority": r[10],
                "remedy": r[11],
                "tree_id": r[12],
                "cache_hit": bool(r[13])
            })
        return jsonify({"success": True, "history": history_list})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@flask_app.route('/api/export/excel', methods=['GET'])
def api_export_excel():
    import sqlite3
    db_path = "/home/johny/durian_ws/durian_inspection.db"
    if not os.path.exists(db_path):
        return jsonify({"success": False, "error": "Database not found"})
    try:
        conn = sqlite3.connect(db_path)
        df = pd.read_sql_query("SELECT id, tree_x, tree_y, distance, scan_time, status, timestamp, disease_name, coverage, priority, remedy FROM inspection_log", conn)
        conn.close()

        output = io.BytesIO()
        with pd.ExcelWriter(output, engine='openpyxl') as writer:
            df.to_excel(writer, index=False, sheet_name='Inspection Log')
        output.seek(0)

        return send_file(
            output,
            mimetype='application/vnd.openxmlformats-officedocument.spreadsheetml.sheet',
            as_attachment=True,
            download_name='inspection_report.xlsx'
        )
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@flask_app.route('/api/export/pdf', methods=['GET'])
def api_export_pdf():
    import sqlite3
    db_path = "/home/johny/durian_ws/durian_inspection.db"
    if not os.path.exists(db_path):
        return jsonify({"success": False, "error": "Database not found"})
    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT id, tree_x, tree_y, timestamp, disease_name, priority FROM inspection_log ORDER BY id DESC")
        rows = cursor.fetchall()
        conn.close()

        pdf = FPDF()
        pdf.add_page()
        pdf.set_font("Arial", size=16, style='B')
        pdf.cell(200, 10, txt="Durian Orchard Inspection Report", ln=True, align='C')
        pdf.ln(10)

        pdf.set_font("Arial", size=10, style='B')
        pdf.cell(15, 10, "ID", border=1, align='C')
        pdf.cell(25, 10, "Tree X", border=1, align='C')
        pdf.cell(25, 10, "Tree Y", border=1, align='C')
        pdf.cell(45, 10, "Time", border=1, align='C')
        pdf.cell(50, 10, "Disease", border=1, align='C')
        pdf.cell(30, 10, "Priority", border=1, align='C')
        pdf.ln(10)

        pdf.set_font("Arial", size=10)
        for row in rows:
            pdf.cell(15, 10, str(row[0]), border=1, align='C')
            pdf.cell(25, 10, f"{row[1]:.2f}", border=1, align='C')
            pdf.cell(25, 10, f"{row[2]:.2f}", border=1, align='C')
            pdf.cell(45, 10, str(row[3])[:19], border=1, align='C')
            pdf.cell(50, 10, str(row[4]), border=1, align='C')
            pdf.cell(30, 10, str(row[5]), border=1, align='C')
            pdf.ln(10)

        pdf_output = pdf.output(dest='S').encode('latin1')
        output = io.BytesIO(pdf_output)

        return send_file(
            output,
            mimetype='application/pdf',
            as_attachment=True,
            download_name='inspection_report.pdf'
        )
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


class RosGuiClient(Node):
    def __init__(self, ui_app):
        super().__init__("gui_node")
        self.ui_app = ui_app
        self.client = self.create_client(InspectionControl, "control_inspection")
        self.bridge = CvBridge()
        self.latest_frame = None
        self.nav2_ready = False

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, sub_qos)
        self.inspection_status_sub = self.create_subscription(
            String,
            "/inspection_status",
            self.inspection_status_callback,
            mission_status_qos(),
        )

        from geometry_msgs.msg import PoseStamped
        self.tree_found = False
        self.tree_sub = self.create_subscription(PoseStamped, "/discovered_tree_pose", self.tree_callback, 10)

        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        global ros_node_ref
        ros_node_ref = self

    def tree_callback(self, msg):
        self.tree_found = True

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def inspection_status_callback(self, msg):
        try:
            self.ui_app.apply_mission_status(parse_mission_status(msg.data))
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Invalid /inspection_status payload: {exc}")

    def monitor_service_status(self):
        time.sleep(0.5)
        if self.ui_app.get_mission_snapshot()["inspection_state"] in (
            "UNKNOWN",
            "READY",
        ):
            self.ui_app.update_status(
                "Attempting to connect to the control service node...", "orange"
            )

        while rclpy.ok():
            if self.client.service_is_ready():
                if self.nav_client.server_is_ready():
                    self.nav2_ready = True
                    if self.ui_app.get_mission_snapshot()["inspection_state"] in (
                        "UNKNOWN",
                        "READY",
                    ):
                        self.ui_app.update_status("Ready! Start inspection.", "green")
                    self.ui_app.enable_buttons()
                    break
                else:
                    if self.ui_app.get_mission_snapshot()["inspection_state"] in (
                        "UNKNOWN",
                        "READY",
                    ):
                        self.ui_app.update_status(
                            "Service Connected | Waiting for Nav2...", "orange"
                        )
            time.sleep(1.0)

    def send_command(self, cmd_string):
        if not self.client.service_is_ready():
            self.get_logger().error("Control service is not ready, cannot send command!")
            return

        req = InspectionControl.Request()
        req.command = cmd_string

        def cb(future):
            try:
                res = future.result()
                self.get_logger().info(f"Command '{cmd_string}' executed: {res.success}")
            except Exception as e:
                self.get_logger().error(f"Service response exception: {e}")

        try:
            future = self.client.call_async(req)
            future.add_done_callback(cb)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def send_command_sync(self, cmd_string):
        if not self.client.service_is_ready():
            self.get_logger().error("Control service is not ready, cannot send command!")
            return False

        req = InspectionControl.Request()
        req.command = cmd_string

        event = threading.Event()
        result = [False]
        def cb(future):
            try:
                res = future.result()
                result[0] = res.success
                self.get_logger().info(f"Command '{cmd_string}' sync executed: {res.success}")
            except Exception as e:
                self.get_logger().error(f"Service response sync exception: {e}")
            event.set()

        try:
            future = self.client.call_async(req)
            future.add_done_callback(cb)
            event.wait(timeout=15.0)
            return result[0]
        except Exception as e:
            self.get_logger().error(f"Service sync call failed: {e}")
            return False


class HeadlessApp:
    def __init__(self):
        self.current_status_text = "Initializing..."
        self._mission_state_lock = threading.Lock()
        self.inspection_state = "UNKNOWN"
        self.inspection_id = ""
        self.inspection_finished = False
        self.inspection_succeeded = False
        self.failure_reason = ""
        self.ros_node = RosGuiClient(self)

        self.ros_spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_spin_thread.start()

        self.check_thread = threading.Thread(target=self.ros_node.monitor_service_status, daemon=True)
        self.check_thread.start()

        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()

    def run_flask(self):
        try:
            from waitress import serve
            serve(flask_app, host="0.0.0.0", port=5000, threads=10)
        except ImportError:
            flask_app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False, threaded=True)

    def spin_ros(self):
        try:
            rclpy.spin(self.ros_node)
        except (KeyboardInterrupt, Exception):
            pass

    def update_status(self, text, color=None):
        self.current_status_text = text
        print(f"[STATUS UPDATE]: {text}")

    def apply_mission_status(self, payload):
        status_text = {
            "READY": "Ready! Start inspection.",
            "INSPECTING": "Inspecting...",
            "RETURNING_HOME": "Returning home...",
            "PROCESSING": "Processing inspection results...",
            "COMPLETED": "Inspection complete.",
            "CANCELED": "Inspection canceled.",
            "FAILED": "Inspection failed.",
        }[payload["state"]]
        if payload["state"] == "FAILED" and payload["reason"]:
            status_text = f"Inspection failed: {payload['reason']}"

        with self._mission_state_lock:
            self.inspection_state = payload["state"]
            self.inspection_id = payload["inspection_id"]
            self.inspection_finished = payload["terminal"]
            self.inspection_succeeded = payload["success"]
            self.failure_reason = (
                payload["reason"] if payload["state"] == "FAILED" else ""
            )
            self.current_status_text = status_text
        print(f"[MISSION STATUS]: {status_text}")

    def get_mission_snapshot(self):
        with self._mission_state_lock:
            return {
                "inspection_state": self.inspection_state,
                "inspection_id": self.inspection_id,
                "inspection_finished": self.inspection_finished,
                "inspection_succeeded": self.inspection_succeeded,
                "failure_reason": self.failure_reason,
            }

    def enable_buttons(self):
        pass

    def on_start_click(self):
        self.ros_node.send_command("start")

    def run(self):
        try:
            while rclpy.ok():
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass

    def shutdown(self):
        """Stop the ROS executor thread before destroying its node."""
        if rclpy.ok():
            rclpy.shutdown()
        self.ros_spin_thread.join(timeout=2.0)
        self.ros_node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    app = HeadlessApp()
    try:
        app.run()
    finally:
        app.shutdown()


if __name__ == "__main__":
    main()

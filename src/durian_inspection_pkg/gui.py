#!/usr/bin/env python3
import tkinter as tk
from tkinter import messagebox
import rclpy
from rclpy.node import Node
from durian_message.srv import InspectionControl
import threading
import time
import sys

class RosGuiClient(Node):
    def __init__(self, ui_app):
        super().__init__("gui_node")
        self.ui_app = ui_app

        self.client = self.create_client(InspectionControl, "/control_inspection")

    def monitor_service_status(self):
       
        time.sleep(0.5)
        self.ui_app.update_status("Attempting to connect to the control service node...", "orange")
        
        while rclpy.ok():
            if self.client.service_is_ready():
                self.ui_app.update_status(" Service Connected | System Ready", "green")
                self.ui_app.enable_buttons()
                break
            time.sleep(1.0)

    def send_command(self, cmd_string):
        if not self.client.service_is_ready():
            messagebox.showerror("Error", "Control service is not ready, cannot send command!")
            return

        req = InspectionControl.Request()
        req.command = cmd_string
    
        future = self.client.call_async(req)
        future.add_done_callback(self.command_response_callback)

    def command_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Command executed successfully")
            else:
                self.get_logger().warn("Command rejected by server")
                messagebox.showwarning("Notice", "Cannot perform this operation in the current state")
        except Exception as e:
            self.get_logger().error(f"Service call exception: {e}")

class InspectionGuiApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Durian Orchard Automated Inspection Console")
        self.root.geometry("450x250")
        self.root.resizable(False, False)

        # UI Layout
        self.label_title = tk.Label(self.root, text="Durian Inspection System", font=("Arial", 16, "bold"))
        self.label_title.pack(pady=15)
        
        self.status_frame = tk.Frame(self.root)
        self.status_frame.pack(pady=5)
        self.label_status_title = tk.Label(self.status_frame, text="System Status: ", font=("Arial", 11))
        self.label_status_title.pack(side=tk.LEFT)
        self.label_status = tk.Label(self.status_frame, text="Initializing...", font=("Arial", 11, "bold"), fg="orange")
        self.label_status.pack(side=tk.LEFT)
        
        self.btn_frame = tk.Frame(self.root)
        self.btn_frame.pack(pady=20)

        self.btn_start = tk.Button(self.btn_frame, text="Start Inspection", state=tk.DISABLED, command=self.on_start_click, font=("Arial", 12), bg="#4CAF50", fg="white", width=14, height=2)
        self.btn_start.pack(side=tk.LEFT, padx=15)

        self.btn_stop = tk.Button(self.btn_frame, text="Cancel / Return Home", state=tk.DISABLED, command=self.on_stop_click, font=("Arial", 12), bg="#f44336", fg="white", width=14, height=2)
        self.btn_stop.pack(side=tk.LEFT, padx=15)

        self.ros_node = RosGuiClient(self)

        self.ros_spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_spin_thread.start()

        self.check_thread = threading.Thread(target=self.ros_node.monitor_service_status, daemon=True)
        self.check_thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def spin_ros(self):
        try:
            rclpy.spin(self.ros_node)
        except Exception:
            pass

    def update_status(self, text, color):
        self.label_status.config(text=text, fg=color)

    def enable_buttons(self):
        self.btn_start.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.NORMAL)

    def on_start_click(self):
        self.update_status("🔵 Inspecting...", "blue")
        self.ros_node.send_command("start")

    def on_stop_click(self):
        if messagebox.askyesno("Cancellation Confirmation", "Are you sure you want to interrupt the current inspection task and return the robot home?"):
            self.update_status(" Inspection canceled, returning home...", "orange")
            self.ros_node.send_command("stop")

    def on_close(self):
        self.root.destroy()
        self.ros_node.destroy_node()

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    app = InspectionGuiApp()
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
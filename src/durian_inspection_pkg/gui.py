import tkinter as tk
import rclpy
from rclpy.node import Node
from durian_message.srv import InspectionControl

class InspectionGUI(Node):
    def __init__(self):
        super().__init__('inspection_gui')
        self.cli = self.create_client(InspectionControl, 'control_inspection')
        self.root = tk.Tk()
        self.root.title("Durian Robot Control")
        tk.Button(self.root, text="START", command=lambda: self.send_cmd("start"), bg="green").pack(fill=tk.X)
        tk.Button(self.root, text="STOP", command=lambda: self.send_cmd("stop"), bg="red").pack(fill=tk.X)
        self.root.after(100, self.ros_loop)
        self.root.mainloop()

    def ros_loop(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(100, self.ros_loop)

    def send_cmd(self, cmd):
        req = InspectionControl.Request()
        req.command = cmd
        self.cli.call_async(req)

def main():
    rclpy.init()
    node = InspectionGUI() 
    # 彻底删除原来的 rclpy.spin(node)
    # 因为我们在 __init__ 里已经有了 self.root.mainloop()
    
    # 这样启动：
    node.root.mainloop()
    node.destroy_node()
    rclpy.shutdown()
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from rclpy.qos import QoSProfile, ReliabilityPolicy
# from tf2_ros import Buffer, DurabilityPolicy, TransformListener
# from tf_transformations import euler_from_quaternion
# import math

# class ScannerNode(Node):
#     def __init__(self):
#         super().__init__('scanner_node')
        
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             depth=10
#         )
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)

#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
#         self.state = "ROTATING"
#         self.accumulated_yaw = 0.0
#         self.last_yaw = None
#         self.create_timer(0.1, self.control_callback)

#     def control_callback(self):
#         current_yaw = self.get_current_yaw()
#         if current_yaw is None: return

#         # 1. 增量计算逻辑
#         if self.last_yaw is not None:
#             delta = current_yaw - self.last_yaw
#             if delta > math.pi: delta -= 2 * math.pi
#             elif delta < -math.pi: delta += 2 * math.pi
#             self.accumulated_yaw += abs(delta)
#         self.last_yaw = current_yaw

#         # 2. 状态机逻辑
#         msg = Twist() # 提前初始化，避免 UnboundLocalError
        
#         if self.state == "ROTATING":
#             self.get_logger().info(f"Scanning... Accumulated: {self.accumulated_yaw:.2f}")
            
#             if self.accumulated_yaw > 5.8: 
#                 msg.angular.z = 0.0
#                 self.state = "ANALYZING"
#                 self.accumulated_yaw = 0.0 
#                 self.get_logger().info("✅ 扫描完成")
#             else:
#                 msg.angular.z = 0.2
            
#             self.cmd_vel_pub.publish(msg) # 只在这里发布
            
#         elif self.state == "MOVING":
#             # 这里可以添加你后续的移动逻辑
#             msg.linear.x = 0.1
#             self.cmd_vel_pub.publish(msg)

#     def get_current_yaw(self):
#         try:
#             # 等待变换可用，设定一个极短的超时以防阻塞
#             target_frame = 'odom' 
#             t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
#             q = [
#                 t.transform.rotation.x, 
#                 t.transform.rotation.y, 
#                 t.transform.rotation.z, 
#                 t.transform.rotation.w
#             ]
#             _, _, yaw = euler_from_quaternion(q)
#             return yaw
#         except Exception as e:
#             # 这里的日志可以帮助你判断是否是 TF 树没对齐
#             # self.get_logger().warn(f"TF lookup failed: {e}") 
#             return None

# def main(args=None):
#     rclpy.init(args=args)
#     node = ScannerNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
# from sensor_msgs.msg import LaserScan

# class LidarSimulatorNode(Node):
#     def __init__(self):
#         super().__init__('lidar_simulator')
#         # 发布到 /scan 话题
#         qos_profile_sensor_data = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             depth=10
#         )
#         self.publisher = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
#         self.timer = self.create_timer(0.1, self.timer_callback)

#     def timer_callback(self):
#         scan = LaserScan()
#         # 关键：使用当前系统时间，防止 TF 时间戳滞后导致的“漂移”
#         scan.header.stamp = self.get_clock().now().to_msg()
#         scan.header.frame_id = 'laser'
        
#         # 激光参数设置
#         scan.angle_min = -1.57
#         scan.angle_max = 1.57
#         scan.angle_increment = 0.01745 # 约 1 度
#         scan.range_min = 0.1
#         scan.range_max = 20.0
#         # 填充数据（模拟）
#         scan.ranges = [1.0] * 768
        
#         self.publisher.publish(scan)

# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarSimulatorNode()
#     rclpy.spin(node)
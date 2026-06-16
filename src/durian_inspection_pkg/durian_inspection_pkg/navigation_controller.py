# import rclpy
# import numpy as np
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist

# class RobotController(Node):
#     def __init__(self):
#         super().__init__('robot_controller')

#         # params
#         self.declare_parameter('odom_pose_covariance', [
#             0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
#             0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
#             0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
#             0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
#             0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
#             0.0, 0.0, 0.0, 0.0, 0.0, 0.01
#         ])

#         self.pose_cov = self.get_parameter('odom_pose_covariance').value

#         self.declare_parameter('update_rate', 0.02)
#         self.declare_parameter('odom_frame', 'odom')
#         self.declare_parameter('base_frame', 'base_link')

#         self.update_rate = self.get_parameter('update_rate').value
#         self.odom_frame = self.get_parameter('odom_frame').value
#         self.base_frame = self.get_parameter('base_frame').value

#         # state
#         self.x, self.y, self.theta = 0.0, 0.0, 0.0
#         self.last_time = self.get_clock().now()
#         self.cmd_vel = Twist()

#         # pub/sub
#         self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_vel_callback, 10)
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

#         self.create_timer(self.update_rate, self.publish_odom)

#         self.get_logger().info("Robot Controller started (NO TF publishing)")

#     def cmd_vel_callback(self, msg):
#         self.cmd_vel = msg

#     def publish_odom(self):
#         now = self.get_clock().now()
#         dt = (now - self.last_time).nanoseconds / 1e9
#         self.last_time = now

#         # integrate motion
#         self.x += self.cmd_vel.linear.x * np.cos(self.theta) * dt
#         self.y += self.cmd_vel.linear.x * np.sin(self.theta) * dt
#         self.theta += self.cmd_vel.angular.z * dt

#         # odom msg
#         odom = Odometry()
#         odom.header.stamp = now.to_msg()
#         odom.header.frame_id = self.odom_frame
#         odom.child_frame_id = self.base_frame

#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y

#         odom.pose.pose.orientation.x = 0.0
#         odom.pose.pose.orientation.y = 0.0
#         odom.pose.pose.orientation.z = np.sin(self.theta / 2.0)
#         odom.pose.pose.orientation.w = np.cos(self.theta / 2.0)

#         odom.pose.covariance = list(self.pose_cov)

#         odom.twist.twist.linear.x = self.cmd_vel.linear.x
#         odom.twist.twist.linear.y = self.cmd_vel.linear.y
#         odom.twist.twist.angular.z = self.cmd_vel.angular.z

#         self.odom_pub.publish(odom)


# def main():
#     rclpy.init()
#     node = RobotController()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
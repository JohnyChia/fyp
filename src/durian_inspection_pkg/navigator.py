import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Bool
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf2_ros import Buffer, TransformListener

class DurianNavigator(Node):
    def __init__(self):
        super().__init__('durian_navigator')
        self.navigator = BasicNavigator()
        self.is_running = False
        self.goal_sent = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Bool, '/inspection_status', self.status_callback, 10)
        self.create_subscription(PointCloud2, '/cloud_map', self.map_callback, 10)

    def status_callback(self, msg):
        new_status = msg.data
        # 只有状态从 True 变为 False 时，才执行取消任务
        if self.is_running and not new_status:
            self.get_logger().info("Received stop signal, canceling navigation.")
            self.goal_sent = False
            self.navigator.cancelTask()
        self.is_running = new_status

    def map_callback(self, msg):
        if not self.is_running or self.goal_sent: return
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x, robot_y = trans.transform.translation.x, trans.transform.translation.y
            min_dist, target_tree = float('inf'), None
            
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                if p[2] > 0.5:
                    dist = math.sqrt((p[0]-robot_x)**2 + (p[1]-robot_y)**2)
                    if dist < min_dist:
                        min_dist, target_tree = dist, (p[0], p[1])
            
            if target_tree and min_dist > 1.0:
                dist_to_tree = math.sqrt((robot_x - target_tree[0])**2 + (robot_y - target_tree[1])**2)
                goal_x = target_tree[0] + (robot_x - target_tree[0]) / dist_to_tree * 1.0
                goal_y = target_tree[1] + (robot_y - target_tree[1]) / dist_to_tree * 1.0
                self.goal_sent = True
                self.send_goal(goal_x, goal_y)
        except Exception as e: 
            # 这里修复了你之前代码里 e 未定义的 bug
            self.get_logger().warn(f"TF Lookup failed: {e}")

    def send_goal(self, x, y):
        self.get_logger().info(f"发送目标点到: x={x}, y={y}")
        goal = PoseStamped()
        goal.header.frame_id = 'map'; goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x; goal.pose.position.y = y
        self.navigator.goToPose(goal)

def main():
    rclpy.init()
    rclpy.spin(DurianNavigator())
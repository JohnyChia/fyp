#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import traceback

class TestPub(Node):
    def __init__(self):
        super().__init__('test_pub')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("--- 测试节点已启动 ---")

    def timer_callback(self):
        try:
            msg = LaserScan()
            msg.header.frame_id = "laser"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.angle_min = -3.14
            msg.angle_max = 3.14
            msg.angle_increment = 0.01
            msg.ranges = [1.0] * 629
            self.pub.publish(msg)
            self.get_logger().info(">>> 已成功发布一帧数据 >>>")
        except Exception as e:
            self.get_logger().error(f"发生错误: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TestPub()
    try:
        rclpy.spin(node)
    except Exception:
        print(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
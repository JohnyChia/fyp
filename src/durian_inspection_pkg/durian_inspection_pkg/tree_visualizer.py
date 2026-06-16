# import rclpy
# from rclpy.node import Node
# from visualization_msgs.msg import Marker, MarkerArray
# from std_msgs.msg import Float32MultiArray, Int32

# class TreeVisualizer(Node):
#     def __init__(self):
#         super().__init__('tree_visualizer')
#         # 发布 MarkerArray 到 RViz
#         self.publisher = self.create_publisher(MarkerArray, '/tree_markers', 10)
        
#         # 存储已完成的树木 ID
#         self.finished_ids = set()
        
#         # 订阅数据：[rel_x, rel_y, gid, disease_pct]
#         self.create_subscription(Float32MultiArray, '/robot_dog/tree_target', self.listener_callback, 10)
#         self.create_subscription(Int32, '/tree_finished', self.finished_callback, 10)

#     def finished_callback(self, msg):
#         self.finished_ids.add(msg.data)

#     def get_color_by_disease(self, pct):
#         """根据病害百分比返回颜色 (r, g, b)"""
#         # Green: < 20%, Yellow: 20-60%, Red: >= 60%
#         if pct < 0.2:
#             return (0.0, 1.0, 0.0)  # 健康: 绿色
#         elif pct < 0.6:
#             return (1.0, 1.0, 0.0)  # 轻度: 黄色
#         else:
#             return (1.0, 0.0, 0.0)  # 重度: 红色

#     def listener_callback(self, msg):
#         self.get_logger().info(f"Received: {msg.data}")
#         # 假设 msg.data 格式为: [x, y, gid, disease_pct]
#         if len(msg.data) < 4:
#             self.get_logger().warn("接收到的数据维度不足")
#             return

#         rel_x, rel_y, gid, disease_pct = msg.data[0], msg.data[1], int(msg.data[2]), msg.data[3]
        
#         # 使用 0 表示永久显示，或设置如 5.0 表示5秒后消失
#         lifetime = rclpy.duration.Duration(seconds=0.0).to_msg() 
        
#         # 1. 创建球体 Marker
#         sphere_marker = Marker()
#         sphere_marker.header.frame_id = "map"
#         sphere_marker.header.stamp = self.get_clock().now().to_msg()
#         sphere_marker.ns = "trees"
#         sphere_marker.id = gid
#         sphere_marker.type = Marker.SPHERE
#         sphere_marker.action = Marker.ADD
#         sphere_marker.lifetime = lifetime
#         sphere_marker.pose.position.x = float(rel_x) * 2.0
#         sphere_marker.pose.position.y = float(rel_y) * 2.0
#         sphere_marker.pose.position.z = 0.25
        
#         # 颜色逻辑
#         if gid in self.finished_ids:
#             sphere_marker.color.r, sphere_marker.color.g, sphere_marker.color.b = (0.5, 0.5, 0.5) # 已完成: 灰色
#         else:
#             r, g, b = self.get_color_by_disease(disease_pct)
#             sphere_marker.color.r, sphere_marker.color.g, sphere_marker.color.b = (r, g, b)
        
#         sphere_marker.color.a = 1.0
#         sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = 0.5
        
#         # 2. 创建文字 Marker
#         text_marker = Marker()
#         text_marker.header.frame_id = "map"
#         text_marker.header.stamp = self.get_clock().now().to_msg()
#         text_marker.ns = "tree_labels"
#         text_marker.id = gid
#         text_marker.type = Marker.TEXT_VIEW_FACING
#         text_marker.action = Marker.ADD
#         text_marker.lifetime = lifetime
#         text_marker.pose.position.x = float(rel_x) * 2.0
#         text_marker.pose.position.y = float(rel_y) * 2.0 + 0.3 # 标签上移
#         text_marker.pose.position.z = 0.8
#         text_marker.text = f"ID:{gid} ({int(disease_pct*100)}%)"
#         text_marker.scale.z = 0.3
#         text_marker.color.r = text_marker.color.g = text_marker.color.b = 1.0
#         text_marker.color.a = 1.0
        
#         # 发布 MarkerArray
#         arr = MarkerArray(markers=[sphere_marker, text_marker])
#         self.publisher.publish(arr)

# def main(args=None):
#     rclpy.init(args=args)
#     node = TreeVisualizer()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
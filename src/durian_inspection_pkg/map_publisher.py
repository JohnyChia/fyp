import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import sqlite3


class DbMapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.publisher = self.create_publisher(PointCloud2, '/static_map', qos_profile)
        self.cloud_msg = self.load_map_from_db()
        self.timer = self.create_timer(1.0, self.publish_once)

    def publish_once(self):
        if self.cloud_msg:
            self.cloud_msg.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            self.publisher.publish(
                self.cloud_msg
            )

    def load_map_from_db(self):
        self.get_logger().info("Loading map from DB into memory...")
        try:
            conn = sqlite3.connect("/home/johny/durian_ws/durian_data.db")
            cursor = conn.cursor()

            cursor.execute("SELECT x, y, z, rgb FROM point_clouds WHERE rowid % 50 = 0 AND x BETWEEN -200 AND 200 AND y BETWEEN -200 AND 200")
            rows = cursor.fetchall()
            conn.close()

            header = Header()

            header.frame_id = "map"

            header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            return pc2.create_cloud(header, fields, rows)
        except Exception as e:
            self.get_logger().error(f"Failed to read map data from DB: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = DbMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
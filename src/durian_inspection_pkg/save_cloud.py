import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import sqlite3
import os


class CloudAggregator(Node):
    def __init__(self):
        super().__init__('cloud_aggregator')
        self.sub = self.create_subscription(PointCloud2, '/cloud_map', self.callback, 10)

        self.db_name = "durian_data.db"
        self.conn = sqlite3.connect(self.db_name)
        self.cursor = self.conn.cursor()
        self.cursor.execute('''CREATE TABLE IF NOT EXISTS point_clouds 
                               (id INTEGER PRIMARY KEY, x REAL, y REAL, z REAL, rgb REAL)''')
        self.conn.commit()

    def callback(self, msg):
        raw_points = pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)
        points = [(float(x), float(y), float(z), float(rgb)) for x, y, z, rgb in raw_points]
        try:
            self.cursor.execute("BEGIN TRANSACTION")
            self.cursor.execute("DELETE FROM point_clouds")
            self.cursor.executemany("INSERT INTO point_clouds (x, y, z, rgb) VALUES (?, ?, ?, ?)", points)
            self.cursor.execute("SELECT count(*) FROM point_clouds")
            db_count = self.cursor.fetchone()[0]
            self.conn.commit()
            self.get_logger().info(f"Replaced DB with /cloud_map (frame_id={msg.header.frame_id}, stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}): {len(points)} pts in msg, {db_count} pts in DB")
        except Exception as e:
            self.conn.rollback()
            self.get_logger().error(f"Failed to save map snapshot: {e}")

    def export_to_pcd(self):
        print("Exporting database to durian_tree.pcd...")
        self.cursor.execute("SELECT x, y, z, rgb FROM point_clouds")
        rows = self.cursor.fetchall()

        if not rows:
            print("No points in database.")
            return

        with open("durian_tree.pcd", "w") as f:
            f.write(f"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(rows)}\nHEIGHT 1\nPOINTS {len(rows)}\nDATA ascii\n")
            for r in rows:
                f.write(f"{r[0]} {r[1]} {r[2]} {r[3]}\n")
        print(f"Successfully exported {len(rows)} points to .pcd")

    def destroy_node(self):
        self.conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CloudAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.export_to_pcd()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
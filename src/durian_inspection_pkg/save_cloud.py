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
        points = list(pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True))
        self.cursor.executemany("INSERT INTO point_clouds (x, y, z, rgb) VALUES (?, ?, ?, ?)", points)
        self.conn.commit() 
        self.get_logger().info(f"Buffered points to DB", throttle_duration_sec=5.0)

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
import asyncio
import json
import rclpy
import socket
from rclpy.node import Node
from sensor_msgs.msg import Imu
from aiohttp import web
import websockets
from geometry_msgs.msg import TransformStamped
import tf2_ros
import base64
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu, CameraInfo

class BridgeNode(Node):
    def __init__(self):
        super().__init__('durian_bridge')
        self.pub_imu = self.create_publisher(Imu, '/mobile/imu', 10)
        self.pub_image = self.create_publisher(Image, '/camera/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.cv_bridge = CvBridge()
       
    def publish_image(self, data, width, height, stride):
        try:
            raw_data = base64.b64decode(data)
            nparr = np.frombuffer(raw_data, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
            msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
            
            now = self.get_clock().now().to_msg()
            msg.header.stamp = now
            msg.header.frame_id = 'camera_link_optical'
            self.pub_image.publish(msg)

            info_msg = CameraInfo()
            info_msg.header.stamp = now
            info_msg.header.frame_id = 'camera_link_optical'
            info_msg.width = width
            info_msg.height = height
            fx = 525.0
            fy = 525.0
            cx = float(width) / 2.0
            cy = float(height) / 2.0
            info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
 
            info_msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.pub_info.publish(info_msg)
            
        except Exception as e:
            self.get_logger().error(f"convert image failed: {e}")
        

    def publish_imu(self, imu):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.linear_acceleration.x = float(imu.get('ax', 0))
        msg.linear_acceleration.y = float(imu.get('ay', 0))
        msg.linear_acceleration.z = float(imu.get('az', 0))

        self.pub_imu.publish(msg)
        
async def handle_config(request):
    print("Receive HTTP request")
    return web.Response(text="OK", status=200)

async def ws_handler(websocket):
    print("WebSocket connected")
    await websocket.send(json.dumps({
        "type": "status",
        "status": "connected"
    }))
    
    try:
        async for message in websocket:
            data = json.loads(message)
            msg_type = data.get("type")
            
            if msg_type == "image":
                if "data" in data and "w" in data and "h" in data:
                    bridge.publish_image(data["data"], data["w"], data["h"], data.get("stride", data["w"]))
                else:
                    print("Error")
            
            if "imu" in data:
                bridge.publish_imu(data["imu"])
                
    except websockets.exceptions.ConnectionClosed:
        print("WebSocket connection failed")
    except Exception as e:
        print(f"analyze data fa: {e}")

def main(args=None):
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        pass

async def main_async():
    global bridge
    rclpy.init()
    bridge = BridgeNode()

    app = web.Application()
    app.router.add_post('/config', handle_config)
    runner = web.AppRunner(app)
    await runner.setup()
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', 9988))
    
    site = web.SockSite(runner, sock)
    await site.start()
    print("HTTP service (Port 9988)")

    server = await websockets.serve(ws_handler, "0.0.0.0", 9095, reuse_address=True)
    print("WebSocket service (Port 9095)")
    
    try:
        while rclpy.ok():
            rclpy.spin_once(bridge, timeout_sec=0.1)
            await asyncio.sleep(0.01)
    finally:
        print("service closing ...")
        await server.close()
        await runner.cleanup()
        bridge.destroy_node()
        rclpy.shutdown()

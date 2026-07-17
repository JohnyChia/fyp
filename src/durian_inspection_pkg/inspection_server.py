import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist

from durian_message.srv import InspectionControl

import sensor_msgs_py.point_cloud2 as pc2

import math
import threading
import time


class InspectionServer(Node):

    def __init__(self):

        super().__init__(
            "inspection_server"
        )


        # =========================
        # Nav2 Action
        # =========================

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose"
        )


        # =========================
        # Service
        # =========================

        self.service = self.create_service(
            InspectionControl,
            "control_inspection",
            self.command_callback
        )


        # =========================
        # Subscribers
        # =========================

        self.create_subscription(
            PointCloud2,
            "/cloud_map",
            self.map_callback,
            10
        )


        self.create_subscription(
            PointCloud2,
            "/camera/points",
            self.camera_callback,
            10
        )


        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )



        # =========================
        # Publisher
        # =========================

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )


        # =========================
        # Variables
        # =========================

        self.running = False


        # robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0


        # database map
        self.tree_points = []


        # camera cloud
        self.camera_points = None


        self.goal_running=False



        self.get_logger().info(
            "Inspection Server Ready"
        )



    # =====================================================
    # START / STOP
    # =====================================================

    def command_callback(
        self,
        request,
        response
    ):


        if request.command=="start":

            self.running=True

            self.get_logger().info(
                "Inspection START"
            )


            threading.Thread(
                target=self.inspect_loop,
                daemon=True
            ).start()


            response.success=True



        elif request.command=="stop":

            self.running=False

            self.get_logger().info(
                "Inspection STOP"
            )


            response.success=True


        return response





    # =====================================================
    # Robot odometry
    # =====================================================

    def odom_callback(
        self,
        msg
    ):

        self.robot_x = (
            msg.pose.pose.position.x
        )

        self.robot_y = (
            msg.pose.pose.position.y
        )




    # =====================================================
    # Receive SQLite cloud map
    # =====================================================

    def map_callback(
        self,
        msg
    ):


        self.tree_points=[]


        for p in pc2.read_points(
            msg,
            field_names=(
                "x",
                "y",
                "z"
            ),
            skip_nans=True
        ):


            x,y,z=p


            # filter tree height

            if z > 0.5:

                self.tree_points.append(
                    (
                        x,
                        y,
                        z
                    )
                )



        self.get_logger().info(
            f"Map trees:{len(self.tree_points)}"
        )






    # =====================================================
    # Depth camera
    # =====================================================

    def camera_callback(
        self,
        msg
    ):

        self.camera_points=msg




    # =====================================================
    # Main logic
    # =====================================================


    def inspect_loop(self):


        while self.running:


            if len(self.tree_points)==0:

                self.get_logger().warn(
                    "No tree in cloud_map"
                )

                time.sleep(2)
                continue



            if self.goal_running:

                time.sleep(1)
                continue



            target = self.find_nearest_tree()


            if target is None:

                continue



            goal_x,goal_y=\
                self.calculate_goal(target)



            self.send_goal(
                goal_x,
                goal_y
            )


            self.goal_running=True


            time.sleep(10)





    # =====================================================
    # Find nearest tree
    # =====================================================


    def find_nearest_tree(self):


        min_distance=float("inf")

        nearest=None



        for x,y,z in self.tree_points:


            distance=math.sqrt(
                (x-self.robot_x)**2+
                (y-self.robot_y)**2
            )


            if distance < min_distance:


                min_distance=distance

                nearest=(x,y,z)



        if nearest:


            self.get_logger().info(
                f"""
Nearest tree:
x={nearest[0]}
y={nearest[1]}
distance={min_distance}
"""
            )



        return nearest





    # =====================================================
    # stop 1 meter before tree
    # =====================================================

    def calculate_goal(
        self,
        tree
    ):


        tx,ty,tz=tree


        dx=self.robot_x-tx
        dy=self.robot_y-ty


        distance=math.sqrt(
            dx*dx+dy*dy
        )


        stop_distance=1.0



        goal_x=(
            tx+
            dx/distance*
            stop_distance
        )


        goal_y=(
            ty+
            dy/distance*
            stop_distance
        )



        return goal_x,goal_y





    # =====================================================
    # Send Nav2 goal
    # =====================================================


    def send_goal(
        self,
        x,
        y
    ):


        if not self.nav_client.wait_for_server(
            timeout_sec=5
        ):

            self.get_logger().error(
                "Nav2 unavailable"
            )

            return




        goal=NavigateToPose.Goal()


        goal.pose.header.frame_id="map"


        goal.pose.header.stamp=(
            self.get_clock()
            .now()
            .to_msg()
        )


        goal.pose.pose.position.x=x

        goal.pose.pose.position.y=y

        goal.pose.pose.orientation.w=1.0



        self.get_logger().info(
            f"""
Send Nav Goal:
{x},{y}
"""
        )


        future=\
            self.nav_client.send_goal_async(
                goal
            )


        future.add_done_callback(
            self.goal_response
        )





    def goal_response(
        self,
        future
    ):


        goal_handle=future.result()


        if not goal_handle.accepted:


            self.get_logger().error(
                "Goal rejected"
            )

            self.goal_running=False

            return



        self.get_logger().info(
            "Goal accepted"
        )


        result_future=\
            goal_handle.get_result_async()


        result_future.add_done_callback(
            self.goal_finished
        )





    def goal_finished(
        self,
        future
    ):


        self.get_logger().info(
            "Arrived tree area"
        )


        self.goal_running=False





def main():

    rclpy.init()

    node=InspectionServer()


    executor=rclpy.executors.MultiThreadedExecutor()

    executor.add_node(node)


    executor.spin()



    node.destroy_node()

    rclpy.shutdown()



if __name__=="__main__":

    main()
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
from durian_message.action import Inspection 

class InspectionActionServer(Node):
    def __init__(self):
        super().__init__('inspection_action_server')
        self._callback_group = ReentrantCallbackGroup()
        
        self._action_server = ActionServer(
            self,
            Inspection,
            'perform_inspection',
            execute_callback=self.execute_callback,
            callback_group=self._callback_group,
            cancel_callback=self.cancel_callback)
            
        self.get_logger().info("Edge AI Durian Tree Inspection Action Server is ready and running in parallel.")

    def cancel_callback(self):
        self.get_logger().warn("Cancel request received")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Starting Edge AI scan/task for tree [ID: {goal_handle.request.tree_id}]...")
        feedback_msg = Inspection.Feedback()

        for i in range(1, 6):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn("Inspection task successfully and safely cancelled!")
                result = Inspection.Result()
                result.success = False
                result.report = "Inspection task aborted safely due to external command."
                return result

            feedback_msg.progress = i * 20.0
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Inspection analysis progress: {feedback_msg.progress}%")
            time.sleep(1.0) 
 
        goal_handle.succeed()
        result = Inspection.Result()
        result.success = True
        result.report = f"Health assessment for tree [ID: {goal_handle.request.tree_id}] completed"
        return result

def main(args=None):
    rclpy.init(args=args)
    server = InspectionActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(server, executor=executor)
    server.destroy_node()
    rclpy.shutdown()
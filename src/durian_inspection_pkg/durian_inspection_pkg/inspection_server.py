import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
# 确保你的包在 package.xml 中依赖了 durian_msgs
from durian_message.action import Inspection 

class InspectionActionServer(Node):
    def __init__(self):
        super().__init__('inspection_action_server')
        
        # 引入并行回调组，防止动作阻塞节点其他线程
        self._callback_group = ReentrantCallbackGroup()
        
        self._action_server = ActionServer(
            self,
            Inspection,
            'perform_inspection',
            execute_callback=self.execute_callback,
            callback_group=self._callback_group,
            cancel_callback=self.cancel_callback) # 增加了取消机制，保障野外四足狗安全
            
        self.get_logger().info("✅ 边缘端 Edge AI 榴莲树巡检 Action Server 已并行就绪")

    def cancel_callback(self, cancel_request):
        """当遇到紧急情况（例如机器人滑坡），允许导航控制器强行中止喷洒/检测作业"""
        self.get_logger().warn("🚨 收到取消作业请求！正在紧急中止当前树木巡检...")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"🚀 开始对树木 [ID: {goal_handle.request.tree_id}] 进行 Edge AI 扫描/作业...")
        
        feedback_msg = Inspection.Feedback()
        
        # 循环 5 次模拟巡检或精密喷洒过程
        for i in range(1, 6):
            # 检查客户端是否发起了取消请求
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn("🛑 作业已成功安全取消！")
                result = Inspection.Result()
                result.success = False
                result.report = "由于外部指令，巡检作业提前安全中止"
                return result

            feedback_msg.progress = i * 20.0
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"📊 巡检分析进度: {feedback_msg.progress}%")
            
            # 使用了多线程执行器后，这里可以直接使用 time.sleep 
            time.sleep(1.0) 
            
        # 作业圆满完成
        goal_handle.succeed()
        result = Inspection.Result()
        result.success = True
        result.report = f"树木 {goal_handle.request.tree_id} 健康度分析完毕：未发现病虫害，生长状态良好。"
        return result

def main(args=None):
    rclpy.init(args=args)
    server = InspectionActionServer()
    
    # 修复提示：使用 MultiThreadedExecutor 代替单线程 spin，释放多核边缘计算算力
    executor = MultiThreadedExecutor()
    rclpy.spin(server, executor=executor)
    
    server.destroy_node()
    rclpy.shutdown()
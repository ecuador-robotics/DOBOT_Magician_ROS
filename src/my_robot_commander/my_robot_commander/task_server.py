import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from my_robot_msgs.action import Task
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


class TaskServer(Node):
    def __init__(self):
        super().__init__(node_name="task_server")
        self.get_logger().info("Starting the Server")
        
        self.action_server = ActionServer(
            self,
            Task,
            "task_server",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.myrobot = MoveItPy(node_name="moveit_py")
        self.myrobot_arm = self.myrobot.get_planning_component("arm")
        
        self.get_logger().info("Task Server Ready")

    def goal_callback(self, goal_request):
        task_num = goal_request.task_number
        
        if task_num < 1 or task_num > 5:
            self.get_logger().warn(f"Received invalid task number: {task_num}")
            return GoalResponse.REJECT
        
        self.get_logger().info(f"Received valid task number: {task_num}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Executing task {goal_handle.request.task_number}")
        
        feedback_msg = Task.Feedback()
        result = Task.Result()

        task_positions = {
            1: np.array([0.0, 0.0, 0.0]),      # Home
            2: np.array([1.57, 0.0, 0.0]),     # Rotate 90 degrees
            3: np.array([0.0, 0.78, 0.0]),     # Extend arm
            4: np.array([-1.57, 0.5, 0.3]),    # Custom position 1
            5: np.array([0.78, 1.0, -0.1])     # Custom position 2
        }

        task_num = goal_handle.request.task_number
        
        if task_num not in task_positions:
            self.get_logger().error(f"Task number {task_num} not defined")
            goal_handle.abort()
            result.success = False
            return result

        feedback_msg.percentage = 10
        goal_handle.publish_feedback(feedback_msg)

        arm_state = RobotState(self.myrobot.get_robot_model())
        arm_state.set_joint_group_positions("arm", task_positions[task_num])

        self.myrobot_arm.set_start_state_to_current_state()
        self.myrobot_arm.set_goal_state(robot_state=arm_state)

        feedback_msg.percentage = 30
        goal_handle.publish_feedback(feedback_msg)

        arm_plan_result = self.myrobot_arm.plan()

        if not arm_plan_result:
            self.get_logger().error("Planning failed")
            goal_handle.abort()
            result.success = False
            return result

        feedback_msg.percentage = 60
        goal_handle.publish_feedback(feedback_msg)

        try:
            self.myrobot.execute(arm_plan_result.trajectory, controllers=[])
            
            feedback_msg.percentage = 100
            goal_handle.publish_feedback(feedback_msg)
            
            goal_handle.succeed()
            result.success = True
            self.get_logger().info(f"Task {task_num} completed successfully")
            
        except Exception as e:
            self.get_logger().error(f"Execution failed: {e}")
            goal_handle.abort()
            result.success = False

        return result


def main(args=None):
    rclpy.init(args=args)
    task_server = TaskServer()
    
    try:
        rclpy.spin(task_server)
    except KeyboardInterrupt:
        pass
    finally:
        task_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
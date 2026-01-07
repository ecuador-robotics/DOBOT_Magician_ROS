import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from my_robot_msgs.action import Task
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

class TaskServer(Node):
    def __init__(self):
        super().__init__("task_server")
        self.get_logger().info("Starting the Server")
        self.action_server = ActionServer(
            self, Task, "task_server", self.goalCallback
        )

        self.myrobot = MoveItPy(node_name="moveit_py")
        self.myrobot_arm = self.myrobot.get_planning_component("arm")

    def goalCallback(self, goal_handle):
        self.get_logger().info(
            "Received goal request with task_number %d" % goal_handle.request.task_number
        )

        arm_state = RobotState(self.myrobot.get_robot_model())

        arm_joint_goal = []

        if goal_handle.request.task_number == 1:
            arm_joint_goal = np.array([0.0, 0.0, 0.0])
        elif goal_handle.request.task_number == 2:
            arm_joint_goal = np.array([1.57, 0.0, 0.0])
        elif goal_handle.request.task_number == 3:
            arm_joint_goal = np.array([0.0, 0.78, 0.0])
        else:
            self.get_logger().info("Invalid task number")
            return 
        
        arm_state.set_joint_group_positions("arm", arm_joint_goal)

        self.myrobot_arm.set_start_state_to_current_state()

        self.myrobot_arm.set_goal_state(robot_state=arm_state)

        arm_plan_result = self.myrobot_arm.plan()

        if arm_plan_result:
            self.myrobot.execute(arm_plan_result.trajectory, controllers=[])
        else:
            self.get_logger().info("Planning failed")
            return
        
        goal_handle.succeed()
        result = Task.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    task_server = TaskServer()
    rclpy.spin(task_server)


if __name__ == "__main__":
    main()
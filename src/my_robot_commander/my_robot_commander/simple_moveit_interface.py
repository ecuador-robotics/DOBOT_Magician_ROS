import rclpy
from rclpy.logging import get_logger
import numpy as np
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


def move_robot():
    myrobot = MoveItPy(node_name="moveit")
    myrobot_arm = myrobot.get_planning_component("arm")

    arm_state = RobotState(myrobot.get_robot_model())

    arm_state.set_joint_group_positions("arm", np.array([1.57, 0.0, 0.0]))

    myrobot_arm.set_start_state_to_current_state()

    myrobot_arm.set_goal_state(robot_state=arm_state)

    arm_plan_result = myrobot_arm.plan()

    if arm_plan_result:
        myrobot.execute(arm_plan_result.trajectory, controllers=[])
    else:
        get_logger("rclpy").error("Planning failed")


def main():
    rclpy.init()
    move_robot()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



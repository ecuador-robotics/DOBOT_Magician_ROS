#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

SPHERE_JOINTS = {
    "rojo":  [ 0.2359,  0.8288, -0.0091],
    "azul":  [-0.0106,  0.8277,  0.0550],
    "verde": [-0.2448,  0.8252, -0.0019],
}

HOME_JOINTS = [0.0, 0.0, 0.0]

class GoalNode(Node):
    def __init__(self):
        super().__init__("goal_node")
        self.robot = MoveItPy(node_name="goal_node")
        self.arm   = self.robot.get_planning_component("arm")
        self.get_logger().info("GoalNode listo")

    def esperar(self, segundos=0.5):
        inicio = self.get_clock().now()
        while (self.get_clock().now() - inicio).nanoseconds < segundos * 1e9:
            rclpy.spin_once(self, timeout_sec=0.05)

    def mover_joints(self, j1, j2, j3):
        arm_state = RobotState(self.robot.get_robot_model())
        arm_state.set_joint_group_positions("arm", np.array([j1, j2, j3]))
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(robot_state=arm_state)
        plan = self.arm.plan()
        if plan:
            self.robot.execute(plan.trajectory, controllers=[])
            self.esperar(1.0)
            return True
        self.get_logger().error(f" Plan falló")
        return False

    def ir_a_home(self):
        self.get_logger().info(" Home...")
        self.mover_joints(*HOME_JOINTS)

    def ir_a_esfera(self, color: str):
        if color not in SPHERE_JOINTS:
            self.get_logger().error(f"Color '{color}' no reconocido")
            return False
        self.get_logger().info(f" Yendo a esfera {color}")
        joints = SPHERE_JOINTS[color]
        ok = self.mover_joints(*joints)
        if ok:
            self.get_logger().info(f"End effector en esfera {color}")
            self.esperar(1.0)
            self.ir_a_home()
        return ok


def main():
    import sys
    rclpy.init()
    try:
        node = GoalNode()
        color = sys.argv[1] if len(sys.argv) > 1 else "azul"
        node.ir_a_esfera(color)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("myrobot", package_name="my_robot_moveit")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("my_robot_description"),
            "urdf",
            "arm.urdf.xacro"
            )
        )
        .robot_description_semantic(file_path="config/myrobot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path="config/planing_python_api.yaml")
        .to_moveit_configs()
    )

    simple_moveit_interface = Node(
        package="my_robot_commander",
        executable="simple_moveit_interface",
        parameters=[moveit_config.to_dict(), 
                    {"use_sim_time": True}],
    )

    return LaunchDescription([
        simple_moveit_interface
    ])
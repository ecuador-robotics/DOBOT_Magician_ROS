import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("myrobot", package_name="my_robot_moveit")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("my_robot_description"),
            "urdf", "arm.urdf.xacro"
        ))
        .robot_description_semantic(file_path="config/myrobot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path="config/planning_python_api.yaml")
        .to_moveit_configs()
    )

    pipeline_node = Node(
        package="my_robot_pipeline",
        executable="pipeline_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )

    llm_node = Node(
        package="my_robot_pipeline",
        executable="llm_node",
        output="screen",
    )

    return LaunchDescription([
        pipeline_node,
        llm_node,
    ])
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    my_robot_description = get_package_share_directory("my_robot_description")
    
    gz_launch_path = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world',
        description='Default world file name'
    )

    world_file = [LaunchConfiguration('world'), TextSubstitution(text='.sdf')]

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(my_robot_description, "urdf", "arm.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(my_robot_description).parent.resolve()),
            ":",
            os.path.join(my_robot_description, "worlds")
        ]
    )
    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=False"
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': [
                TextSubstitution(text='-v 4 -r '),
                PathJoinSubstitution([my_robot_description, 'worlds', world_file])
            ],
            'on_exit_shutdown': 'True'
        }.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "armrobot"],
    )

    bridge_params = os.path.join(
        my_robot_description,
        'params',
        'armrobot_bridge.yaml'
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/rgb_camera/image_raw'],
        output='screen',
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        start_gazebo_ros_image_bridge_cmd,
    ])
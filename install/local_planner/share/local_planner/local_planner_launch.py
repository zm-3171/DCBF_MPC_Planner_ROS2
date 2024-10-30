import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share_dir = get_package_share_directory('local_planner')
    config_file_path = os.path.join(share_dir, 'config.yaml')
    print(config_file_path)

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=config_file_path,
        description='FPath to the ROS2 parameters file to use.'
    )

    local_planner_node = Node(
        package='local_planner',
        executable='local_planner_node',
        name='local_planner_node',
        output='screen',
        parameters=[config_file_path],
    )
    controller_node = Node(
        package='local_planner',
        executable='controller_node',
        name='controller_node',
        output='screen',
    )
    return LaunchDescription([
        params_declare,
        local_planner_node,
        controller_node,
    ])
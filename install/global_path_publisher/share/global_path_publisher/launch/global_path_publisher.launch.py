import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('global_path_publisher')
    config_file_path = os.path.join(share_dir, 'config/config.yaml')

    print(config_file_path)

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=config_file_path,
        description='FPath to the ROS2 parameters file to use.'
    )

    global_path_publish_node = Node(
        package='global_path_publisher',
        executable='global_path_publisher',
        name='global_path_publisher',
        output='screen',
        parameters=[config_file_path]
    )


    return LaunchDescription([
        params_declare,
        global_path_publish_node
    ])
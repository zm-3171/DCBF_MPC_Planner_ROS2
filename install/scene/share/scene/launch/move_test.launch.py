import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('scene')
    config_file_path = os.path.join(share_dir, 'config/config.yaml')

    print(config_file_path)

    # parameter_file = LaunchConfiguration('params_file')
    # config_file_path = "/home/c/weston_ws/MinkLoc/src/global_locator/config/config.yaml"
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=config_file_path,
        description='FPath to the ROS2 parameters file to use.'
    )

    move_test_node = Node(
        package='scene',
        executable='move_test',
        name='move_test',
        output='screen',
        parameters=[config_file_path]
    )

    return LaunchDescription([
        params_declare,
        move_test_node
    ])
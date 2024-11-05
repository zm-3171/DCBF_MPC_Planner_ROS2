import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    local_map_config_file_path = PathJoinSubstitution(
        [get_package_share_directory('local_map'), 'config', 'config.yaml']
    )
    obs_kf_config_file_path = PathJoinSubstitution(
        [get_package_share_directory('obs_param'), 'config', 'config.yaml']
    )
    local_map_params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=local_map_config_file_path,
        description='FPath to the ROS2 parameters file to use.'
    )

    # Define the launch description
    return LaunchDescription([
        # Static transform broadcaster (world to odom)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='world_odom_broadcaster',
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '0', 
        #         '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', 
        #         '--frame-id', 'world', '--child-frame-id', 'odom'
        #     ]
        # ),

        # Include the local map launch
        local_map_params_declare,
        Node(
            package='local_map',
            executable='local_map',
            name='local_map',
            output='screen',
            parameters=[local_map_config_file_path]
            # parameters=[LaunchConfiguration('params_file')]
        ),

        # # Launch the observation parameter node
        Node(
            package='obs_param',
            executable='obs_kf',
            name='obs_kf',
            output='screen',
            parameters=[obs_kf_config_file_path]
        ),

        # Launch linear path publisher node
        Node(
            package='linear_path_publisher',
            executable='linear_path_publisher',
            name='linear_path_publisher',
            output='screen'
        ),

        # Launch RViz with the provided config file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', PathJoinSubstitution(
                [get_package_share_directory('local_map'), 'rviz', 'rviz_config.rviz'])],
            output='screen'
        ),
    ])

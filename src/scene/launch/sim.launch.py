import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value=[PathJoinSubstitution(
            [get_package_share_directory('scene'), 'worlds', 'world1.world'])],
        description='World file to load'
    )
    
    # Paths to launch files
    scene_launch_file = PathJoinSubstitution(
        [get_package_share_directory('scene'), 'launch', 'start.launch.py'])
    
    movetest_launch_file = PathJoinSubstitution(
        [get_package_share_directory('scene'), 'launch', 'move_test.launch.py'])

    # global_path_launch_file = PathJoinSubstitution(
    #     [get_package_share_directory('global_path_publisher'), 'launch', 'global_path_publisher.launch.py'])
    
    local_map_config_file_path = PathJoinSubstitution(
        [get_package_share_directory('local_map'), 'config', 'config.yaml']
    )
    local_map_params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=local_map_config_file_path,
        description='FPath to the ROS2 parameters file to use.'
    )

    turtlebot_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Define the launch description
    return LaunchDescription([
        declare_world_arg,

        # Include the scene launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(scene_launch_file),
            launch_arguments={'world': LaunchConfiguration('world')}.items(),
        ),

        # Static transform broadcaster (world to odom)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_odom_broadcaster',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0', 
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', 
                '--frame-id', 'world', '--child-frame-id', 'odom'
            ]
        ),

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

        # # Include movetest node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(movetest_launch_file)
        ),

        # # Launch the observation parameter node
        Node(
            package='obs_param',
            executable='obs_kf',
            name='obs_kf',
            output='screen'
        ),

        # # Launch the observation parameter node
        Node(
            package='scene',
            executable='pseudo_odom',
            name='pseudo_odom',
            output='screen'
        ),

        # # Include global path publisher launch
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(global_path_launch_file)
        # ),
        # # Launch the observation parameter node
        Node(
            package='linear_path_publisher',
            executable='linear_path_publisher',
            name='linear_path_publisher',
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot_launch_file_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot_launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose
            }.items()
        ),


        # Launch RViz with the provided config file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', PathJoinSubstitution(
                [get_package_share_directory('scene'), 'rviz', 'rviz_config.rviz'])],
            output='screen'
        ),
    ])

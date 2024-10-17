from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare launch arguments
    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value='worlds/map.world',
        description='World file to load'
    )
    
    declare_gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Whether to run Gazebo GUI'
    )
    
    declare_args_arg = DeclareLaunchArgument(
        name='args',
        default_value='',
        description='Additional arguments for Gazebo'
    )

    # Define nodes and groups
    use_sim_time_param = ParameterValue(True, value_type=bool)

    # Start gazebo
    gazebo_node = Node(
        package='gazebo_ros',
        executable='/usr/bin/gazebo',  # 注意这里的变化
        name='gazebo',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_param}],
        arguments=[LaunchConfiguration('world')]
    )

    # Conditionally start gazebo with GUI
    gazebo_gui_node = GroupAction(
        actions=[
            Node(
                package='gazebo_ros',
                executable='/usr/bin/gazebo',  # 注意这里的变化
                name='gazebo_gui',
                output='screen',
                arguments=['--gui']
            )
        ],
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Nodes from scene package
    # getrobot_pose_node = Node(
    #     package='scene',
    #     executable='getrobot_pose',
    #     name='getrobot_pose',
    #     output='screen'
    # )

    # pseudo_odom_node = Node(
    #     package='scene',
    #     executable='pseudo_odom',
    #     name='pseudo_odom',
    #     output='screen'
    # )

    # Launch description
    return LaunchDescription([
        declare_world_arg,
        declare_gui_arg,
        declare_args_arg,
        gazebo_node,
        gazebo_gui_node,
        # getrobot_pose_node,
        # pseudo_odom_node,
    ])

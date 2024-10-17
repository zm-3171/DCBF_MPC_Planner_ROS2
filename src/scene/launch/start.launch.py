from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value=PathJoinSubstitution([
            get_package_share_directory('scene'),
            'worlds',
            'world0.world'
        ]),
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

    # Include the standard Gazebo launch file from gazebo_ros package
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'verbose': 'true'  # 可选，启用详细输出
        }.items(),
    )

    # Launch description
    return LaunchDescription([
        declare_world_arg,
        declare_gui_arg,
        declare_args_arg,
        gazebo_launch,
    ])

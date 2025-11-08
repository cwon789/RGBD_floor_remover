import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('floor_removal_rgbd')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'params.yaml'),
        description='Path to the configuration file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Create floor removal node
    floor_removal_node = Node(
        package='floor_removal_rgbd',
        executable='floor_removal_node',
        name='floor_removal_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Add remappings here if needed
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        floor_removal_node
    ])

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get directories and file paths
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bumperbot_navigation_dir = get_package_share_directory('bumperbot_navigation')
    
    default_param_file = os.path.join(
        bumperbot_navigation_dir,
        'config',
        'waypoint.yaml'
    )
    
    # Launch configurations
    param_file = LaunchConfiguration('param_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='Full path to the ROS2 parameters file to use for waypoint follower'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Waypoint follower and lifecycle manager nodes
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[param_file, {'use_sim_time': use_sim_time}]
    )
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_waypoint',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['waypoint_follower']}
        ]
    )
    
    return LaunchDescription([
        declare_param_file_cmd,
        declare_use_sim_time_cmd,
        waypoint_follower_node,
        lifecycle_manager_node,
        ])
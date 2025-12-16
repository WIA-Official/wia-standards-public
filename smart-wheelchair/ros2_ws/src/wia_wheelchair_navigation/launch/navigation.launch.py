"""
WIA Smart Wheelchair Navigation Launch File

Launches all navigation nodes including SLAM, planners, and goal manager.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('wia_wheelchair_navigation')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to map file for localization mode'
    )

    mapping_mode_arg = DeclareLaunchArgument(
        'mapping_mode',
        default_value='false',
        description='Enable mapping mode (vs localization mode)'
    )

    locations_file_arg = DeclareLaunchArgument(
        'locations_file',
        default_value='',
        description='Path to saved locations YAML file'
    )

    # Config file paths
    nav_params_file = PathJoinSubstitution([
        pkg_share, 'config', 'nav_params.yaml'
    ])

    costmap_params_file = PathJoinSubstitution([
        pkg_share, 'config', 'costmap_params.yaml'
    ])

    # SLAM Manager Node
    slam_manager_node = Node(
        package='wia_wheelchair_navigation',
        executable='slam_node.py',
        name='slam_manager',
        namespace='wia_wheelchair',
        parameters=[
            nav_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    # Global Planner Node
    global_planner_node = Node(
        package='wia_wheelchair_navigation',
        executable='global_planner.py',
        name='global_planner',
        namespace='wia_wheelchair',
        parameters=[
            nav_params_file,
            costmap_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    # Local Planner Node
    local_planner_node = Node(
        package='wia_wheelchair_navigation',
        executable='local_planner.py',
        name='local_planner',
        namespace='wia_wheelchair',
        parameters=[
            nav_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    # Goal Manager Node
    goal_manager_node = Node(
        package='wia_wheelchair_navigation',
        executable='goal_manager.py',
        name='goal_manager',
        namespace='wia_wheelchair',
        parameters=[
            nav_params_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'locations_file': LaunchConfiguration('locations_file'),
            },
        ],
        output='screen',
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        map_file_arg,
        mapping_mode_arg,
        locations_file_arg,

        # Nodes
        slam_manager_node,
        global_planner_node,
        local_planner_node,
        goal_manager_node,
    ])

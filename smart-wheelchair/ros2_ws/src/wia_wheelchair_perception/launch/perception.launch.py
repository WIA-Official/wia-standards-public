"""
WIA Smart Wheelchair Perception Launch File

Launches obstacle detection and sensor fusion nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.55',
        description='Distance between wheels (meters)'
    )

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.15',
        description='Wheel radius (meters)'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish odom -> base_link transform'
    )

    # Node configurations
    obstacle_detector_node = Node(
        package='wia_wheelchair_perception',
        executable='obstacle_detector.py',
        name='obstacle_detector',
        namespace='wia_wheelchair',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'detection_range': 5.0,
            'safety_margin': 0.3,
            'danger_distance': 0.5,
            'warning_distance': 1.5,
            'cluster_tolerance': 0.1,
            'min_cluster_size': 3,
            'robot_width': 0.7,
        }],
        remappings=[
            ('scan', '/wia_wheelchair/scan'),
            ('safety_status', '/wia_wheelchair/safety_status'),
            ('collision_warning', '/wia_wheelchair/collision_warning'),
        ],
        output='screen',
    )

    sensor_fusion_node = Node(
        package='wia_wheelchair_perception',
        executable='sensor_fusion.py',
        name='sensor_fusion',
        namespace='wia_wheelchair',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }],
        remappings=[
            ('imu', '/wia_wheelchair/imu'),
            ('joint_states', '/wia_wheelchair/joint_states'),
            ('odom', '/wia_wheelchair/odom'),
            ('state', '/wia_wheelchair/state'),
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        wheel_base_arg,
        wheel_radius_arg,
        publish_tf_arg,
        obstacle_detector_node,
        sensor_fusion_node,
    ])

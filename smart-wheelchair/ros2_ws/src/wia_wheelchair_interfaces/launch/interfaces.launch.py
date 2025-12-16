"""
WIA Smart Wheelchair Interfaces Launch File

Launches all assistive device interface nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('wia_wheelchair_interfaces')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    enable_gaze_arg = DeclareLaunchArgument(
        'enable_gaze',
        default_value='false',
        description='Enable eye gaze control'
    )

    enable_bci_arg = DeclareLaunchArgument(
        'enable_bci',
        default_value='false',
        description='Enable BCI control'
    )

    enable_voice_arg = DeclareLaunchArgument(
        'enable_voice',
        default_value='true',
        description='Enable voice control'
    )

    enable_smarthome_arg = DeclareLaunchArgument(
        'enable_smarthome',
        default_value='true',
        description='Enable smart home integration'
    )

    # Config file path
    params_file = PathJoinSubstitution([
        pkg_share, 'config', 'interfaces_params.yaml'
    ])

    # Interface Manager (always runs)
    interface_manager_node = Node(
        package='wia_wheelchair_interfaces',
        executable='interface_manager.py',
        name='interface_manager',
        namespace='wia_wheelchair',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    # Eye Gaze Controller
    gaze_node = Node(
        package='wia_wheelchair_interfaces',
        executable='gaze_node.py',
        name='gaze_controller',
        namespace='wia_wheelchair',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_gaze')),
    )

    # BCI Controller
    bci_node = Node(
        package='wia_wheelchair_interfaces',
        executable='bci_node.py',
        name='bci_controller',
        namespace='wia_wheelchair',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_bci')),
    )

    # Voice Controller
    voice_node = Node(
        package='wia_wheelchair_interfaces',
        executable='voice_node.py',
        name='voice_controller',
        namespace='wia_wheelchair',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_voice')),
    )

    # Smart Home Bridge
    smarthome_node = Node(
        package='wia_wheelchair_interfaces',
        executable='smarthome_node.py',
        name='matter_bridge',
        namespace='wia_wheelchair',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_smarthome')),
    )

    # Transition Manager (for exoskeleton integration)
    transition_node = Node(
        package='wia_wheelchair_interfaces',
        executable='transition_node.py',
        name='transition_manager',
        namespace='wia_wheelchair',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        enable_gaze_arg,
        enable_bci_arg,
        enable_voice_arg,
        enable_smarthome_arg,

        # Nodes
        interface_manager_node,
        gaze_node,
        bci_node,
        voice_node,
        smarthome_node,
        transition_node,
    ])

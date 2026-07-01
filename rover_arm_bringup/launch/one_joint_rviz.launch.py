"""Launch robot_state_publisher with the one-joint URDF, and optionally the bridge node.

This launch file lets you visualize /joint_states on the one-joint URDF in
RViz. RViz itself is not auto-launched (per README instructions, run `rviz2`
manually) so you can configure the display once and save your own config.

Usage:
    ros2 launch rover_arm_bringup one_joint_rviz.launch.py
    ros2 launch rover_arm_bringup one_joint_rviz.launch.py start_bridge:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('rover_arm_bringup')
    default_xacro = os.path.join(pkg_share, 'urdf', 'one_joint_arm.urdf.xacro')
    default_config = os.path.join(pkg_share, 'config', 'single_joint.yaml')

    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value=default_xacro,
        description='Path to the one-joint URDF/xacro file',
    )
    start_bridge_arg = DeclareLaunchArgument(
        'start_bridge',
        default_value='true',
        description='Whether to also start single_joint_odrive_bridge alongside RViz support',
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the YAML parameter file for single_joint_odrive_bridge',
    )
    use_joint_state_publisher_gui_arg = DeclareLaunchArgument(
        'use_joint_state_publisher_gui',
        default_value='false',
        description=(
            'Start joint_state_publisher_gui to drive /joint_states with sliders. '
            'Useful for checking the URDF moves correctly with no hardware connected. '
            'Do not combine with start_bridge:=true on real hardware - both would '
            'publish /joint_states.'
        ),
    )

    # The xacro_file path is wrapped in literal quotes before being handed to
    # the Command substitution, which shlex-splits the final string. Without
    # quoting, a workspace path containing spaces (e.g. ".../arm base joint
    # rover/...") would be split into multiple bogus arguments for xacro.
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', '"', LaunchConfiguration('xacro_file'), '"']),
            value_type=str,
        )
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_state_publisher_gui')),
    )

    bridge_node = Node(
        package='rover_arm_bringup',
        executable='single_joint_odrive_bridge',
        name='single_joint_odrive_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config_file')],
        condition=IfCondition(LaunchConfiguration('start_bridge')),
    )

    return LaunchDescription([
        xacro_file_arg,
        start_bridge_arg,
        config_file_arg,
        use_joint_state_publisher_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        bridge_node,
    ])

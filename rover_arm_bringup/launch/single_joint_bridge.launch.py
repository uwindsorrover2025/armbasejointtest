"""Launch only the single-joint ODrive bridge node, parameterized from YAML.

Usage:
    ros2 launch rover_arm_bringup single_joint_bridge.launch.py
    ros2 launch rover_arm_bringup single_joint_bridge.launch.py config_file:=/path/to/other.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('rover_arm_bringup')
    default_config = os.path.join(pkg_share, 'config', 'single_joint.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the YAML parameter file for single_joint_odrive_bridge',
    )

    bridge_node = Node(
        package='rover_arm_bringup',
        executable='single_joint_odrive_bridge',
        name='single_joint_odrive_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_file_arg,
        bridge_node,
    ])

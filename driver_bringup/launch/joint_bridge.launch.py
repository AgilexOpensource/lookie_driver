from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from driver_bringup.models import DEFAULT_MODEL, default_config_paths


def generate_launch_description() -> LaunchDescription:
    config_dir = Path(get_package_share_directory('driver_bringup')) / 'config'
    default_config = default_config_paths(config_dir, DEFAULT_MODEL)['bridge_config']

    return LaunchDescription([
        DeclareLaunchArgument(
            'bridge_config',
            default_value=str(default_config),
            description='Joint bridge YAML (modules + name mapping).',
        ),
        Node(
            package='driver_bringup',
            executable='joint_bridge_node',
            name='joint_bridge_node',
            output='screen',
            parameters=[
                {'config_file': LaunchConfiguration('bridge_config')},
            ],
        ),
    ])

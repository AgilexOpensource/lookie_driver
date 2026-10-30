"""Lookie driver bringup: head, torso (lift/fold), dual arms and joint bridge.

Model names follow lookie_description. Unknown names are rejected; configs are
checked for consistency before any driver node is started.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from driver_bringup.models import DEFAULT_MODEL, MODELS, default_config_paths, get_spec, validate


def _include(package: str, launch_file: str, arguments: dict) -> IncludeLaunchDescription:
    path = Path(get_package_share_directory(package)) / 'launch' / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=arguments.items(),
    )


def _arm_node(namespace: str, params_file: str) -> Node:
    return Node(
        package='agx_arm_ctrl',
        executable='agx_arm_ctrl_single',
        name='agx_arm_ctrl_single_node',
        namespace=namespace,
        output='screen',
        parameters=[params_file],
    )


def _setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    spec = get_spec(model)

    bringup_share = Path(get_package_share_directory('driver_bringup'))
    defaults = default_config_paths(bringup_share / 'config', model)

    files = {}
    for key, default in defaults.items():
        value = LaunchConfiguration(key).perform(context).strip()
        path = Path(value) if value else default
        if not path.is_absolute():
            raise ValueError(f'{key} must be an absolute path, got {value!r}')
        if not path.is_file():
            raise FileNotFoundError(f'{key} not found: {path}')
        files[key] = str(path)

    validate(model, files['robot_yaml'], files['params_file'], files['bridge_config'])

    torso_package = f'{spec.torso_module}_ros'
    actions = [
        _include('head_ros', 'head.launch.py', {
            'namespace': 'head',
            'yaml_path': files['robot_yaml'],
            'params_file': files['params_file'],
        }),
        _include(torso_package, f'{spec.torso_module}.launch.py', {
            'namespace': spec.torso_module,
            'yaml_path': files['robot_yaml'],
            'params_file': files['params_file'],
        }),
        _arm_node('left_arm', files['params_file']),
        _arm_node('right_arm', files['params_file']),
        _include('driver_bringup', 'joint_bridge.launch.py', {
            'bridge_config': files['bridge_config'],
        }),
    ]

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=DEFAULT_MODEL,
            choices=list(MODELS),
            description='Lookie model name.',
        ),
        DeclareLaunchArgument(
            'robot_yaml',
            default_value='',
            description='Absolute motor YAML path; empty selects '
                        'config/lookie__pf-<platform>_motors.yaml.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Absolute ROS params path; empty selects config/<model>_ros.yaml.',
        ),
        DeclareLaunchArgument(
            'bridge_config',
            default_value='',
            description='Absolute joint bridge YAML path; empty selects '
                        'config/<model>_joint_bridge.yaml.',
        ),
        OpaqueFunction(function=_setup),
    ])

"""Lookie model registry and driver config consistency checks.

Model names follow lookie_description (``lookie__pf-<platform>__ee-<ee>``);
the package itself is not required at runtime.
Only registered names are accepted; unknown names are rejected, never
mapped to a default.
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class ModelSpec:
    platform: str  # lookie_description platform token, e.g. lookie_trio_r01
    torso_module: str  # motor YAML module key / ROS namespace: lift or fold
    effector_type: str  # agx_arm_ctrl effector_type


MODELS = {
    'lookie__pf-lookie_neo_r01__ee-agx_gripper_r01': ModelSpec(
        'lookie_neo_r01', 'lift', 'agx_gripper'),
    'lookie__pf-lookie_neo_r01__ee-revo2std_r01': ModelSpec(
        'lookie_neo_r01', 'lift', 'revo2'),
    'lookie__pf-lookie_trio_r01__ee-agx_gripper_r01': ModelSpec(
        'lookie_trio_r01', 'fold', 'agx_gripper'),
    'lookie__pf-lookie_trio_r01__ee-revo2std_r01': ModelSpec(
        'lookie_trio_r01', 'fold', 'revo2'),
}

# Fully qualified node names started by lift_ros / fold_ros launch files.
TORSO_NODES = {'lift': '/lift/motor_node', 'fold': '/fold/fold_node'}

DEFAULT_MODEL = 'lookie__pf-lookie_trio_r01__ee-revo2std_r01'

# Joint limits in driver YAML may be tighter than URDF, never wider.
LIMIT_TOLERANCE = 1e-4


def get_spec(model: str) -> ModelSpec:
    if model not in MODELS:
        raise ValueError(f'Unknown model {model!r}, choose from {sorted(MODELS)}')
    return MODELS[model]


def default_config_paths(config_dir, model: str) -> dict:
    """Return default robot_yaml / params_file / bridge_config for a model."""
    spec = get_spec(model)
    config_dir = Path(config_dir)
    return {
        'robot_yaml': config_dir / f'lookie__pf-{spec.platform}_motors.yaml',
        'params_file': config_dir / f'{model}_ros.yaml',
        'bridge_config': config_dir / f'{model}_joint_bridge.yaml',
    }


def _load_yaml(path) -> dict:
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'{path}: expected a mapping at top level')
    return data


def actuated_urdf_joints(urdf_path) -> dict:
    """Map non-fixed, non-mimic, non-wheel URDF joints to (lower, upper)."""
    joints = {}
    for joint in ET.parse(urdf_path).getroot().findall('joint'):
        name = joint.get('name')
        jtype = joint.get('type')
        if jtype in ('fixed', 'floating', 'planar') or joint.find('mimic') is not None:
            continue
        if jtype == 'continuous' or 'wheel' in name:
            continue  # chassis wheels are not driven by this bringup
        limit = joint.find('limit')
        if limit is None:
            raise ValueError(f'{urdf_path}: joint {name!r} has no <limit>')
        joints[name] = (float(limit.get('lower', '-inf')), float(limit.get('upper', 'inf')))
    return joints


def bridge_joints(bridge_config) -> dict:
    """Map bridge module name to the list of robot-side joint names."""
    data = _load_yaml(bridge_config)
    result = {}
    for module in data.get('modules') or []:
        if 'joint_map' in module:
            result[module['name']] = [str(v) for v in module['joint_map'].values()]
        else:
            result[module['name']] = [str(v) for v in module.get('joints') or []]
    return result


def motor_joint_limits(robot_yaml) -> dict:
    """Map motor YAML module key to {joint_name: (lower, upper)}."""
    data = _load_yaml(robot_yaml)
    result = {}
    for key, module in data.items():
        result[key] = {
            str(j['joint_name']): (float(j['lower_limit']), float(j['upper_limit']))
            for j in module.get('joints') or []
        }
    return result


def validate(model: str, robot_yaml, params_file, bridge_config, urdf_path=None) -> None:
    """Raise ValueError if driver configs are inconsistent (and, if given, mismatch the URDF)."""
    spec = get_spec(model)
    urdf = actuated_urdf_joints(urdf_path) if urdf_path is not None else None
    errors = []

    motors = motor_joint_limits(robot_yaml)
    for key in ('head', spec.torso_module):
        if key not in motors:
            errors.append(f'{robot_yaml}: missing motor module {key!r}')
    for key, joints in motors.items():
        for name, (lower, upper) in joints.items():
            if lower > upper:
                errors.append(f'{robot_yaml}: {name} lower_limit > upper_limit')
            if urdf is None:
                continue
            if name not in urdf:
                errors.append(f'{robot_yaml}: {key}.{name} is not an actuated URDF joint')
                continue
            u_lower, u_upper = urdf[name]
            if lower < u_lower - LIMIT_TOLERANCE or upper > u_upper + LIMIT_TOLERANCE:
                errors.append(
                    f'{robot_yaml}: {name} limits [{lower}, {upper}] exceed '
                    f'URDF [{u_lower}, {u_upper}]')

    bridged = bridge_joints(bridge_config)
    flat = [name for names in bridged.values() for name in names]
    duplicated = sorted({name for name in flat if flat.count(name) > 1})
    if duplicated:
        errors.append(f'{bridge_config}: joints bridged more than once: {duplicated}')
    if urdf is not None:
        unknown = sorted(set(flat) - set(urdf))
        missing = sorted(set(urdf) - set(flat))
        if unknown:
            errors.append(f'{bridge_config}: joints not actuated in URDF: {unknown}')
        if missing:
            errors.append(f'{bridge_config}: URDF joints not bridged: {missing}')
    for module in ('head', spec.torso_module):
        if sorted(bridged.get(module, [])) != sorted(motors.get(module, {})):
            errors.append(
                f'{module}: bridge joints {bridged.get(module)} != motor joints '
                f'{sorted(motors.get(module, {}))}')

    params = _load_yaml(params_file)
    torso_node = TORSO_NODES[spec.torso_module]
    for node in ('/head/head_node', torso_node):
        if node not in params:
            errors.append(f'{params_file}: missing {node} section')
    for side in ('left_arm', 'right_arm'):
        node = f'/{side}/agx_arm_ctrl_single_node'
        effector = (params.get(node) or {}).get('ros__parameters', {}).get('effector_type')
        if effector != spec.effector_type:
            errors.append(
                f'{params_file}: {node} effector_type {effector!r} != {spec.effector_type!r}')

    if errors:
        raise ValueError(f'Config mismatch for model {model}:\n  ' + '\n  '.join(errors))


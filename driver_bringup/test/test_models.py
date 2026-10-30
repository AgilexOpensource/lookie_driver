"""Driver config consistency; URDF checks run only when lookie_description is a sibling checkout."""

from pathlib import Path

import pytest
import yaml

from driver_bringup.models import (
    DEFAULT_MODEL,
    MODELS,
    actuated_urdf_joints,
    default_config_paths,
    get_spec,
    validate,
)

PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_DIR / 'config'
DESCRIPTION_DIR = PACKAGE_DIR.parents[1] / 'lookie_description'
needs_urdf = pytest.mark.skipif(
    not (DESCRIPTION_DIR / 'urdf').is_dir(), reason='lookie_description not checked out')

HAND_JOINTS = (
    'thumb_metacarpal_joint',
    'thumb_proximal_joint',
    'index_proximal_joint',
    'middle_proximal_joint',
    'ring_proximal_joint',
    'pinky_proximal_joint',
)


def _urdf(model: str) -> Path:
    return DESCRIPTION_DIR / 'urdf' / f'{model}.urdf'


@needs_urdf
def test_registry_matches_description_models():
    urdf_stems = {
        p.stem for p in (DESCRIPTION_DIR / 'urdf').glob('lookie__*.urdf')
        if not p.stem.endswith('_collision')
    }
    assert set(MODELS) == urdf_stems
    assert DEFAULT_MODEL in MODELS


def test_every_config_file_is_referenced():
    referenced = {
        path.name
        for model in MODELS
        for path in default_config_paths(CONFIG_DIR, model).values()
    }
    assert referenced == {p.name for p in CONFIG_DIR.glob('*.yaml')}


@pytest.mark.parametrize('model', sorted(MODELS))
def test_configs_consistent_without_urdf(model):
    paths = default_config_paths(CONFIG_DIR, model)
    validate(model, paths['robot_yaml'], paths['params_file'], paths['bridge_config'])


@needs_urdf
@pytest.mark.parametrize('model', sorted(MODELS))
def test_configs_match_urdf(model):
    paths = default_config_paths(CONFIG_DIR, model)
    validate(model, paths['robot_yaml'], paths['params_file'], paths['bridge_config'],
             _urdf(model))


@pytest.mark.parametrize('model', sorted(MODELS))
def test_bridge_uses_driver_side_names(model):
    paths = default_config_paths(CONFIG_DIR, model)
    bridge = yaml.safe_load(paths['bridge_config'].read_text(encoding='utf-8'))
    effector = get_spec(model).effector_type
    for module in bridge['modules']:
        if 'joint_map' not in module:
            continue
        prefix = 'left_' if module['name'] == 'left_arm' else 'right_'
        local = set(module['joint_map'])
        arm = {f'joint{i}' for i in range(1, 8)}
        if effector == 'revo2':
            assert local == arm | {prefix + j for j in HAND_JOINTS}
        else:
            assert local == arm | {'gripper'}


def test_unknown_model_rejected():
    with pytest.raises(ValueError, match='Unknown model'):
        get_spec('lookie__pf-lookie_trio_r01__ee-unknown_r01')
    with pytest.raises(ValueError, match='Unknown model'):
        default_config_paths(CONFIG_DIR, 'lookie__pf-lookie_trio_r01__ee-revo2std_r01_collision')


def test_mismatched_platform_rejected():
    neo = 'lookie__pf-lookie_neo_r01__ee-agx_gripper_r01'
    trio_paths = default_config_paths(CONFIG_DIR, 'lookie__pf-lookie_trio_r01__ee-agx_gripper_r01')
    with pytest.raises(ValueError, match='Config mismatch'):
        validate(neo, trio_paths['robot_yaml'], trio_paths['params_file'],
                 trio_paths['bridge_config'])


def test_mismatched_effector_rejected():
    model = 'lookie__pf-lookie_trio_r01__ee-revo2std_r01'
    paths = default_config_paths(CONFIG_DIR, model)
    gripper = default_config_paths(CONFIG_DIR, 'lookie__pf-lookie_trio_r01__ee-agx_gripper_r01')
    with pytest.raises(ValueError, match='effector_type'):
        validate(model, paths['robot_yaml'], gripper['params_file'], paths['bridge_config'])


@needs_urdf
def test_mismatched_bridge_rejected_by_urdf():
    model = 'lookie__pf-lookie_trio_r01__ee-revo2std_r01'
    paths = default_config_paths(CONFIG_DIR, model)
    gripper = default_config_paths(CONFIG_DIR, 'lookie__pf-lookie_trio_r01__ee-agx_gripper_r01')
    with pytest.raises(ValueError, match='not actuated in URDF'):
        validate(model, paths['robot_yaml'], paths['params_file'], gripper['bridge_config'],
                 _urdf(model))


@needs_urdf
def test_limit_wider_than_urdf_rejected(tmp_path):
    model = 'lookie__pf-lookie_neo_r01__ee-agx_gripper_r01'
    paths = default_config_paths(CONFIG_DIR, model)
    motors = yaml.safe_load(paths['robot_yaml'].read_text(encoding='utf-8'))
    upper = actuated_urdf_joints(_urdf(model))['torso_joint1'][1]
    motors['lift']['joints'][0]['upper_limit'] = upper + 0.1
    robot_yaml = tmp_path / 'motors.yaml'
    robot_yaml.write_text(yaml.safe_dump(motors), encoding='utf-8')
    with pytest.raises(ValueError, match='exceed URDF'):
        validate(model, robot_yaml, paths['params_file'], paths['bridge_config'], _urdf(model))


@needs_urdf
@pytest.mark.parametrize('model', sorted(MODELS))
def test_mimic_and_wheel_joints_not_actuated(model):
    joints = actuated_urdf_joints(_urdf(model))
    assert not any('distal' in name or 'gripper_joint' in name for name in joints)
    assert not any('wheel' in name for name in joints)

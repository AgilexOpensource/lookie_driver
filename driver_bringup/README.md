# driver_bringup

Starts the Lookie head, torso, dual arms and joint bridge for a given model. Install and build: see [lookie_driver](../README.md).

## Models

| Platform | Torso | Models (`ee-agx_gripper_r01` / `ee-revo2std_r01`) |
| --- | --- | --- |
| Neo | lift_ros | `lookie__pf-lookie_neo_r01__ee-<ee>` |
| Trio | fold_ros | `lookie__pf-lookie_trio_r01__ee-<ee>` |

Default: `lookie__pf-lookie_trio_r01__ee-revo2std_r01`. Unknown names are rejected.

## CAN interfaces

The default configs expect these interfaces to be up (see each driver package README for setup):

| Platform | Head | Torso | Arms |
| --- | --- | --- | --- |
| Neo | `can_head` | `can_lift` | `can_left_arm`, `can_right_arm` |
| Trio | `can_head` | `can_torso` | `can_left_arm`, `can_right_arm` |

## Launch

```bash
ros2 launch driver_bringup lookie.launch.py model:=lookie__pf-lookie_neo_r01__ee-agx_gripper_r01

# Joint bridge only
ros2 launch driver_bringup joint_bridge.launch.py bridge_config:=/abs/path/to/<model>_joint_bridge.yaml
```

`robot_yaml`, `params_file` and `bridge_config` override the default config files (absolute paths). Configs are checked for consistency with the model before any node starts.

## Topics

The joint bridge merges module joint states into `/<platform>/feedback/joint_states` and accepts commands on `/<platform>/control/joint_states`, where `<platform>` is `neo` or `trio`. Joint names follow the Lookie URDF; Revo2 mimic joints (`*_distal`) are not bridged.

## Configuration

| File | Content |
| --- | --- |
| `config/lookie__pf-lookie_{neo,trio}_r01_motors.yaml` | Head and torso motors |
| `config/<model>_ros.yaml` | Head, torso and arm node parameters (50 Hz) |
| `config/<model>_joint_bridge.yaml` | Namespace, topics, rate (50 Hz) and per-module joint mapping |

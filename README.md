# lookie_driver

ROS 2 drivers for the Lookie robot.

## Packages

| Package | Description |
| --- | --- |
| [driver_bringup](driver_bringup/README.md) | Whole-robot bringup: head, torso, dual arms and joint bridge |
| [head_ros](https://github.com/AgilexOpensource/head_ros) | 2-DOF head |
| [lift_ros](https://github.com/AgilexOpensource/lift_ros) | Lift column and waist pitch (Neo) |
| [fold_ros](https://github.com/AgilexOpensource/fold_ros) | 4-DOF fold module (Trio) |
| [motor_ros](https://github.com/AgilexOpensource/motor_ros) | Shared motor bridge for `agx_motor_sdk` joint groups |
| [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros/tree/ros2) | AgileX robotic arm driver |

See each package README for CAN setup and safety notes.

## Install

Commands target ROS 2 Humble; on Jazzy append `--break-system-packages` to `pip3 install`.

```bash
# Python SDKs: pyAgxArm (arms), agx_motor_sdk (head/lift/fold), bc-stark-sdk (Revo2 hands only)
pip3 install git+https://github.com/agilexrobotics/pyAgxArm.git
pip3 install git+https://github.com/kehuanjack/agx_motor_sdk.git
pip3 install bc-stark-sdk

mkdir -p ~/lookie_ws/src && cd ~/lookie_ws/src
git clone --recurse-submodules https://github.com/AgilexOpensource/lookie_driver.git
# Already cloned without submodules: git -C lookie_driver submodule update --init --recursive

cd ~/lookie_ws
colcon build --packages-up-to driver_bringup
source install/setup.bash
```

## Run

```bash
ros2 launch driver_bringup lookie.launch.py model:=lookie__pf-lookie_trio_r01__ee-revo2std_r01
```

See [driver_bringup](driver_bringup/README.md) for models, topics and configuration.

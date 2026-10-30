#!/usr/bin/env python3
"""Merge module joint_states into robot-space names; split robot control back."""

import os
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState

DEFAULT_FEEDBACK_TOPIC = "feedback/joint_states"
DEFAULT_CONTROL_TOPIC = "control/joint_states"
DEFAULT_FEEDBACK_RATE = 50.0


def _abs_topic(path: str) -> str:
    path = str(path).strip()
    if not path:
        raise ValueError("topic path must not be empty")
    return path if path.startswith("/") else f"/{path}"


def _robot_topic(namespace: str, relative: str) -> str:
    ns = str(namespace).strip("/")
    rel = str(relative).lstrip("/")
    return f"/{ns}/{rel}" if ns else f"/{rel}"


def _parse_joint_mapping(raw: dict, module_name: str) -> Tuple[Dict[str, str], Set[str]]:
    """Build local_name -> robot_name map from config only (no naming conventions in code)."""
    joint_map = raw.get("joint_map")
    if joint_map is not None:
        if not isinstance(joint_map, dict):
            raise ValueError(f"module '{module_name}': joint_map must be a mapping")
        mapping = {str(k): str(v) for k, v in joint_map.items()}
        if not mapping:
            raise ValueError(f"module '{module_name}': joint_map is empty")
        return mapping, set(mapping.values())

    joints = raw.get("joints")
    if joints is None:
        joints = raw.get("joint_names")
    if joints is not None:
        names = [str(j) for j in joints]
        if not names:
            raise ValueError(f"module '{module_name}': joints list is empty")
        return {name: name for name in names}, set(names)

    raise ValueError(
        f"module '{module_name}': set 'joint_map' (local->robot) or 'joints' (identity)"
    )


def _parse_module_topics(raw: dict, module_name: str) -> Tuple[str, str]:
    namespace = str(raw.get("namespace", "")).strip("/")
    feedback = raw.get("feedback_topic")
    control = raw.get("control_topic")

    if feedback is None:
        if not namespace:
            raise ValueError(
                f"module '{module_name}': feedback_topic or namespace is required"
            )
        feedback = _robot_topic(namespace, DEFAULT_FEEDBACK_TOPIC)
    if control is None:
        if not namespace:
            raise ValueError(
                f"module '{module_name}': control_topic or namespace is required"
            )
        control = _robot_topic(namespace, DEFAULT_CONTROL_TOPIC)

    return _abs_topic(str(feedback)), _abs_topic(str(control))


def _remap_joint_state(msg: JointState, local_to_robot: Dict[str, str]) -> JointState:
    out = JointState()
    out.header = msg.header
    for i, name in enumerate(msg.name):
        robot_name = local_to_robot.get(name)
        if robot_name is None:
            continue
        out.name.append(robot_name)
        if i < len(msg.position):
            out.position.append(msg.position[i])
        if i < len(msg.velocity):
            out.velocity.append(msg.velocity[i])
        if i < len(msg.effort):
            out.effort.append(msg.effort[i])
    return out


def _append_field(dst: JointState, src: JointState, index: int) -> None:
    dst.name.append(src.name[index])
    if index < len(src.position):
        dst.position.append(src.position[index])
    if index < len(src.velocity):
        dst.velocity.append(src.velocity[index])
    if index < len(src.effort):
        dst.effort.append(src.effort[index])


@dataclass
class BridgeModule:
    name: str
    feedback_topic: str
    control_topic: str
    local_to_robot: Dict[str, str]
    robot_to_local: Dict[str, str]
    robot_joints: Set[str]
    latest_feedback: Optional[JointState] = field(default=None, repr=False)
    feedback_stamp: Optional[float] = field(default=None, repr=False)
    control_pub: object = field(default=None, repr=False)


class JointBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("joint_bridge_node")

        config_path = self.declare_parameter("config_file", "").get_parameter_value().string_value
        if not config_path:
            raise RuntimeError("joint_bridge_node requires parameter 'config_file'")

        config_path = os.path.expanduser(config_path)
        if not os.path.isabs(config_path):
            raise RuntimeError(f"config_file must be an absolute path: {config_path}")
        if not os.path.isfile(config_path):
            raise RuntimeError(f"config_file not found: {config_path}")

        with open(config_path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}

        robot_ns = str(cfg.get("namespace", "")).strip("/")
        feedback_rel = str(cfg.get("feedback_topic", DEFAULT_FEEDBACK_TOPIC)).lstrip("/")
        control_rel = str(cfg.get("control_topic", DEFAULT_CONTROL_TOPIC)).lstrip("/")

        self._robo_feedback_topic = _robot_topic(robot_ns, feedback_rel)
        self._robo_control_topic = _robot_topic(robot_ns, control_rel)

        feedback_rate = float(cfg.get("feedback_rate", DEFAULT_FEEDBACK_RATE))
        self.declare_parameter("feedback_rate", feedback_rate)
        feedback_rate = float(self.get_parameter("feedback_rate").value)
        if feedback_rate <= 0.0:
            raise ValueError(f"feedback_rate must be > 0, got {feedback_rate}")

        timeout = float(cfg.get("feedback_timeout_s", 0.5))
        self._feedback_timeout_s = float(self.declare_parameter("feedback_timeout_s", timeout).value)
        if not math.isfinite(self._feedback_timeout_s) or self._feedback_timeout_s <= 0.0:
            raise ValueError("feedback_timeout_s must be finite and positive")

        self._modules: List[BridgeModule] = []
        module_order: List[str] = []
        claimed_robot_joints: Dict[str, str] = {}

        for raw in cfg.get("modules") or []:
            if not isinstance(raw, dict):
                continue
            name = str(raw.get("name", "")).strip()
            if not name:
                continue

            local_to_robot, robot_joints = _parse_joint_mapping(raw, name)
            for robot_joint in robot_joints:
                owner = claimed_robot_joints.get(robot_joint)
                if owner is not None and owner != name:
                    raise ValueError(
                        f"robot joint '{robot_joint}' claimed by both '{owner}' and '{name}'"
                    )
                claimed_robot_joints[robot_joint] = name

            feedback_topic, control_topic = _parse_module_topics(raw, name)
            module = BridgeModule(
                name=name,
                feedback_topic=feedback_topic,
                control_topic=control_topic,
                local_to_robot=local_to_robot,
                robot_to_local={v: k for k, v in local_to_robot.items()},
                robot_joints=robot_joints,
            )
            self._modules.append(module)
            module_order.append(name)

        if not self._modules:
            raise RuntimeError(f"No modules defined in {config_path}")

        self._feedback_pub = self.create_publisher(JointState, self._robo_feedback_topic, 10)
        self._control_sub = self.create_subscription(
            JointState,
            self._robo_control_topic,
            self._on_robo_control,
            10,
        )

        for module in self._modules:
            self.create_subscription(
                JointState,
                module.feedback_topic,
                lambda msg, mod=module: self._on_module_feedback(mod, msg),
                10,
            )
            module.control_pub = self.create_publisher(JointState, module.control_topic, 10)

        self._feedback_timer = self.create_timer(
            1.0 / feedback_rate,
            self._publish_merged_feedback,
        )

        self.get_logger().info(
            "joint_bridge ready: feedback=%s @ %.1f Hz, control=%s, modules=%s"
            % (
                self._robo_feedback_topic,
                feedback_rate,
                self._robo_control_topic,
                module_order,
            )
        )

    def _on_module_feedback(self, module: BridgeModule, msg: JointState) -> None:
        if not msg.name or len(msg.name) != len(msg.position) or not all(math.isfinite(value) for value in msg.position):
            return
        module.latest_feedback = _remap_joint_state(msg, module.local_to_robot)
        module.feedback_stamp = time.monotonic()

    def _publish_merged_feedback(self) -> None:
        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()
        now = time.monotonic()

        for module in self._modules:
            fb = module.latest_feedback
            if (fb is None or not fb.name or module.feedback_stamp is None
                    or now - module.feedback_stamp > self._feedback_timeout_s):
                continue

            for i in range(len(fb.name)):
                _append_field(merged, fb, i)

        if not merged.name:
            return

        self._feedback_pub.publish(merged)

    def _on_robo_control(self, msg: JointState) -> None:
        if not msg.name:
            return

        per_module: Dict[str, JointState] = {m.name: JointState() for m in self._modules}
        stamp = self.get_clock().now().to_msg()

        for i, robot_name in enumerate(msg.name):
            for module in self._modules:
                if robot_name not in module.robot_joints:
                    continue
                local_name = module.robot_to_local.get(robot_name)
                if local_name is None:
                    continue

                out = per_module[module.name]
                out.name.append(local_name)
                if i < len(msg.position):
                    out.position.append(msg.position[i])
                if i < len(msg.velocity):
                    out.velocity.append(msg.velocity[i])
                if i < len(msg.effort):
                    out.effort.append(msg.effort[i])
                break

        for module in self._modules:
            out = per_module[module.name]
            if not out.name:
                continue
            out.header.stamp = stamp
            module.control_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

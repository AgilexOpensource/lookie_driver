from types import SimpleNamespace
from unittest.mock import patch

from rclpy.clock import Clock
from sensor_msgs.msg import JointState

from driver_bringup.joint_bridge_node import BridgeModule, JointBridgeNode


def bridge():
    module = BridgeModule("arm", "/feedback", "/control", {"local": "robot"}, {"robot": "local"}, {"robot"})
    messages = []
    node = SimpleNamespace(
        _modules=[module], _feedback_timeout_s=0.5,
        _feedback_pub=SimpleNamespace(publish=messages.append), get_clock=Clock)
    return node, module, messages


def test_cached_module_feedback_expires_even_while_bridge_timer_runs():
    node, module, messages = bridge()
    with patch("driver_bringup.joint_bridge_node.time.monotonic", return_value=1.0):
        JointBridgeNode._on_module_feedback(node, module, JointState(name=["local"], position=[0.4]))
        JointBridgeNode._publish_merged_feedback(node)
    assert messages[-1].name == ["robot"]
    assert list(messages[-1].position) == [0.4]
    with patch("driver_bringup.joint_bridge_node.time.monotonic", return_value=1.6):
        JointBridgeNode._publish_merged_feedback(node)
    assert len(messages) == 1


def test_invalid_feedback_does_not_refresh_cached_measurement():
    node, module, messages = bridge()
    with patch("driver_bringup.joint_bridge_node.time.monotonic", return_value=1.0):
        JointBridgeNode._on_module_feedback(node, module, JointState(name=["local"], position=[0.4]))
    with patch("driver_bringup.joint_bridge_node.time.monotonic", return_value=1.6):
        JointBridgeNode._on_module_feedback(node, module, JointState(name=["local"], position=[float("nan")]))
        JointBridgeNode._on_module_feedback(node, module, JointState(name=["local"], position=[]))
        JointBridgeNode._publish_merged_feedback(node)
    assert module.feedback_stamp == 1.0 and messages == []


def test_fresh_module_resumes_after_expiry():
    node, module, messages = bridge()
    with patch("driver_bringup.joint_bridge_node.time.monotonic", return_value=2.0):
        JointBridgeNode._on_module_feedback(node, module, JointState(name=["local"], position=[0.7]))
        JointBridgeNode._publish_merged_feedback(node)
    assert list(messages[-1].position) == [0.7]
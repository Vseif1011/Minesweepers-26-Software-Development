"""Signed drive mixing and the zero/disarm -> zero/arm -> drive handshake."""
import math

from minesweeper_interfaces.msg import MotorStatus
import pytest
import rclpy
from sensor_msgs.msg import Joy

from minesweeper_laptop import stm32_teleop
from minesweeper_laptop.stm32_teleop import MotorSession, mix_targets


@pytest.mark.parametrize('demand,expected', [
    ((1., 0., 0.), [.2, .2, .2, .2, 0.]),
    ((-1., 0., 0.), [-.2, -.2, -.2, -.2, 0.]),
    ((0., -1., 0.), [.2, .2, -.2, -.2, 0.]),
    ((0., 1., 0.), [-.2, -.2, .2, .2, 0.]),
    ((0., 0., 1.), [0., 0., 0., 0., .15]),
    ((0., 0., -1.), [0., 0., 0., 0., -.15]),
    ((1., 1., 0.), [0., 0., .2, .2, 0.]),
])
def test_forward_reverse_twist_and_independent_arm(demand, expected):
    assert mix_targets(*demand, [0, 1], [2, 3], [1]*5, .2, .15) == pytest.approx(expected)


def test_motor_groups_and_polarities_are_configurable():
    assert mix_targets(0., 1., 1., [0, 2], [1, 3], [-1, 1, -1, 1, -1], .2, .15) == pytest.approx(
        [.2, .2, .2, .2, -.15])
    with pytest.raises(ValueError):
        mix_targets(math.nan, 0., 0., [0, 1], [2, 3], [1]*5, .2, .15)


def status(controller, now, sequence=0, *, ready=False, armed=False, session=42, hardware=True):
    controller.on_status(MotorStatus(session=session, sequence=sequence, hardware_ok=hardware,
                                    ready=ready, armed=armed), now)


def active_controller():
    controller = MotorSession()
    status(controller, 10.0)
    controller.on_joy([0.]*5, False, True, 10.0)
    controller.on_joy([0.]*5, True, True, 10.01)
    disarm = controller.step(10.01, 10001)
    assert not disarm.arm and list(disarm.target) == [0.]*5
    status(controller, 10.02, disarm.sequence, ready=True)
    arm = controller.step(10.02, 10002)
    assert arm.arm and list(arm.target) == [0.]*5
    status(controller, 10.03, arm.sequence, armed=True)
    controller.on_joy([.2, .2, -.2, -.2, .15], True, False, 10.03)
    command = controller.step(10.03, 10003)
    assert command.arm and list(command.target) == pytest.approx([.2, .2, -.2, -.2, .15])
    return controller, command


def test_nonzero_targets_wait_for_both_mcu_acknowledgements():
    controller = MotorSession()
    status(controller, 10.0)
    controller.on_joy([0.]*5, False, True, 10.0)
    controller.on_joy([0.]*5, True, True, 10.01)
    first = controller.step(10.01, 10001)
    controller.on_joy([.2]*5, True, False, 10.02)
    assert not controller.step(10.02, 10002).arm
    status(controller, 10.03, first.sequence, ready=True)
    arm = controller.step(10.03, 10003)
    assert arm.arm and not any(arm.target)
    assert not any(controller.step(10.04, 10004).target)
    status(controller, 10.05, arm.sequence, armed=True)
    assert list(controller.step(10.05, 10005).target) == pytest.approx([.2]*5)


def test_sequence_and_creation_time_are_not_replayed():
    controller, previous = active_controller()
    command = controller.step(10.04, 10004)
    assert command.session == previous.session == 42
    assert command.sequence > previous.sequence
    assert command.issued_at_ms == 10004


def test_button_release_stops_every_channel_immediately():
    controller, _ = active_controller()
    controller.on_joy([.2]*5, False, False, 10.04)
    stopped = controller.step(10.04, 10004)
    assert not stopped.arm and not any(stopped.target)
    controller.on_joy([.2]*5, True, False, 10.05)
    assert not controller.step(10.05, 10005).arm


def test_starting_with_button_held_or_lever_off_center_cannot_arm():
    controller = MotorSession()
    status(controller, 10.0)
    controller.on_joy([0.]*5, True, True, 10.0)
    assert not controller.step(10.0, 10000).arm
    controller.on_joy([0., 0., 0., 0., .15], False, False, 10.01)
    controller.on_joy([0., 0., 0., 0., .15], True, False, 10.02)
    assert not controller.step(10.02, 10002).arm


@pytest.mark.parametrize('loss', ['status', 'joystick', 'session', 'hardware', 'unexpected_disarm'])
def test_input_or_connection_loss_requires_release_and_neutral_before_rearming(loss):
    controller, command = active_controller()
    now = 10.04
    if loss == 'status':
        now = 10.3
        assert controller.step(now, 10300) is None
        status(controller, now, command.sequence, armed=True)
    elif loss == 'joystick':
        now = 10.3
        status(controller, now, command.sequence, armed=True)
    elif loss == 'session':
        status(controller, now, session=43)
    elif loss == 'hardware':
        status(controller, now, command.sequence, hardware=False)
    else:
        status(controller, now, command.sequence, armed=False)
    stopped = controller.step(now, 10301)
    assert stopped is not None and not stopped.arm and not any(stopped.target)
    status(controller, now+.01, stopped.sequence, session=controller.session, ready=True)
    controller.on_joy([0.]*5, True, True, now+.01)
    assert not controller.step(now+.01, 10302).arm
    controller.on_joy([0.]*5, False, True, now+.02)
    controller.on_joy([0.]*5, True, True, now+.03)
    assert controller.phase == 'prepare' and not controller.locked


def test_ack_timeout_and_sequence_exhaustion_do_not_retry_arming_forever():
    controller = MotorSession()
    status(controller, 10.)
    controller.on_joy([0.]*5, False, True, 10.)
    controller.on_joy([0.]*5, True, True, 10.01)
    controller.step(10.01, 10001)
    status(controller, 11.1)
    controller.on_joy([0.]*5, True, True, 11.1)
    assert not controller.step(11.1, 11100).arm and controller.locked
    controller.sequence = 0xffffffff
    assert controller.step(11.1, 11100) is None


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '94')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    original = stm32_teleop.param
    monkeypatch.setattr(stm32_teleop, 'param', lambda instance, name, default:
                        '' if name == 'joystick_device' else original(instance, name, default))
    rclpy.init()
    instance = stm32_teleop.Stm32Teleop()
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def joy(node, axes, age=0.):
    msg = Joy(axes=axes, buttons=[0, 1]+[0]*10)
    ns = node.get_clock().now().nanoseconds-int(age*1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
    return msg


def test_default_is_preview_and_fourth_axis_only_drives_motor_five(node):
    assert node.pub.topic_name == '/motor/setpoints_preview'
    node.on_joy(joy(node, [1., 0., 0., 1., 0., 0.]))  # lateral axis 0 is intentionally unused
    assert node.control.targets == pytest.approx([0., 0., 0., 0., .15])


@pytest.mark.parametrize('axes,age', [([0.]*6, 2.), ([0., math.nan, 0., 0.], 0.), ([0.], 0.)])
def test_stale_nonfinite_and_truncated_joystick_packets_inhibit_control(node, axes, age):
    node.on_joy(joy(node, axes, age))
    assert node.control.locked and node.control.joy_at == float('-inf')

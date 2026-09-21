"""Test request-driven movement on an isolated ROS domain."""

import time

import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from control_dashboard.srv import MoveToPose
from robot_kinematics import generateH, H06
from robot_motion.turnNode import TurnNode


@pytest.fixture
def robot():
    context = Context()
    rclpy.init(context=context, domain_id=192)
    node = TurnNode(context=context)
    node.timer.cancel()  # Tick deterministically without a connected robot.
    probe = Node('move_test_client', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(probe)
    events = []
    report = node.report

    def record(state, percent, message):
        events.append((state, percent, message))
        report(state, percent, message)

    node.report = record
    try:
        yield node, probe, executor, events
    finally:
        executor.shutdown()
        probe.destroy_node()
        node.destroy_node()
        rclpy.shutdown(context=context)


def feedback(node, degrees):
    msg = JointState()
    msg.name = list(node.joint_convention.names)[::-1]
    msg.position = node.joint_convention.model_to_ros(degrees)[::-1].tolist()
    node.on_joint_states(msg)


def prepare(node, degrees):
    # Limits are read from the same standard URDF fields as robot_state_publisher.
    xml = '<robot name="test">' + ''.join(
        f'<joint name="{name}"><limit lower="-3.2" upper="3.2"/></joint>'
        for name in node.joint_convention.names) + '</robot>'
    node.on_robot_description(String(data=xml))
    feedback(node, degrees)


def request_for(degrees, linear=False):
    transform = H06(*degrees)
    request = MoveToPose.Request()
    p, q = request.target.position, request.target.orientation
    p.x, p.y, p.z = map(float, transform[:3, 3] / 1000.0)
    q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(transform[:3, :3]).as_quat())
    request.move_linear = linear
    return request


def test_service_moves_from_feedback_and_only_succeeds_after_arrival(robot):
    node, probe, executor, events = robot
    start = np.array([10., -10., 20., 10., 20., -10.])
    target = start + [2., 0., 0., 0., 0., 0.]
    assert not node.active and node.joint_pos == []
    prepare(node, start)
    client = probe.create_client(MoveToPose, '/robot/move_to_pose')
    assert client.wait_for_service(timeout_sec=3)
    future = client.call_async(request_for(target, linear=True))
    deadline = time.monotonic() + 8
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert future.done() and future.result().accepted
    np.testing.assert_allclose(node.joint_pos[0], node.joint_convention.model_to_ros(start))
    rejected = node.move_to_pose(request_for(start), MoveToPose.Response())
    assert not rejected.accepted and 'busy' in rejected.message
    for _ in range(node.joint_posLen):
        node.timer_callback()
    node.timer_callback()
    assert events[-1][0] == 'settling' and events[-1][1] == 99
    assert node.active  # Commands finished, but feedback has not reached the goal.
    feedback(node, target)
    node.timer_callback()
    assert not node.active and events[-1][:2] == ('succeeded', 100.0)
    assert node.move_to_pose(request_for(start), MoveToPose.Response()).accepted
    np.testing.assert_allclose(node.joint_pos[0], node.joint_convention.model_to_ros(target))


def test_rejects_missing_feedback_bad_pose_and_limits(robot):
    node, _, _, events = robot
    request = request_for([0]*6)
    assert not node.move_to_pose(request, MoveToPose.Response()).accepted
    feedback(node, [0]*6)
    assert 'robot_description' in node.move_to_pose(request, MoveToPose.Response()).message
    prepare(node, [0]*6)
    request.target.orientation.w = request.target.orientation.x = 0.
    request.target.orientation.y = request.target.orientation.z = 0.
    assert not node.move_to_pose(request, MoveToPose.Response()).accepted
    request = request_for([0]*6)
    request.target.position.x = float('nan')
    assert not node.move_to_pose(request, MoveToPose.Response()).accepted
    node.joint_limits[:] = [-0.01, 0.01]
    response = node.move_to_pose(request_for([5., 0, 0, 0, 0, 0]), MoveToPose.Response())
    assert not response.accepted and 'limits' in response.message
    assert not node.active and events[-1][0] == 'failed'


def test_lost_feedback_and_settling_timeout_report_failure(robot):
    node, _, _, events = robot
    prepare(node, [0]*6)
    assert node.move_to_pose(request_for([5., 0, 0, 0, 0, 0]), MoveToPose.Response()).accepted
    node.feedback_time = time.monotonic() - 3
    node.move_started = time.monotonic() - 3
    node.timer_callback()
    assert not node.active and 'feedback lost' in events[-1][2]
    feedback(node, [0]*6)
    assert node.move_to_pose(request_for([5., 0, 0, 0, 0, 0]), MoveToPose.Response()).accepted
    node.count = node.joint_posLen
    node.settle_started = time.monotonic() - 11
    node.timer_callback()
    assert not node.active and 'Timed out' in events[-1][2]


def test_orientation_only_movel_plans_through_wrist_singularity(robot):
    node, _, _, events = robot
    start = np.zeros(6)
    prepare(node, start)
    start_transform = H06(*start)
    target = generateH(
        Rotation.from_euler('xyz', [-90.0, 0.0, 10.0], degrees=True).as_matrix(),
        start_transform[:3, 3],
    )
    request = MoveToPose.Request()
    request.target.position.x, request.target.position.y, request.target.position.z = (
        target[:3, 3] / 1000.0
    )
    (
        request.target.orientation.x,
        request.target.orientation.y,
        request.target.orientation.z,
        request.target.orientation.w,
    ) = Rotation.from_matrix(target[:3, :3]).as_quat()
    request.move_linear = True

    response = node.move_to_pose(request, MoveToPose.Response())

    assert response.accepted
    assert node.active
    assert node.joint_posLen > 1
    assert events[-1][0] == 'moving'

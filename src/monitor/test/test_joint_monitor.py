"""Exercise real ROS feedback/subscriptions without commanding the robot."""

import time

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from monitor.joint_monitor import JointMonitor
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from robot_kinematics import H06
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def test_monitor_reorders_feedback_and_applies_conventions():
    context = Context()
    rclpy.init(context=context, domain_id=191)
    node = JointMonitor(context=context, parameter_overrides=[
        Parameter('joint_directions', value=[1., -1., 1., -1., 1., -1.]),
        Parameter('joint_offsets_deg', value=[0., 10., -20., 0., 5., 90.]),
    ])
    probe = Node('monitor_test_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(probe)
    received = {}
    subscriptions = []
    for key, topic, msg_type in (
        ('angles', '/monitor/joint_angles_deg', Float64MultiArray),
        ('model', '/monitor/model_angles_deg', Float64MultiArray),
        ('pose', '/monitor/end_effector_pose', PoseStamped),
        ('rpy', '/monitor/end_effector_rpy_deg', Vector3Stamped),
    ):
        subscriptions.append(probe.create_subscription(
            msg_type, topic, lambda msg, key=key: received.__setitem__(key, msg), 10))
    publisher = probe.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
    try:
        # Invalid samples must not update the monitored state.
        node.on_joint_states(JointState(name=['Joint1'], position=[1.]))
        assert node.last_received is None
        msg = JointState()
        msg.header.stamp.sec = 123
        msg.header.stamp.nanosec = 456
        msg.name = [f'Joint{i}' for i in range(6, 0, -1)]
        msg.position = np.deg2rad([60, 50, 40, 30, 20, 10]).tolist()
        deadline = time.monotonic() + 8.0
        while len(received) < 4 and time.monotonic() < deadline:
            publisher.publish(msg)
            executor.spin_once(timeout_sec=0.02)
        assert len(received) == 4, 'Timed out receiving monitor outputs'
        np.testing.assert_allclose(received['angles'].data, [10, 20, 30, 40, 50, 60])
        model_angles = [10, -10, 10, -40, 55, 30]
        np.testing.assert_allclose(received['model'].data, model_angles)
        pose = received['pose']
        assert pose.header.frame_id == 'world'
        assert pose.header.stamp == msg.header.stamp
        expected = H06(*model_angles)
        p = pose.pose.position
        np.testing.assert_allclose([p.x, p.y, p.z], expected[:3, 3] / 1000)
        q = pose.pose.orientation
        np.testing.assert_allclose(
            Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), expected[:3, :3], atol=1e-12)
        rpy = received['rpy'].vector
        np.testing.assert_allclose(
            Rotation.from_euler('xyz', [rpy.x, rpy.y, rpy.z], degrees=True).as_matrix(),
            expected[:3, :3], atol=1e-12)
        last = node.last_received
        msg.position[0] = float('nan')
        node.on_joint_states(msg)
        assert node.last_received == last
    finally:
        executor.shutdown()
        probe.destroy_node()
        node.destroy_node()
        rclpy.shutdown(context=context)

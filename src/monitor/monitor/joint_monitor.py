"""Observe measured joint feedback and calculate the world-to-Link6 pose."""

import time

from geometry_msgs.msg import PoseStamped, Vector3Stamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_kinematics import H06
from robot_kinematics.conventions import convention_from_node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


class JointMonitor(Node):
    """Publish monitoring data only when complete, finite feedback arrives."""

    def __init__(self, **kwargs):
        super().__init__('joint_monitor', **kwargs)
        self.convention = convention_from_node(self)
        topic = self.declare_parameter('joint_states_topic', '/joint_states').value
        period = self.declare_parameter('log_period', 1.0).value
        self.stale_timeout = self.declare_parameter('stale_timeout', 2.0).value
        if period <= 0 or self.stale_timeout <= 0:
            raise ValueError('log_period and stale_timeout must be positive')
        self.angles_pub = self.create_publisher(Float64MultiArray, '/monitor/joint_angles_deg', 10)
        self.model_pub = self.create_publisher(Float64MultiArray, '/monitor/model_angles_deg', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/monitor/end_effector_pose', 10)
        self.rpy_pub = self.create_publisher(Vector3Stamped, '/monitor/end_effector_rpy_deg', 10)
        self.subscription = self.create_subscription(
            JointState, topic, self.on_joint_states, qos_profile_sensor_data)
        self.last_received = None
        self.summary = None
        self.timer = self.create_timer(period, self.log_status)
        self.get_logger().info(f'Monitoring {topic}; pose is world -> Link6 (metres/quaternion)')

    def angles_message(self, values):
        msg = Float64MultiArray()
        msg.layout.dim = [MultiArrayDimension(
            label=','.join(self.convention.names), size=6, stride=6)]
        msg.data = np.asarray(values).tolist()
        return msg

    def on_joint_states(self, msg):
        try:
            ros_angles = self.convention.ordered_positions(msg.name, msg.position)
            model_angles = self.convention.ros_to_model(ros_angles)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return

        transform = H06(*model_angles)
        rotation = Rotation.from_matrix(transform[:3, :3])
        quaternion = rotation.as_quat()  # ROS order: x, y, z, w
        xyz = transform[:3, 3] / 1000.0
        # Fixed-axis roll, pitch, yaw; quaternion remains valid at gimbal lock.
        rpy = rotation.as_euler('xyz', degrees=True)
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'world'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, xyz)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = map(float, quaternion)
        orientation = Vector3Stamped()
        orientation.header = pose.header
        orientation.vector.x, orientation.vector.y, orientation.vector.z = map(float, rpy)
        self.angles_pub.publish(self.angles_message(np.rad2deg(ros_angles)))
        self.model_pub.publish(self.angles_message(model_angles))
        self.pose_pub.publish(pose)
        self.rpy_pub.publish(orientation)
        self.last_received = time.monotonic()
        self.summary = (
            f'Joints [deg]: {np.round(np.rad2deg(ros_angles), 2).tolist()} | '
            f'XYZ [m]: {np.round(xyz, 4).tolist()} | '
            f'RPY [deg]: {np.round(rpy, 2).tolist()}')

    def log_status(self):
        if self.last_received is None:
            self.get_logger().warning('Waiting for a complete /joint_states sample')
        elif time.monotonic() - self.last_received > self.stale_timeout:
            self.get_logger().warning('Joint feedback is stale; no new pose is being published')
        else:
            self.get_logger().info(self.summary)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JointMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

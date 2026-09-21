import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
from control_dashboard.srv import MoveToPose
from control_dashboard.msg import MoveProgress
from visualization_msgs.msg import Marker
from robot_kinematics import s_curve, generateH, IK, H06, joint_to_cartesian_motion
from robot_kinematics.conventions import convention_from_node
from numpy import array, isfinite, linalg
from scipy.spatial.transform import Rotation

class TurnNode(Node):
    def __init__(self, **kwargs):
        super().__init__("turn_node", **kwargs)
        self.joint_convention = convention_from_node(self)
        self.show_plot = self.declare_parameter('show_plot', False).value
        self.current_angles = None
        self.feedback_time = 0.0
        self.joint_limits = None
        self.target = None
        self.active = False
        self.joint_pos = []
        self.joint_posLen = 0
        self.count = 0
        self.settle_started = None

        self.joints = {}

        for i in range(1,7):
            self.joints[f"j{i}"] = self.create_publisher(Float64, f"/robot/joint{i}/command", 10)

        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.point_pub = self.create_publisher(
            Marker,
            "/vis/point",
            marker_qos,
        )

        self.target_marker = Marker()
        # H06/IK coordinates are expressed in the workcell world frame.
        self.target_marker.header.frame_id = "world"
        self.target_marker.ns = "robot_motion"
        self.target_marker.id = 0
        self.target_marker.type = Marker.SPHERE
        self.target_marker.action = Marker.ADD
        self.target_marker.pose.orientation.w = 1.0
        self.target_marker.scale.x = 0.05
        self.target_marker.scale.y = 0.05
        self.target_marker.scale.z = 0.05
        self.target_marker.color.r = 1.0
        self.target_marker.color.g = 0.15
        self.target_marker.color.b = 0.05
        self.target_marker.color.a = 1.0
        self.marker_timer = self.create_timer(1.0, self.publish_target_marker)
        self.progress_pub = self.create_publisher(MoveProgress, '/robot/move_progress', marker_qos)
        self.feedback_sub = self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, qos_profile_sensor_data)
        self.description_sub = self.create_subscription(
            String, '/robot_description', self.on_robot_description, marker_qos)
        self.move_service = self.create_service(MoveToPose, '/robot/move_to_pose', self.move_to_pose)

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.report('idle', 0.0, 'Ready for a target; waiting for measured joints and robot description')
        self.get_logger().info('Ready: /robot/move_to_pose')

    def on_joint_states(self, msg):
        try:
            self.current_angles = self.joint_convention.ordered_positions(msg.name, msg.position)
            self.feedback_time = time.monotonic()
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)

    def on_robot_description(self, msg):
        try:
            joints = {joint.get('name'): joint for joint in ET.fromstring(msg.data).findall('joint')}
            self.joint_limits = array([
                [float(joints[name].find('limit').get('lower')),
                 float(joints[name].find('limit').get('upper'))]
                for name in self.joint_convention.names])
        except (ET.ParseError, KeyError, AttributeError, TypeError, ValueError) as error:
            self.joint_limits = None
            self.get_logger().error(f'Cannot read joint limits: {error}')

    def report(self, state, percent, message):
        self.progress_pub.publish(MoveProgress(state=state, percent=float(percent), message=message))

    def move_to_pose(self, request, response):
        if self.active:
            response.message = 'Robot is busy; wait for the current move to finish'
            return response
        if self.current_angles is None or time.monotonic() - self.feedback_time > 2.0:
            response.message = 'No fresh /joint_states feedback'
            return response
        if self.joint_limits is None:
            response.message = 'Waiting for joint limits on /robot_description'
            return response
        p, q = request.target.position, request.target.orientation
        try:
            xyz = array([p.x, p.y, p.z]) * 1000.0
            quaternion = array([q.x, q.y, q.z, q.w])
            if not isfinite(xyz).all() or not isfinite(quaternion).all():
                raise ValueError('Target pose must contain finite numbers')
            if linalg.norm(quaternion) < 1e-8:
                raise ValueError('Target orientation quaternion must be nonzero')
            end = generateH(Rotation.from_quat(quaternion).as_matrix(), xyz)
            self.report('planning', 0.0, 'Calculating trajectory')
            self.plan_move(end, request.move_linear)
        except (ValueError, RuntimeError, linalg.LinAlgError) as error:
            response.message = str(error)
            self.report('failed', 0.0, response.message)
            return response

        self.target = end
        self.target_marker.pose = request.target
        q = self.target_marker.pose.orientation
        q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(end[:3, :3]).as_quat())
        self.publish_target_marker()
        self.active = True
        self.move_started = time.monotonic()
        self.settle_started = None
        response.accepted = True
        response.message = 'Move accepted; follow /robot/move_progress for completion'
        self.report('moving', 0.0, 'Executing trajectory')
        return response

    def plan_move(self, end, move_linear):
        start_angles = self.joint_convention.ros_to_model(self.current_angles)
        ik, converged = IK(end, start_angles)
        if not converged:
            raise RuntimeError("Target inverse kinematics did not converge")
        curve = s_curve(start_angles, ik, 60, 60, True, move_linear=move_linear)

        self.joint_pos = []
        self.velocity = []
        self.acc = []
        self.position = []
        for state in curve:
            joint_position = state["position"]
            self.joint_pos.append(self.joint_convention.model_to_ros(joint_position))
            if self.show_plot:
                xyz_velocity, xyz_acceleration = joint_to_cartesian_motion(
                    joint_position, state['velocity'], state['acceleration'])
                self.position.append(H06(*joint_position)[:3, 3])
                self.velocity.append(xyz_velocity)
                self.acc.append(xyz_acceleration)
        positions = array(self.joint_pos)
        if (not isfinite(positions).all()
                or (positions < self.joint_limits[:, 0]).any()
                or (positions > self.joint_limits[:, 1]).any()):
            raise ValueError('Planned trajectory exceeds URDF joint limits')
        if self.show_plot:
            import matplotlib.pyplot as plt
            from .plotter import plot_velocity_and_acceleration
            plot_velocity_and_acceleration(self.position, self.velocity, self.acc)
            plt.show(block=False)
        self.joint_posLen = len(self.joint_pos)
        self.count = 0

    def publish_target_marker(self):
        if self.target is None:
            return
        self.target_marker.header.stamp = self.get_clock().now().to_msg()
        self.point_pub.publish(self.target_marker)

    def timer_callback(self):
        if not self.active:
            return
        if time.monotonic() - self.feedback_time > 2.0:
            # Planning uses this executor; allow queued feedback to arrive first.
            if self.count == 0 and time.monotonic() - self.move_started < 2.0:
                return
            self.active = False
            self.report('failed', 99.0 * self.count / self.joint_posLen, 'Joint feedback lost')
            return
        if self.count < self.joint_posLen:
            pos = self.joint_pos[self.count]
            for i in range(1,7):
                msg = Float64()
                msg.data = float(pos[i-1])
                self.joints[f"j{i}"].publish(msg)
            self.count += 1
            if self.count % 10 == 0:
                self.report('moving', 99.0 * self.count / self.joint_posLen, 'Executing trajectory')
            return
        if self.settle_started is None:
            self.settle_started = time.monotonic()
            self.report('settling', 99.0, 'Waiting for measured target position and orientation')
        measured = H06(*self.joint_convention.ros_to_model(self.current_angles))
        position_error = linalg.norm(measured[:3, 3] - self.target[:3, 3])
        orientation_error = Rotation.from_matrix(
            self.target[:3, :3] @ measured[:3, :3].T).magnitude()
        if position_error < 5.0 and orientation_error < 0.03:
            self.active = False
            self.report('succeeded', 100.0, 'Measured target reached (within 5 mm and 0.03 rad)')
        elif time.monotonic() - self.settle_started > 10.0:
            self.active = False
            self.report('failed', 99.0, 'Timed out waiting for measured target pose')

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TurnNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exited with code 0")

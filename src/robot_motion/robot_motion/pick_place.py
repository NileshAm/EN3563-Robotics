"""Feedback-gated, one-shot upright pick and place for the workcell."""
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
from control_dashboard.srv import MoveToPose
from .turnNode import TurnNode


class PickPlace(TurnNode):
    def __init__(self):
        super().__init__()
        self.attached = False
        self.box = None
        self.box_time = 0.0
        self.phase = 'waiting'
        self.step = 0
        self.deadline = time.monotonic() + 30
        self.motion_result = None
        self.grip = self.create_publisher(Bool, '/gripper/close', 10)
        self.create_subscription(Bool, '/gripper/attached', self.grasp_feedback, 10)
        self.create_subscription(Pose, '/box_b/pose', self.box_feedback, 10)
        # Link6 +X points down. The 120 mm tool reaches the top of Box B.
        self.rotation = Rotation.from_euler('y', 90, degrees=True)
        self.steps = [
            ('approach', [0.45, -0.2, 0.47], False),
            ('descend', [0.45, -0.2, 0.17], True),
            ('attach', None, None),
            ('lift', [0.45, -0.2, 0.47], True),
            ('transfer', [0.5, 0.5, 0.47], True),
            ('place', [0.5, 0.5, 0.372], True),
            ('release', None, None),
            ('retreat', [0.5, 0.5, 0.47], True),
            ('verify', None, None),
        ]
        self.sequence_timer = self.create_timer(0.1, self.advance)

    def grasp_feedback(self, msg):
        self.attached = msg.data

    def box_feedback(self, msg):
        self.box = msg
        self.box_time = time.monotonic()

    def report(self, state, percent, message):
        super().report(state, percent, message)
        self.motion_result = state

    def fail(self, message):
        self.phase = 'failed'
        self.active = False
        self.get_logger().error(message)
        self.report('failed', 0.0, message)
        # Keep the jaws closed after a carrying failure; do not drop the object.

    def advance(self):
        if self.phase in ('failed', 'done'):
            return
        now = time.monotonic()
        if now > self.deadline:
            self.fail(f'Pick and place timed out during {self.phase}')
            return
        if self.phase == 'waiting':
            if (self.current_angles is None or self.joint_limits is None or
                    self.box is None or now - self.box_time > 2):
                return
            p = self.box.position
            if np.linalg.norm(np.array([p.x, p.y, p.z]) - [0.45, -0.2, 0.025]) > 0.01:
                self.fail('Box B is not at its start location; restart the simulation')
                return
            self.initial_orientation = Rotation.from_quat([
                self.box.orientation.x, self.box.orientation.y,
                self.box.orientation.z, self.box.orientation.w])
            self.phase = 'next'
        if self.phase == 'moving':
            if self.motion_result == 'failed':
                self.fail('Motion failed; sequence stopped')
                return
            if 3 <= self.step <= 5:
                if not self.attached or now - self.box_time > 2:
                    self.fail('Grasp or Box B feedback lost')
                    return
                q = self.box.orientation
                angle = (Rotation.from_quat([q.x, q.y, q.z, q.w]) *
                         self.initial_orientation.inv()).magnitude()
                if angle > 0.08:
                    self.fail('Box B orientation changed by more than 0.08 rad')
                    return
            if self.active or self.motion_result != 'succeeded':
                return
            self.step += 1
            self.phase = 'next'
        if self.phase in ('attach', 'release'):
            want = self.phase == 'attach'
            self.grip.publish(Bool(data=want))
            if self.attached != want or (not want and now - self.grip_started < 0.8):
                return
            self.step += 1
            self.phase = 'next'
        if self.phase == 'verify':
            if now < self.verify_after:
                return
            p, q = self.box.position, self.box.orientation
            error = np.linalg.norm(np.array([p.x, p.y, p.z]) - [0.5, 0.5, 0.225])
            angle = (Rotation.from_quat([q.x, q.y, q.z, q.w]) *
                     self.initial_orientation.inv()).magnitude()
            if now - self.box_time > 2 or error > 0.01 or angle > 0.08 or self.attached:
                self.fail(f'Placement verification failed: position error {error:.4f} m')
            else:
                self.phase = 'done'
                self.report('succeeded', 100.0, 'Box B verified upright on Box A')
                self.get_logger().info('COMPLETE: Box B verified at (0.5, 0.5, 0.225) m')
            return
        if self.phase != 'next':
            return
        name, xyz, linear = self.steps[self.step]
        self.get_logger().info(f'Pick and place: {name}')
        self.deadline = now + 180
        if xyz is None:
            self.phase = name
            self.grip_started = now
            self.deadline = now + 10
            if name == 'verify':
                self.verify_after = now + 2
            elif name == 'attach':
                self.grip.publish(Bool(data=True))
            else:
                self.grip.publish(Bool(data=False))
            return
        request = MoveToPose.Request()
        request.target.position.x, request.target.position.y, request.target.position.z = xyz
        q = request.target.orientation
        q.x, q.y, q.z, q.w = map(float, self.rotation.as_quat())
        request.move_linear = linear
        response = self.move_to_pose(request, MoveToPose.Response())
        if not response.accepted:
            self.fail(response.message)
            return
        self.phase = 'moving'
        self.deadline = time.monotonic() + 180


def main(args=None):
    rclpy.init(args=args)
    node = PickPlace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

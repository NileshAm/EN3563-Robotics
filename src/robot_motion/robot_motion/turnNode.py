import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from .velocityProfile import s_curve
from .Homogenous import generateH, IK, H06, joint_to_cartesian_motion
from numpy import zeros, deg2rad
import matplotlib.pyplot as plt
from .plotter import plot_velocity_and_acceleration

class TurnNode(Node):
    def __init__(self):
        super().__init__("turn_node")

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

        target_position_mm = [547.497705,0,815.002259] 
        self.target_marker = Marker()
        # H06/IK coordinates are expressed in the workcell world frame.
        self.target_marker.header.frame_id = "world"
        self.target_marker.ns = "robot_motion"
        self.target_marker.id = 0
        self.target_marker.type = Marker.SPHERE
        self.target_marker.action = Marker.ADD
        self.target_marker.pose.position.x = target_position_mm[0] / 1000.0
        self.target_marker.pose.position.y = target_position_mm[1] / 1000.0
        self.target_marker.pose.position.z = target_position_mm[2] / 1000.0
        self.target_marker.pose.orientation.w = 1.0
        self.target_marker.scale.x = 0.05
        self.target_marker.scale.y = 0.05
        self.target_marker.scale.z = 0.05
        self.target_marker.color.r = 1.0
        self.target_marker.color.g = 0.15
        self.target_marker.color.b = 0.05
        self.target_marker.color.a = 1.0
        self.publish_target_marker()
        self.marker_timer = self.create_timer(1.0, self.publish_target_marker)
        self.get_logger().info("Marker Published")

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Timer initialized.")

        end = generateH(
            [[1,0,0], [0, 1, 0], [0, 0, 1]],
            target_position_mm,
        )
        ik = IK(end, zeros(6,))[0]
        curve = s_curve(zeros((6,)), ik, 30, 30, True, move_linear=True)

        self.joint_pos = []
        self.velocity = []
        self.acc = []
        self.position = []
        for state in curve:
            # print(state["position"])
            joint_position = state["position"]
            xyz_velocity, xyz_acceleration = joint_to_cartesian_motion(
                joint_position,
                state["velocity"],
                state["acceleration"],
            )

            self.position.append(H06(*joint_position)[:3, 3])
            self.joint_pos.append(deg2rad(joint_position))
            self.velocity.append(xyz_velocity)
            self.acc.append(xyz_acceleration)
        plot_velocity_and_acceleration(self.position, self.velocity, self.acc)
        plt.show(block=False)
        self.joint_posLen = len(self.joint_pos)
        self.count = 0

    def publish_target_marker(self):
        self.target_marker.header.stamp = self.get_clock().now().to_msg()
        self.point_pub.publish(self.target_marker)

    def timer_callback(self):
        if self.count < self.joint_posLen:
            pos = self.joint_pos[self.count]
            print(f"Timer {self.count} : (",end="")
            for i in range(1,7):
                msg = Float64()
                msg.data = pos[i-1]
                self.joints[f"j{i}"].publish(msg)
                print(msg.data, end=",")
            print(")")
            self.count += 1

        # if self.state:
        #     angles = [0.4, -0.32, -0.41, 1.19, -0.69, -1.18]
        # else:
        #     angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # self.state = not(self.state)
        # for i in range(1,7):
            

def main(args=None):
    rclpy.init(args=args)
    node = TurnNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exited with code 0")

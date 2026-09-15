import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TurnNode(Node):
    def __init__(self):
        super().__init__("turn_node")

        self.joints = {}

        for i in range(1,7):
            self.joints[f"j{i}"] = self.create_publisher(Float64, f"/robot/joint{i}/command", 10)

        timer_period = 5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Timer initialized.")

        self.state = True

    def timer_callback(self):
        if self.state:
            angles = [0.4, -0.32, -0.41, 1.19, -0.69, -1.18]
        else:
            angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.state = not(self.state)
        for i in range(1,7):
            msg = Float64()
            msg.data = angles[i-1]
            self.joints[f"j{i}"].publish(msg)
            

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
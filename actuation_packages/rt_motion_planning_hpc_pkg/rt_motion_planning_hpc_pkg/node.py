import rclpy
from rclpy.node import Node
from rt_motion_planning_hpc_msgs.msg import HpcOutputData, HpcInputData

class MyNode(Node):
    def __init__(self):
        super().__init__("node")

        # Publisher to /hpc_output_data
        self.publisher = self.create_publisher(HpcOutputData, "/hpc_output_data", 1)
        self.timer = self.create_timer(1.0, self.publish_message)

        # Subscriber to /hpc_input_data
        self.subscription = self.create_subscription(
            HpcInputData,
            "/hpc_input_data",
            self.input_callback,
            1
        )

        self.get_logger().info("Robot node started!")

    def publish_message(self):
        msg = HpcOutputData()
        msg.s = 1.0
        msg.ss = 2.0
        msg.sss = 3.0
        msg.d = 0.5
        msg.dd = 0.1
        msg.ddd = 0.01
        msg.velocity = 1.0
        msg.timestep = 0.1
        msg.desired_velocity = 1.5
        self.publisher.publish(msg)
        self.get_logger().info("Published HPCOutputData message.")

    def input_callback(self, msg: HpcInputData):
        self.get_logger().info(f"Received HPCInputData:")
        self.get_logger().info(f"  cost = {msg.cost}")
        self.get_logger().info(f"  feasibility = {msg.feasibility}")
        self.get_logger().info(f"  x = {list(msg.x)}")
        self.get_logger().info(f"  y = {list(msg.y)}")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

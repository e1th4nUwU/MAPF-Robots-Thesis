import rclpy
from rclpy.node import Node

class DStarExecutor(Node):
    def __init__(self):
        super().__init__('d_star_executor')
        self.get_logger().info('d_star_executor arrancó')

def main(args=None):
    rclpy.init(args=args)
    node = DStarExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

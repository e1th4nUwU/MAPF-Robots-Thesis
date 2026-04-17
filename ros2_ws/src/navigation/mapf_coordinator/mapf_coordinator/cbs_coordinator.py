import rclpy
from rclpy.node import Node

class CbsCoordinatorNode(Node):
    def __init__(self):
        super().__init__('cbs_coordinator')
        self.get_logger().info('cbs_coordinator arrancó')

def main(args=None):
    rclpy.init(args=args)
    node = CbsCoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

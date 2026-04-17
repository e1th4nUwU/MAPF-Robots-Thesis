import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

ROBOT_NAMES = ['alvin', 'teodoro', 'simon']

class WhcaCoordinatorNode(Node):
    def __init__(self):
        super().__init__('whca_coordinator')
        self._goals = {}  # {robot_name: PoseStamped}

        # Escuchar goals de RViz, GUI o CLI
        for robot in ROBOT_NAMES:
            self.create_subscription(
                PoseStamped,
                f'/{robot}/goal_pose',
                lambda msg, r=robot: self._goal_cb(msg, r),
                10)

        self.get_logger().info('whca_coordinator listo — esperando goals...')

    def _goal_cb(self, msg, robot_name):
        self._goals[robot_name] = msg
        self.get_logger().info(
            f'Goal recibido para {robot_name}: '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        # Aquí se coordinará el planning WHCA* en pasos futuros


def main(args=None):
    rclpy.init(args=args)
    node = WhcaCoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

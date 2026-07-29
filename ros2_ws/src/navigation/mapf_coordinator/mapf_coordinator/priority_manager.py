import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from navig_msgs.msg import RobotHealth

ROBOT_NAMES = ['alvin', 'teodoro', 'simon']

class PriorityManagerNode(Node):
    def __init__(self):
        super().__init__('priority_manager')
        self._positions = {r: None for r in ROBOT_NAMES}  # Point (x, y, z)
        self._goals     = {r: None for r in ROBOT_NAMES}  # PoseStamped

        # Suscribe to health and goals of each robot
        for robot in ROBOT_NAMES:
            self.create_subscription(RobotHealth, f'/{robot}/health',
                lambda msg, r=robot: self._health_cb(msg, r), 10)
            self.create_subscription(PoseStamped, f'/{robot}/goal_pose',
                lambda msg, r=robot: self._goal_cb(msg, r), 10)

        self._pub = self.create_publisher(String, '/robot_priorities', 10)
        self.create_timer(1.0, self._publish_priorities)
        self.get_logger().info('priority_manager arrancó')

    def _health_cb(self, msg: RobotHealth, robot: str):
        self._positions[robot] = msg.position  # geometry_msgs/Point

    def _goal_cb(self, msg: PoseStamped, robot: str):
        self._goals[robot] = msg

    def _publish_priorities(self):
        distances = {}
        for robot in ROBOT_NAMES:
            pos  = self._positions[robot]
            goal = self._goals[robot]
            # No info -> Distance 0
            if pos is None or goal is None:
                distances[robot] = 0.0
                continue
            dx = goal.pose.position.x - pos.x
            dy = goal.pose.position.y - pos.y
            distances[robot] = (dx**2 + dy**2) ** 0.5

        # Closer to goal -> higher priority.
        # Rationale: the robot nearest its goal finishes soonest and parks
        # (stops blocking corridors).  Letting it through first frees the
        # space faster than letting a farther robot pass.  Also avoids the
        # pathological "lead robot has low priority and won't move out of
        # the way of the trailing robot" deadlock observed in rocks.
        # Unassigned robots (distance == 0) are sent to the back so they
        # never block live navigators.
        def _key(r):
            d = distances[r]
            return (d == 0.0, d)  # parked/unassigned last, then ascending dist
        ordered = sorted(ROBOT_NAMES, key=_key)
        self._pub.publish(String(data=','.join(ordered)))

def main(args=None):
    rclpy.init(args=args)
    node = PriorityManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

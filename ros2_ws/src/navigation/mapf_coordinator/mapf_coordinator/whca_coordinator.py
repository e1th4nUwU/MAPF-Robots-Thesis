import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

ROBOT_NAMES = ['alvin', 'teodoro', 'simon']

class WhcaCoordinatorNode(Node):
    def __init__(self):
        super().__init__('whca_coordinator')
        self.declare_parameter('replan_period_sec', 5.0)
        self.declare_parameter('stagger_sec', 0.8)

        self._goals     = {}
        self._done      = set()
        self._priorities = ROBOT_NAMES[:]
        self._initial_replan_timer = None  # debounce: one fire per goal-batch

        self._replan_pubs = {
            r: self.create_publisher(Bool, f'/{r}/whca_replan', 10)
            for r in ROBOT_NAMES
        }

        for robot in ROBOT_NAMES:
            self.create_subscription(PoseStamped, f'/{robot}/goal_pose',
                lambda msg, r=robot: self._goal_cb(msg, r), 10)
            self.create_subscription(Bool, f'/{robot}/navigation/goal_reached',
                lambda msg, r=robot: self._done_cb(msg, r), 10)

        self.create_subscription(String, '/robot_priorities',
            self._priorities_cb, 10)

        period = self.get_parameter('replan_period_sec').value
        self.create_timer(period, self._trigger_replan)

        self.get_logger().info(
            f'whca_coordinator ready — replanning every {period:.1f}s'
        )

    def _goal_cb(self, msg, robot_name):
        self._goals[robot_name] = msg
        self._done.discard(robot_name)
        self.get_logger().info(
            f'Goal for {robot_name}: '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        # Trigger an immediate coordinated replan so the first plan is
        # staggered by priority (not each robot planning independently on
        # goal receipt). Debounce: cancel previous timer so all goals that
        # arrive within 0.5 s collapse into a single _trigger_replan() call.
        if self._initial_replan_timer and self._initial_replan_timer.is_alive():
            self._initial_replan_timer.cancel()
        self._initial_replan_timer = threading.Timer(0.5, self._trigger_replan)
        self._initial_replan_timer.start()

    def _done_cb(self, msg: Bool, robot_name: str):
        if msg.data:
            self._done.add(robot_name)
            self.get_logger().info(f'{robot_name} reached its goal — stopping replans')

    def _priorities_cb(self, msg: String):
        self._priorities = msg.data.split(',')

    def _trigger_replan(self):
        active = [r for r in ROBOT_NAMES if r in self._goals and r not in self._done]
        if not active:
            return
        stagger = self.get_parameter('stagger_sec').value
        ordered = [r for r in self._priorities if r in active]
        for i, robot in enumerate(ordered):
            # One-shot delay so higher-priority robots publish reservations first
            threading.Timer(i * stagger, self._send_replan, args=[robot]).start()

    def _send_replan(self, robot: str):
        if robot not in self._done:
            self._replan_pubs[robot].publish(Bool(data=True))
            self.get_logger().debug(f'Replan → {robot}')


def main(args=None):
    rclpy.init(args=args)
    node = WhcaCoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

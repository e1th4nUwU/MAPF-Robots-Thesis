import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from navig_msgs.msg import RobotHealth

ROBOT_NAMES = ['alvin', 'teodoro', 'simon']

class PriorityManagerNode(Node):
    def __init__(self):
        super().__init__('priority_manager')
        self._positions = {r: None for r in ROBOT_NAMES}  # Point (x, y, z)
        self._goals     = {r: None for r in ROBOT_NAMES}  # PoseStamped
        self._finished  = set()                           # parked on their goal

        # Suscribe to health and goals of each robot
        for robot in ROBOT_NAMES:
            self.create_subscription(RobotHealth, f'/{robot}/health',
                lambda msg, r=robot: self._health_cb(msg, r), 10)
            self.create_subscription(PoseStamped, f'/{robot}/goal_pose',
                lambda msg, r=robot: self._goal_cb(msg, r), 10)
            self.create_subscription(Bool, f'/{robot}/navigation/goal_reached',
                lambda msg, r=robot: self._done_cb(msg, r), 10)

        self._pub = self.create_publisher(String, '/robot_priorities', 10)
        self.create_timer(1.0, self._publish_priorities)
        self.get_logger().info('priority_manager arrancó')

    def _health_cb(self, msg: RobotHealth, robot: str):
        self._positions[robot] = msg.position  # geometry_msgs/Point

    def _goal_cb(self, msg: PoseStamped, robot: str):
        # Only a DIFFERENT goal means the robot is back in the running.  The
        # topic is TRANSIENT_LOCAL, so the same goal is redelivered to any late
        # subscriber; treating that as a new assignment would resurrect a robot
        # that is parked and put it back at the head of the order.
        prev = self._goals[robot]
        changed = prev is None or (
            abs(prev.pose.position.x - msg.pose.position.x) > 1e-3
            or abs(prev.pose.position.y - msg.pose.position.y) > 1e-3)
        self._goals[robot] = msg
        if changed:
            self._finished.discard(robot)

    def _done_cb(self, msg: Bool, robot: str):
        if msg.data:
            self._finished.add(robot)

    def _publish_priorities(self):
        """Highest priority first: the robot whose goal lies DEEPEST.

        Depth is the projection of the assigned goal onto the approach
        direction — from the centroid of the robots to the centroid of their
        goals — so it does not assume the goal zone sits to the east.

        Why depth and not distance to goal.  The previous criterion ranked by
        ascending distance to goal, as a proxy for "this one finishes soonest
        and stops blocking the corridor".  The proxy breaks exactly where it
        matters: in the convergence zone the robot with the SHALLOWEST goal is
        the closest to it, so it won the highest priority, entered first and
        parked in the mouth of the zone — blocking everyone behind it, because
        cost_map paints a parked robot as an obstacle.

        Ranking by depth keeps the property the distance rule was after and
        drops the failure.  gnfc_goal_allocator hands the deepest goal to the
        robot nearest the zone, i.e. the one that leads; ranking by depth gives
        that same robot right of way, so the leader always moves first.  That
        is also what fixed the rocks deadlock, where a lead robot with low
        priority would not get out of the way of the one trailing it.

        Robots that already reached their goal, or have none yet, go last: they
        are not competing for the corridor and must never hold it.
        """
        positions = {r: self._positions[r] for r in ROBOT_NAMES}
        goals     = {r: self._goals[r]     for r in ROBOT_NAMES}

        active = [r for r in ROBOT_NAMES
                  if positions[r] is not None and goals[r] is not None
                  and r not in self._finished]

        depth = {r: 0.0 for r in ROBOT_NAMES}
        if active:
            px = sum(positions[r].x for r in active) / len(active)
            py = sum(positions[r].y for r in active) / len(active)
            gx = sum(goals[r].pose.position.x for r in active) / len(active)
            gy = sum(goals[r].pose.position.y for r in active) / len(active)
            ax, ay = gx - px, gy - py
            norm = (ax * ax + ay * ay) ** 0.5
            if norm > 1e-6:
                ax, ay = ax / norm, ay / norm
                for r in active:
                    depth[r] = (goals[r].pose.position.x * ax
                                + goals[r].pose.position.y * ay)
            else:
                # Robots already on top of their goals: fall back to distance so
                # the ordering stays defined instead of collapsing to a tie.
                for r in active:
                    dx = goals[r].pose.position.x - positions[r].x
                    dy = goals[r].pose.position.y - positions[r].y
                    depth[r] = -((dx * dx + dy * dy) ** 0.5)

        def _key(r):
            return (r in self._finished, r not in active, -depth[r])

        ordered = sorted(ROBOT_NAMES, key=_key)
        self._pub.publish(String(data=','.join(ordered)))


def main(args=None):
    rclpy.init(args=args)
    node = PriorityManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

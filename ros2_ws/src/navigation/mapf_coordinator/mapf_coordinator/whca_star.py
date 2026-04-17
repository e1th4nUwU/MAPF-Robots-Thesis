import rclpy, heapq, numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetMap
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import String
from navig_msgs.msg import SpaceTimeReservation

ROBOT_NAMES = ['alvin', 'teodoro', 'simon']

class WhcaStarNode(Node):
    def __init__(self):
        super().__init__('whca_star')
        self.declare_parameter('robot_name', '')
        self.declare_parameter('window_size', 10)
        self.declare_parameter('diagonals', False)

        self._robot_name  = self.get_parameter('robot_name').value
        self._window      = self.get_parameter('window_size').value
        self._diagonals   = self.get_parameter('diagonals').value

        # Reservations table: {robot_name: SpaceTimeReservation}
        self._reservations = {}

        # Priority list (farther from goal = higher priority)
        self._priorities = ROBOT_NAMES[:]

        # Get maps via service (same pattern as a_star.py)
        self.clt_inflated_map = self.create_client(GetMap, '/get_inflated_map')
        self.clt_cost_map     = self.create_client(GetMap, '/get_cost_map')
        [self._inflated_map, self._cost_map] = self.get_maps()

        # Keep maps fresh via subscriptions
        self.create_subscription(OccupancyGrid, '/inflated_map', self._inflated_cb, 10)
        self.create_subscription(OccupancyGrid, '/cost_map',     self._cost_cb,     10)

        # Space-time reservations and priority updates
        self.create_subscription(SpaceTimeReservation, '/space_time_reservations',
            self._reservation_cb, 10)
        self.create_subscription(String, '/robot_priorities',
            self._priorities_cb, 10)

        # Serve the same GetPlan interface that pure_pursuit uses
        self.create_service(GetPlan, 'path_planning/plan_path', self._plan_cb)

        # Path and reservation publishers
        self._path_pub = self.create_publisher(Path, 'path_planning/path', 10)
        self._res_pub  = self.create_publisher(
            SpaceTimeReservation, '/space_time_reservations', 10)

        self._last_path = Path()
        self.create_timer(0.5, lambda: self._path_pub.publish(self._last_path))

        self.get_logger().info(f'whca_star [{self._robot_name}] listo')

    # ------------------------------------------------------------------
    # Map callbacks (keep maps fresh)
    # ------------------------------------------------------------------
    def _inflated_cb(self, msg: OccupancyGrid):
        self._inflated_map = msg

    def _cost_cb(self, msg: OccupancyGrid):
        self._cost_map = msg

    # ------------------------------------------------------------------
    # Coordination callbacks
    # ------------------------------------------------------------------
    def _reservation_cb(self, msg: SpaceTimeReservation):
        # Store reservations from OTHER robots only
        if msg.robot_name != self._robot_name:
            self._reservations[msg.robot_name] = msg

    def _priorities_cb(self, msg: String):
        self._priorities = msg.data.split(',')

    # ------------------------------------------------------------------
    # Map loading (copied from a_star.py)
    # ------------------------------------------------------------------
    def get_maps(self):
        self.get_logger().info("Waiting for inflated map service...")
        while not self.clt_inflated_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for inflated map service...')
        self.get_logger().info("Waiting for cost map service...")
        while not self.clt_cost_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for cost map service...')

        future = self.clt_inflated_map.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(self, future)
        inflated_map = future.result().map
        self.get_logger().info("Got inflated map.")

        future = self.clt_cost_map.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(self, future)
        cost_map = future.result().map
        self.get_logger().info("Got cost map.")
        return [inflated_map, cost_map]

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------
    def _get_grid(self):
        info = self._inflated_map.info
        return np.reshape(np.asarray(self._inflated_map.data),
                          (info.height, info.width))

    def world_to_cell(self, point):
        info = self._inflated_map.info
        row = int((point.y - info.origin.position.y) / info.resolution)
        col = int((point.x - info.origin.position.x) / info.resolution)
        return (row, col)

    def _world_to_cell_xy(self, x, y):
        info = self._inflated_map.info
        row = int((y - info.origin.position.y) / info.resolution)
        col = int((x - info.origin.position.x) / info.resolution)
        return (row, col)

    def _cell_to_world(self, r, c):
        info = self._inflated_map.info
        x = c * info.resolution + info.origin.position.x
        y = r * info.resolution + info.origin.position.y
        return (x, y)

    def _cells_to_path(self, path_cells):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for (r, c, _) in path_cells:
            x, y = self._cell_to_world(r, c)
            msg.poses.append(
                PoseStamped(pose=Pose(position=Point(x=x, y=y, z=0.0))))
        return msg

    def _reconstruct(self, came_from, node):
        path = []
        while node in came_from:
            path.insert(0, node)
            node = came_from[node]
        path.insert(0, node)  # include start node
        return path

    # ------------------------------------------------------------------
    # Planning service callback
    # ------------------------------------------------------------------
    def _plan_cb(self, request, response):
        blocked     = self._build_blocked_set()
        start_cell  = self.world_to_cell(request.start.pose.position)
        goal_cell   = self.world_to_cell(request.goal.pose.position)
        self.get_logger().info(f'Planning from {start_cell} to {goal_cell}, blocked={len(blocked)} cells')
        path_cells  = self._whca(start_cell, goal_cell, blocked)
        self.get_logger().info(f'WHCA* found path with {len(path_cells)} points')
        self._publish_reservations(path_cells)
        self._last_path = self._cells_to_path(path_cells)
        response.plan   = self._last_path
        return response

    # ------------------------------------------------------------------
    # Blocked-cell set from higher-priority robots
    # ------------------------------------------------------------------
    def _build_blocked_set(self):
        blocked = set()
        my_idx = self._priorities.index(self._robot_name) \
                 if self._robot_name in self._priorities else 999

        for robot, res in self._reservations.items():
            if robot not in self._priorities:
                continue
            if self._priorities.index(robot) < my_idx:
                for x, y, t in zip(res.x_cells, res.y_cells, res.time_steps):
                    cell = self._world_to_cell_xy(x, y)
                    blocked.add((cell[0], cell[1], t))
        return blocked

    # ------------------------------------------------------------------
    # WHCA*: space-time A* with window
    # ------------------------------------------------------------------
    def _heuristic(self, r, c, goal):
        return abs(r - goal[0]) + abs(c - goal[1])

    def _whca(self, start, goal, blocked):
        grid = self._get_grid()
        h, w = grid.shape

        # (f, g, row, col, time)
        open_set = [(self._heuristic(start[0], start[1], goal), 0, start[0], start[1], 0)]
        came_from = {}
        g_score = {(start[0], start[1], 0): 0}

        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if self._diagonals:
            moves += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        moves.append((0, 0))  # wait action

        while open_set:
            f, g, r, c, t = heapq.heappop(open_set)

            # Reached goal
            if (r, c) == goal:
                return self._reconstruct(came_from, (r, c, t))

            # Reached window limit — return best partial path
            if t >= self._window:
                return self._reconstruct(came_from, (r, c, t))

            for dr, dc in moves:
                nr, nc, nt = r + dr, c + dc, t + 1
                if not (0 <= nr < h and 0 <= nc < w):
                    continue
                if grid[nr, nc] > 50:        # obstacle
                    continue
                if (nr, nc, nt) in blocked:  # reserved by higher-priority robot
                    continue

                new_g = g + 1
                if (nr, nc, nt) not in g_score or new_g < g_score[(nr, nc, nt)]:
                    g_score[(nr, nc, nt)] = new_g
                    came_from[(nr, nc, nt)] = (r, c, t)
                    heapq.heappush(open_set,
                        (new_g + self._heuristic(nr, nc, goal), new_g, nr, nc, nt))

        return []  # no solution found

    # ------------------------------------------------------------------
    # Publish own space-time reservations
    # ------------------------------------------------------------------
    def _publish_reservations(self, path_cells):
        msg = SpaceTimeReservation()
        msg.robot_name  = self._robot_name
        msg.window_size = self._window
        for (r, c, t) in path_cells:
            x, y = self._cell_to_world(r, c)
            msg.x_cells.append(x)
            msg.y_cells.append(y)
            msg.time_steps.append(t)
        self._res_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WhcaStarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

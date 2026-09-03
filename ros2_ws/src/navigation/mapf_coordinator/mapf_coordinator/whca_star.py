import rclpy, heapq, math, numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetMap
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import String
from navig_msgs.msg import SpaceTimeReservation
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

ROBOT_NAMES = ['alvin', 'teodoro', 'simon']

# Robot footprint from justina_base.xacro:
#   body 0.45×0.35 m, wheels add 0.04 m → total width 0.39 m
#   circumscribed radius = sqrt(0.225²+0.195²) ≈ 0.30 m
#   Minimum cells to cover physical radius: ceil(0.30/0.05) = 6
# +1 cell tracking buffer → SAFETY_CELLS = 7 (0.35 m per robot,
# 0.70 m centre-to-centre clearance). Map resolution = 0.05 m/cell.
SAFETY_CELLS = 7   # 7 cells × 0.05 m = 0.35 m margin per robot

# Reservations older than RESERVATION_TTL_SEC are considered stale and ignored.
# When a robot dies it stops publishing, so its entries expire automatically.
RESERVATION_TTL_SEC = 12.0

# Lookahead window for spatial blocking: only block cells within this many steps
# of a robot's current position along its reserved path.  40 cells × 0.05 m = 2 m.
# Cells beyond this are open for lower-priority robots to plan through (they'll
# arrive there after the higher-priority robot has moved on).
# Physical close-range safety is still enforced by EMERGENCY_STOP_M / YIELD_DISTANCE_M.
LOOKAHEAD_CELLS = 40

class WhcaStarNode(Node):
    def __init__(self):
        super().__init__('whca_star')
        self.declare_parameter('robot_name', '')
        self.declare_parameter('window_size', 10)  # kept for param compatibility; unused
        self.declare_parameter('diagonals', False)

        self._robot_name  = self.get_parameter('robot_name').value
        self._diagonals   = self.get_parameter('diagonals').value

        # Reservations table: {robot_name: (SpaceTimeReservation, timestamp)}
        self._reservations = {}

        # Priority list, highest priority first.  priority_manager orders it
        # by ASCENDING distance to goal (closest to its goal wins) — see
        # priority_manager._publish_priorities for the rationale.
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
        self._last_reservation_msg = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Republish the path every 0.5 s so RViz keeps it visible, but TRIM
        # the already-traversed portion first.  Otherwise the arrows show the
        # plan from where the robot WAS at plan time (up to replan_period=3 s
        # behind), making the path look offset from the robot in Gazebo.
        self.create_timer(0.5, self._republish_trimmed_path)
        # Re-publish last reservation every 4 s so it never expires (TTL = 12 s).
        self.create_timer(4.0, self._heartbeat_reservation)

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
        if msg.robot_name != self._robot_name:
            self._reservations[msg.robot_name] = (msg, self.get_clock().now())

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
        for (r, c) in path_cells:
            x, y = self._cell_to_world(r, c)
            msg.poses.append(
                PoseStamped(pose=Pose(position=Point(x=x, y=y, z=0.0))))
        return msg

    def _reconstruct(self, came_from, node):
        path = []
        while node in came_from:
            path.insert(0, node)
            node = came_from[node]
        path.insert(0, node)
        return path

    # ------------------------------------------------------------------
    # Occupancy predicate
    #
    # Must match a_star.py (the `independent` baseline) cell for cell, or the
    # two planners search different free spaces and the Ch. 6 comparison stops
    # being attributable to coordination alone.  a_star rejects a neighbour
    # when `grid > 40 or grid < 0`: the 40 threshold, and -1 (unknown) treated
    # as NOT traversable.  The inflated map keeps -1 for unknown cells
    # (get_inflated_map only writes 100), so the previous `> 50` test let this
    # planner drive through unknown space that the baseline refused.
    # ------------------------------------------------------------------
    OCCUPANCY_THRESHOLD = 40

    def _is_free(self, grid, r, c):
        v = grid[r, c]
        return 0 <= v <= self.OCCUPANCY_THRESHOLD

    # ------------------------------------------------------------------
    # Planning service callback
    # ------------------------------------------------------------------
    def _nearest_free(self, start, grid):
        """BFS from start to the nearest non-obstacle cell.

        Used when the robot's current position is inside an inflated obstacle
        (e.g. a just-declared dead robot's body covers the start cell).
        """
        from collections import deque
        h, w = grid.shape
        if self._is_free(grid, start[0], start[1]):
            return start
        q = deque([start])
        visited = {start}
        while q:
            r, c = q.popleft()
            if self._is_free(grid, r, c):
                return (r, c)
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1),
                            (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    q.append((nr, nc))
        return start  # no free cell found — return original as fallback

    def _plan_cb(self, request, response):
        blocked    = self._build_blocked_set()
        start_cell = self.world_to_cell(request.start.pose.position)
        goal_cell  = self.world_to_cell(request.goal.pose.position)

        grid = self._get_grid()
        h, w = grid.shape

        # Reject cells outside the grid BEFORE indexing.  world_to_cell does no
        # clamping, so a robot outside the arena yields a negative index, and
        # numpy silently wraps it round to the far edge — the planner then reads
        # a completely unrelated cell, believes the start is free, and returns
        # an empty path with no explanation.  Seen with a robot ejected past the
        # west boundary: start (45, -27) read as (45, 373).
        for label, (r, c) in (('start', start_cell), ('goal', goal_cell)):
            if not (0 <= r < h and 0 <= c < w):
                self.get_logger().error(
                    f'{label} cell ({r}, {c}) is outside the {h}x{w} map — '
                    f'the robot or its goal is outside the arena. No plan.')
                response.plan = Path()
                response.plan.header.frame_id = 'map'
                response.plan.header.stamp = self.get_clock().now().to_msg()
                return response

        # Escape start if it landed inside an obstacle (e.g. dead-robot inflation
        # covers the robot's current grid cell → all A* neighbours also blocked).
        if not self._is_free(grid, start_cell[0], start_cell[1]):
            escaped = self._nearest_free(start_cell, grid)
            if escaped != start_cell:
                self.get_logger().warn(
                    f'Start {start_cell} inside obstacle — escaped to {escaped}')
            start_cell = escaped

        self.get_logger().info(
            f'Planning from {start_cell} to {goal_cell}, '
            f'blocked={len(blocked)} spatial cells')
        path_cells = self._whca(start_cell, goal_cell, blocked)
        if not path_cells:
            self._explain_failure(grid, start_cell, goal_cell, blocked)
        if not path_cells and blocked:
            # Coordinated path infeasible (narrow corridor busy with higher-priority
            # robots).  Return empty so pure_pursuit stops and waits in
            # SM_WAIT_FOR_NEW_GOAL; the coordinator's periodic replan will retry
            # once the higher-priority robots have cleared the section.
            # Do NOT fall back to uncoordinated A* — that causes physical collision.
            self.get_logger().warn(
                'No coordinated path — stopping to yield (will retry on next replan)')
        self.get_logger().info(f'WHCA* found path with {len(path_cells)} points')
        self._publish_reservations(path_cells)
        self._last_path = self._cells_to_path(path_cells)
        response.plan   = self._last_path
        return response


    # ------------------------------------------------------------------
    # Failure diagnosis
    # ------------------------------------------------------------------
    def _explain_failure(self, grid, start, goal, blocked):
        """Say WHY a plan came back empty.

        An empty path is legitimate — it means "yield and retry" — but until
        now the log could not distinguish the three reasons it happens, which
        need completely different responses:
          * the goal is under a static obstacle      -> the scenario is wrong
          * the goal is under another robot's        -> coordination working,
            reservation halo                            just conservative
          * the goal is free but walled in           -> the corridor to it is
                                                        fully reserved
        """
        h, w = grid.shape

        def classify(r, c):
            if not (0 <= r < h and 0 <= c < w):
                return 'fuera'
            if not self._is_free(grid, r, c):
                return 'mapa'
            if (r, c) in blocked:
                return 'reservado'
            return 'libre'

        gstate = classify(*goal)
        sstate = classify(*start)

        # Free ring around the goal: how much room is actually left there.
        ring = {'libre': 0, 'reservado': 0, 'mapa': 0, 'fuera': 0}
        R = 14  # 0.70 m, the parked-robot paint radius
        for dr in range(-R, R + 1):
            for dc in range(-R, R + 1):
                if dr * dr + dc * dc <= R * R:
                    ring[classify(goal[0] + dr, goal[1] + dc)] += 1
        total = sum(ring.values())

        self.get_logger().warn(
            f'SIN RUTA  inicio {start} [{sstate}]  meta {goal} [{gstate}]  '
            f'| bloqueadas por reservación: {len(blocked)} celdas '
            f'| alrededor de la meta (r=0.70 m): '
            f'{100 * ring["libre"] / total:.0f}% libre, '
            f'{100 * ring["reservado"] / total:.0f}% reservado, '
            f'{100 * ring["mapa"] / total:.0f}% obstáculo'
        )
        if gstate == 'mapa':
            self.get_logger().warn(
                '  -> la meta está DENTRO de un obstáculo del mapa: o es un '
                'robot estacionado pintado encima, o la zona de meta está mal '
                'colocada respecto a la inflación.')
        elif gstate == 'reservado':
            self.get_logger().warn(
                '  -> la meta está dentro del halo reservado por un robot de '
                'mayor prioridad; se libera cuando ese robot avance.')
        elif ring['libre'] * 4 < total:
            self.get_logger().warn(
                '  -> la meta está libre pero rodeada: el corredor hacia ella '
                'está tomado. Es la incompletitud de la planificación '
                'priorizada, no un error de código.')

    # ------------------------------------------------------------------
    # Blocked-cell set from higher-priority robots (LOOKAHEAD SPATIAL)
    #
    # Why lookahead-only (not full-path) spatial blocking:
    # Blocking an entire 300-cell planned path with SAFETY_CELLS=7 covers
    # ~67 500 cells — in a narrow maze corridor (16 navigable cells wide)
    # this blocks the ENTIRE corridor, making it geometrically impossible
    # for lower-priority robots to find any path → deadlock.
    #
    # Lookahead fix: only block the next LOOKAHEAD_CELLS (2 m) of the
    # higher-priority robot's path from its current position.  Cells
    # beyond that are open — the lower-priority robot plans to arrive
    # there AFTER the higher-priority robot has moved on.  Physical
    # safety at close range is still enforced by EMERGENCY_STOP_M and
    # YIELD_DISTANCE_M in pure_pursuit.
    # ------------------------------------------------------------------
    def _build_blocked_set(self):
        """Returns a set of (r, c) spatial cells blocked by higher-priority robots.

        Only blocks cells within LOOKAHEAD_CELLS steps of the robot's current
        position along its reserved path.  This lets lower-priority robots plan
        through corridor segments that the higher-priority robot will vacate
        before the lower-priority robot arrives there.
        """
        blocked = set()
        my_idx = self._priorities.index(self._robot_name) \
                 if self._robot_name in self._priorities else 999
        now = self.get_clock().now()

        for robot, (res, timestamp) in self._reservations.items():
            age_sec = (now - timestamp).nanoseconds / 1e9
            if age_sec > RESERVATION_TTL_SEC:
                continue  # stale — robot likely dead, ignore its reservations
            if robot not in self._priorities:
                continue
            if self._priorities.index(robot) >= my_idx:
                continue  # equal/lower priority — this robot doesn't block us

            # Find where the robot currently sits on its reserved path via TF.
            start_idx = 0
            try:
                t = self.tf_buffer.lookup_transform(
                    'map', f'{robot}/base_link', rclpy.time.Time())
                rx = t.transform.translation.x
                ry = t.transform.translation.y
                min_d2 = float('inf')
                for i, (px, py) in enumerate(zip(res.x_cells, res.y_cells)):
                    d2 = (px - rx) ** 2 + (py - ry) ** 2
                    if d2 < min_d2:
                        min_d2 = d2
                        start_idx = i
            except TransformException:
                start_idx = 0  # TF unavailable — block from reservation start

            end_idx = min(start_idx + LOOKAHEAD_CELLS, len(res.x_cells))
            for x, y in zip(res.x_cells[start_idx:end_idx],
                            res.y_cells[start_idx:end_idx]):
                r, c = self._world_to_cell_xy(x, y)
                for dr in range(-SAFETY_CELLS, SAFETY_CELLS + 1):
                    for dc in range(-SAFETY_CELLS, SAFETY_CELLS + 1):
                        blocked.add((r + dr, c + dc))
        return blocked

    # ------------------------------------------------------------------
    # Spatially-coordinated A* (WHCA* window collapsed to spatial only)
    # ------------------------------------------------------------------
    def _heuristic(self, r, c, goal):
        """Euclidean distance to the goal, in cells.

        Must stay consistent with the step costs below: with 8-connectivity a
        Manhattan heuristic overestimates the remaining cost (a diagonal step
        advances 1 in both axes but costs sqrt(2), not 2), which makes it
        inadmissible and costs A* its optimality guarantee.  a_star.py uses the
        euclidean distance for the same reason.
        """
        return math.hypot(r - goal[0], c - goal[1])

    def _whca(self, start, goal, blocked):
        """Standard A* with an extra set of spatially-blocked cells."""
        grid = self._get_grid()
        h, w = grid.shape

        open_set = [(self._heuristic(start[0], start[1], goal), 0.0, start[0], start[1])]
        came_from = {}
        g_score   = {(start[0], start[1]): 0.0}

        # (dr, dc, step_cost) — a diagonal step covers sqrt(2) cells, so
        # charging it 1 like an orthogonal step made diagonals free and biased
        # the search towards physically longer paths.
        moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]
        if self._diagonals:
            moves += [(-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                      (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]

        while open_set:
            f, g, r, c = heapq.heappop(open_set)

            if (r, c) == goal:
                return self._reconstruct(came_from, (r, c))

            for dr, dc, step_cost in moves:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < h and 0 <= nc < w):
                    continue
                if not self._is_free(grid, nr, nc):
                    continue
                if (nr, nc) in blocked:
                    continue

                new_g = g + step_cost
                key   = (nr, nc)
                if key not in g_score or new_g < g_score[key]:
                    g_score[key] = new_g
                    came_from[key] = (r, c)
                    heapq.heappush(open_set,
                        (new_g + self._heuristic(nr, nc, goal), new_g, nr, nc))

        return []  # no solution found

    # ------------------------------------------------------------------
    # Publish own space-time reservations
    # ------------------------------------------------------------------
    def _republish_trimmed_path(self):
        """Re-publish the last path with already-traversed waypoints dropped.

        Finds the closest waypoint to the robot's current TF position and
        republishes only the suffix.  Keeps RViz arrows aligned with where
        the robot actually is, not where it was at last plan time.
        """
        if not self._last_path.poses:
            self._path_pub.publish(self._last_path)
            return
        try:
            t = self.tf_buffer.lookup_transform(
                'map', f'{self._robot_name}/base_link', rclpy.time.Time())
            rx = t.transform.translation.x
            ry = t.transform.translation.y
        except TransformException:
            self._path_pub.publish(self._last_path)
            return

        best_i, best_d2 = 0, float('inf')
        for i, ps in enumerate(self._last_path.poses):
            dx = ps.pose.position.x - rx
            dy = ps.pose.position.y - ry
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i

        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = self._last_path.poses[best_i:]
        self._path_pub.publish(msg)

    def _heartbeat_reservation(self):
        """Re-publish the last reservation so it never expires (TTL = 12 s)."""
        if self._last_reservation_msg is not None:
            orig = self._last_reservation_msg
            msg = SpaceTimeReservation()
            msg.robot_name  = orig.robot_name
            msg.window_size = orig.window_size
            msg.x_cells     = orig.x_cells
            msg.y_cells     = orig.y_cells
            msg.time_steps  = [0] * len(orig.x_cells)
            self._res_pub.publish(msg)

    def _publish_reservations(self, path_cells):
        msg = SpaceTimeReservation()
        msg.robot_name  = self._robot_name
        msg.window_size = 0  # unused — spatial blocking only
        for i, (r, c) in enumerate(path_cells):
            x, y = self._cell_to_world(r, c)
            msg.x_cells.append(x)
            msg.y_cells.append(y)
            msg.time_steps.append(i)
        self._last_reservation_msg = msg
        self._res_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WhcaStarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

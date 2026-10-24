"""
The centralised planner: one node plans every robot and feeds the plan

Replaces the per robot whca_star nodes. There is no distributed anything
here: this node holds the map, the goals and the plan of all the robots,
which is what both WHCA* and CBS assume

It drives the robots through /<robot>/goal_pose, because pure_pursuit
does not follow a Path: it takes a goal and asks its own A* how to get
there. What this node contributes is which cell each robot is sent to
and in what order

The plan is handed over in legs of `leg` cells, not cell by cell and
not in one piece. Both ends of that range fail, for measured reasons:

A goal every single cell parks the robot on arrival. It publishes
navigation/goal_reached, cost_map freezes it into the shared map as an
obstacle over its own position, and from inside that disc its own A*
can no longer plan anywhere. The margin is the arithmetic: the disc is
PARKED_ROBOT_RADIUS_M = 0.70 m, pure_pursuit stops within tol = 0.1 m
of the goal, so a 0.8 m leg leaves the next goal 0.7 m from the painted
centre, exactly on the edge. Two cells leave 1.5 m

One goal per window is safe but empty: the robot's own A* picks the
route for those 6.4 m and that A* knows nothing about the other robots,
so what keeps them apart is pure_pursuit's reactive yield and not the
cooperation this node exists to provide. A short leg keeps the robot on
the planned route, because it cannot stray far from a waypoint 1.6 m
away

What it still costs is that the robots do not advance in lockstep:
between legs the reservations say where each robot is going, not where
it stands at step k. That is the deviation from chapter 4 that has to
be written down

The window is executed, not only planned: after `window` steps the plan
is thrown away and rebuilt from where the robots actually are. That
rebuild is the W of WHCA*, and the reason a plan that conflicts beyond
the window is not a bug
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap
from navig_msgs.srv import GetGoalPoints
from std_msgs.msg import Bool

import tf2_ros

from swarm_mapf import lattice, planners, timing


class MapfPlannerNode(Node):
    """Plans the whole fleet and walks it through the plan"""

    def __init__(self):
        super().__init__('mapf_planner')

        self.declare_parameter('planner', 'whca')
        self.declare_parameter('robots', ['alvin', 'teodoro', 'simon'])
        self.declare_parameter('cell_size', 0.8)
        self.declare_parameter('window', 8)
        self.declare_parameter('leg', 2)
        self.declare_parameter('replan', True)
        self.declare_parameter('max_rounds', 20)
        self.declare_parameter('arrival_tolerance', 0.30)
        self.declare_parameter('step_timeout', 90.0)
        self.declare_parameter('region_x_min', 7.0)
        self.declare_parameter('region_x_max', 9.0)
        self.declare_parameter('region_y_min', -1.5)
        self.declare_parameter('region_y_max', 1.5)
        self.declare_parameter('min_separation', 1.0)
        self.declare_parameter('map_service', '/get_inflated_map')
        self.declare_parameter('allocation_service',
                               '/goal_allocator/get_goal_points')

        self.robots = list(self.get_parameter('robots').value)
        self.cell_size = self.get_parameter('cell_size').value
        self.window = self.get_parameter('window').value
        self.leg = max(1, self.get_parameter('leg').value)

        # Written as a check and not as a comment: the margin depends on
        # three numbers that live in three different packages, and the
        # failure it guards against is a robot that silently never moves
        reach = self.leg * self.cell_size
        if reach < 1.5:
            self.get_logger().warn(
                'Tramos de %.2f m: cost_map pinta un disco de 0.70 m sobre '
                'un robot que llega a su meta y pure_pursuit se detiene a '
                '0.1 m de ella, asi que por debajo de ~1.5 m el robot puede '
                'quedar atrapado en su propio obstaculo' % reach)

        self.plan = planners.get(self.get_parameter('planner').value)
        if self.plan is None:
            raise RuntimeError(
                'Planificador desconocido: %r. Hay %s'
                % (self.get_parameter('planner').value, planners.names()))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_client = self.create_client(
            GetMap, self.get_parameter('map_service').value)
        self.allocation_client = self.create_client(
            GetGoalPoints, self.get_parameter('allocation_service').value)

        # pure_pursuit subscribes to goal_pose latched, so a goal published
        # before it is up is not lost
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.goal_pubs = {
            name: self.create_publisher(
                PoseStamped, '/%s/goal_pose' % name, latched)
            for name in self.robots
        }
        # The whole plan, for RViz and for the figures. Nothing follows
        # it: it is what the robots are about to do, drawn
        self.path_pubs = {
            name: self.create_publisher(
                Path, '/%s/mapf/path' % name, latched)
            for name in self.robots
        }
        # pure_pursuit does not plan the moment a goal arrives: it waits
        # up to three seconds for this signal, which is how the old
        # coordinator staggered the robots. Without it every step pays
        # that timeout before anybody moves
        self.replan_pubs = {
            name: self.create_publisher(Bool, '/%s/whca_replan' % name, 10)
            for name in self.robots
        }

        self.occupancy_grid = None
        # The last cell each robot was sent to. cost_map only unpaints a
        # parked robot when the new goal differs from the previous one,
        # so a repeated goal leaves it frozen in the map
        self._last_target = {}

    # ── Startup ──────────────────────────────────────────────────────────

    def wait_for_tf(self, seconds=60.0):
        """
        Spin until every robot can be located in tf, or until time runs out

        Args:
            seconds: how long to keep trying before giving up

        Returns:
            True if every robot became reachable in tf
        """
        # time.monotonic() and not the node clock: with use_sim_time and
        # no /clock yet, a deadline taken from the node clock never passes
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(self.tf_buffer.can_transform(
                    'map', '%s/base_link' % name, rclpy.time.Time())
                    for name in self.robots):
                return True
        self.get_logger().error(
            'Tras %.0f s, tf todavia no ubica a todos los robots' % seconds)
        return False

    # ── The loop ─────────────────────────────────────────────────────────

    def run(self):
        """
        Plan, execute a window, replan, until everybody is parked

        Returns:
            True when every robot reached its goal
        """
        self.occupancy_grid = self._fetch_map()
        if self.occupancy_grid is None:
            return False

        goals = self._fetch_goals()
        if goals is None:
            return False

        goal_cells = {}
        for name, goal in goals.items():
            cell = lattice.world_to_cell(
                goal.x, goal.y, self.occupancy_grid.info, self.cell_size)
            if not lattice.is_free(*cell, self.occupancy_grid, self.cell_size):
                self.get_logger().error(
                    'La meta de %s cae en una celda ocupada del enrejado: %s. '
                    'Sube min_separation o baja cell_size' % (name, cell))
                return False
            goal_cells[name] = cell

        self._report_listeners()

        rounds = self.get_parameter('max_rounds').value
        replan = self.get_parameter('replan').value

        for round_index in range(rounds):
            starts = self._current_cells()
            if starts is None:
                return False

            if all(starts[name] == goal_cells[name] for name in self.robots):
                self.get_logger().info('Todos los robots estan en su meta')
                return True

            paths = self.plan(starts, goal_cells, self.occupancy_grid,
                              self.cell_size, window=self.window)
            if paths is None:
                self.get_logger().error(
                    'Ronda %d: el planificador no encontro solucion. '
                    'Con WHCA* esto es un bloqueo, no un error'
                    % round_index)
                return False

            self._publish_paths(paths)
            self._log_round(round_index, starts, goal_cells, paths)

            # Only the cooperative part of the plan is executed; the rest
            # is rebuilt from where the robots really ended up
            steps = len(max(paths.values(), key=len)) - 1
            horizon = min(self.window, steps) if replan else steps
            if not self._execute(paths, horizon):
                return False

            if not replan:
                return True

        self.get_logger().warn(
            'Se acabaron las %d rondas sin que todos llegaran' % rounds)
        return False

    def _report_listeners(self):
        """
        Say whether anybody is subscribed to the goal topics

        A goal nobody receives looks exactly like a robot that refuses to
        move, and the two are fixed in completely different places, so
        the count is worth one line in the log
        """
        for name in self.robots:
            goals = self.goal_pubs[name].get_subscription_count()
            replans = self.replan_pubs[name].get_subscription_count()
            level = self.get_logger().info if goals else self.get_logger().error
            level('%-10s escuchan goal_pose: %d, whca_replan: %d'
                  % (name, goals, replans))

    def _execute(self, paths, horizon):
        """
        Walk every robot along the plan, `leg` cells at a time

        All the robots finish a leg before any of them starts the next.
        That barrier is what makes the plan mean something on robots that
        move continuously: without it a fast robot runs ahead and the
        reservations stop describing where anybody is

        Args:
            paths: map from robot name to its list of (i, j) cells
            horizon: which step of the plan the window ends at

        Returns:
            True if every robot reached the end of the window
        """
        for step in range(self.leg, horizon + self.leg, self.leg):
            step = min(step, horizon)
            targets = {}

            # In the order of `paths`, which whca built in priority order,
            # so the higher priority robot is released first
            for name, path in paths.items():
                cell = path[step] if step < len(path) else path[-1]
                targets[name] = cell

                if self._last_target.get(name) == cell:
                    # cost_map ignores a goal equal to the previous one, so
                    # a robot parked there stays painted as an obstacle and
                    # its A* cannot plan out of its own disc
                    self.get_logger().warn(
                        '%s recibe otra vez la celda %s; si ya esta '
                        'estacionado ahi, cost_map no lo despinta'
                        % (name, cell))
                self._last_target[name] = cell
                self._send_goal(name, cell)

            self.get_logger().info(
                '  paso %d/%d -> %s' % (step, horizon, ', '.join(
                    '%s %s' % (name, cell) for name, cell in targets.items())))

            if not self._wait_until_arrived(targets, step):
                return False

            self._report_drift(targets)
        return True

    def _report_drift(self, targets):
        """
        Say how far each robot ended from the cell the plan sent it to

        The number worth having for chapter 6: the plan is discrete and
        the robots are not, and this is how much of that gap the window
        has to absorb. It is also the first thing to look at when robots
        start colliding in a plan that has no conflicts

        Args:
            targets: map from robot name to the (i, j) cell it was sent to
        """
        drifts = []
        for name, cell in targets.items():
            distance = self._distance_to(name, cell)
            drifts.append('%s %s' % (
                name, 'tf?' if distance is None else '%.2f m' % distance))
        self.get_logger().info('     desvio: %s' % ', '.join(drifts))

    def _wait_until_arrived(self, targets, step):
        """
        Spin until every robot is standing on the cell it was sent to

        Arrival is read from tf and not from navigation/goal_reached: the
        flag says the controller is happy, tf says where the robot is,
        and it is the position the next plan is built from

        Args:
            targets: map from robot name to the (i, j) cell it was sent to
            step: step of the plan this leg ends at, for the log

        Returns:
            True if everybody arrived before step_timeout
        """
        tolerance = self.get_parameter('arrival_tolerance').value
        deadline = time.monotonic() + self.get_parameter('step_timeout').value

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(self._distance_to(name, cell) is not None
                   and self._distance_to(name, cell) <= tolerance
                   for name, cell in targets.items()):
                return True

        late = ['%s a %s m' % (
            name,
            'tf?' if self._distance_to(name, cell) is None
            else '%.2f' % self._distance_to(name, cell))
            for name, cell in targets.items()
            if (self._distance_to(name, cell) or 1e9) > tolerance]
        self.get_logger().error(
            'Tramo hasta el paso %d: no llegaron a tiempo: %s. La '
            'tolerancia es %.2f m' % (step, '; '.join(late), tolerance))
        return False

    def _distance_to(self, name, cell):
        """
        Metres between a robot and the centre of a coarse cell

        Args:
            name: robot name
            cell: the (i, j) coarse cell

        Returns:
            The distance in metres, or None when tf cannot locate it
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', '%s/base_link' % name, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        x, y = lattice.cell_to_world(
            cell[0], cell[1], self.occupancy_grid.info, self.cell_size)
        t = tf.transform.translation
        return math.hypot(t.x - x, t.y - y)

    # ── Talking to the rest of the system ────────────────────────────────

    def _send_goal(self, name, cell):
        """
        Publish one cell of the plan as a goal for one robot

        Args:
            name: robot name
            cell: the (i, j) coarse cell it must reach next
        """
        x, y = lattice.cell_to_world(
            cell[0], cell[1], self.occupancy_grid.info, self.cell_size)
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        # The plan carries no heading: the lattice has no orientation
        msg.pose.orientation.w = 1.0
        self.goal_pubs[name].publish(msg)
        # Released in the order the robots are iterated, which is the
        # priority order, so the sequencing survives down here too
        self.replan_pubs[name].publish(Bool(data=True))

    def _publish_paths(self, paths):
        """
        Draw the current plan, one Path per robot

        Args:
            paths: map from robot name to its list of (i, j) cells
        """
        stamp = self.get_clock().now().to_msg()
        for name, cells in paths.items():
            self.path_pubs[name].publish(timing.cells_to_path(
                cells, self.occupancy_grid.info, self.cell_size,
                'map', stamp))

    def _current_cells(self):
        """
        Read where every robot is, as coarse cells

        Returns:
            A dict in priority order, or None if tf failed. The order is
            the priority: whca reads it from the order of this dict
        """
        cells = {}
        for name in self.robots:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', '%s/base_link' % name, rclpy.time.Time())
            except tf2_ros.TransformException as exc:
                self.get_logger().error('No encuentro a %s en tf: %s'
                                        % (name, exc))
                return None
            t = tf.transform.translation
            cell = lattice.world_to_cell(
                t.x, t.y, self.occupancy_grid.info, self.cell_size)
            if not lattice.is_free(*cell, self.occupancy_grid, self.cell_size):
                # It happens: the robot parks half a cell off and lands on
                # an inflated cell. Planning from there returns nothing,
                # so say which robot it was instead of failing silently
                self.get_logger().error(
                    '%s esta sobre una celda ocupada del enrejado %s'
                    % (name, cell))
                return None
            cells[name] = cell
        return cells

    def _fetch_map(self):
        """
        Ask cost_map for the inflated map

        Returns:
            The nav_msgs/OccupancyGrid, or None if the call failed
        """
        response = self._call(self.map_client, GetMap.Request(), 'mapa inflado')
        if response is None:
            return None
        grid = response.map
        self.get_logger().info(
            'Mapa inflado: %dx%d celdas de %.3f m; enrejado de %.2f m'
            % (grid.info.width, grid.info.height, grid.info.resolution,
               self.cell_size))
        return grid

    def _fetch_goals(self):
        """
        Ask the allocator which goal belongs to which robot

        Returns:
            A map from robot name to geometry_msgs/Point32, or None
        """
        positions = []
        for name in self.robots:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', '%s/base_link' % name, rclpy.time.Time())
            except tf2_ros.TransformException as exc:
                self.get_logger().error('No encuentro a %s en tf: %s'
                                        % (name, exc))
                return None
            t = tf.transform.translation
            positions.append(Point32(x=float(t.x), y=float(t.y), z=0.0))

        request = GetGoalPoints.Request()
        request.region.points = self._region_polygon()
        request.map = self.occupancy_grid
        request.robots = positions
        request.min_separation = self.get_parameter('min_separation').value

        response = self._call(self.allocation_client, request, 'asignacion')
        if response is None:
            return None
        if not response.success:
            self.get_logger().error('Sin asignacion: %s'
                                    % response.error_message)
            return None

        goals = dict(zip(self.robots, response.goals))
        for name, goal in goals.items():
            self.get_logger().info('  %-10s -> (%+.2f, %+.2f)'
                                   % (name, goal.x, goal.y))
        return goals

    def _region_polygon(self):
        """
        Build the goal region as a rectangle, counter-clockwise

        Returns:
            The four corners as geometry_msgs/Point32
        """
        x_min = self.get_parameter('region_x_min').value
        x_max = self.get_parameter('region_x_max').value
        y_min = self.get_parameter('region_y_min').value
        y_max = self.get_parameter('region_y_max').value
        corners = [(x_min, y_min), (x_max, y_min),
                   (x_max, y_max), (x_min, y_max)]
        return [Point32(x=float(x), y=float(y), z=0.0) for x, y in corners]

    def _call(self, client, request, label):
        """
        Call a service and wait for it, without blocking the executor

        This spins the node, so it must never be reached from inside a
        callback: the executor would already be busy running that
        callback and the call would never complete

        Args:
            client: the rclpy client to call
            request: the request to send
            label: name used in log messages

        Returns:
            The response, or None on timeout or failure
        """
        if not client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error('El servicio de %s no aparecio: %s'
                                    % (label, client.srv_name))
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done():
            self.get_logger().error('Sin respuesta de %s' % label)
            return None
        return future.result()

    def _log_round(self, index, starts, goals, paths):
        """
        Report one planning round, in the terms of the algorithm

        Args:
            index: which round this is
            starts: map from robot name to its current cell
            goals: map from robot name to its goal cell
            paths: the plan just produced
        """
        self.get_logger().info('── Ronda %d, ventana w=%d ──'
                               % (index, self.window))
        for name in self.robots:
            path = paths[name]
            waits = sum(1 for a, b in zip(path, path[1:]) if a == b)
            self.get_logger().info(
                '  %-10s %s -> %s  %d pasos, %d esperas'
                % (name, starts[name], goals[name], len(path) - 1, waits))


def main():
    rclpy.init()
    node = MapfPlannerNode()
    try:
        if node.wait_for_tf():
            node.run()
        # Stay alive so the published paths keep a publisher behind them
        # while RViz is being looked at
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

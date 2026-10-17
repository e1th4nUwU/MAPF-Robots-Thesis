import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point, Point32, PolygonStamped
from navig_msgs.srv import GetGoalPoints
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from swarm_mapf import allocators, grid, region


class GoalAllocatorNode(Node):
    """
    Goal assigning-service
    """
    def __init__(self):
        super().__init__('goal_allocator')

         # TODO: Use until we implement GRPA
        # RGB triplets, flattened, one per robot and in the same priority
        # order as the robots of the request. A single triplet paints
        # every goal the same colour.
        self.declare_parameter('goal_colors', [0.0, 0.8, 0.0])

        self.declare_parameter('allocator', 'nearest')
        # Allocator is the function used to assign goals to agents
        self.allocator_name = self.get_parameter('allocator').value

        self.srv = self.create_service(
            GetGoalPoints, '~/get_goal_points', self._callback)

        # Use a latched publisher so the region and markers are available 
        # before calling the service
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # Publish goal regions and markers so they can be visualized in RViz
        self.region_pub = self.create_publisher(
            PolygonStamped, '~/goal_region', latched)
        self.marker_pub = self.create_publisher(
            Marker, '~/goal_markers', latched)
        self.candidate_pub = self.create_publisher(
            Marker, '~/candidates', latched)

        self.get_logger().info(
            f"GOAL ALLOCATOR LISTO (allocator: {self.allocator_name})")


    def _validate(self, request):
        """
        Check the request is well-formed.
        Returns an error message, or None if everything is fine.
        """
        if len(request.region.points) < 3:
            return f'La region tiene {len(request.region.points)} vertices, se necesitan al menos 3'

        info = request.map.info
        if info.resolution <= 0.0:
            return f'Resolución inválida: {info.resolution}'
        
        if info.width == 0 or info.height == 0:
            return f'Retícula vacía: {info.width}x{info.height}'
        
        # The map is not a matrix; it's a flat list of size j * width + i]
        # If it isn't that size, something is wrong with the request
        if len(request.map.data) != info.width * info.height:
            return f'El mapa tiene {len(request.map.data)} celdas, se esperaban {info.width * info.height}'

        if len(request.robots) < 1:
            return 'No se recibió ningún robot'

        if request.min_separation <= 0.0:
            return f'Separación minima inválida: {request.min_separation}'

        return None


    def _callback(self, request, response):
        error = self._validate(request)
        if error:
            return self._fail(response, error)

        allocate = allocators.get(self.allocator_name)
        if allocate is None:
            return self._fail(
                response,
                f"Allocator desconocido: '{self.allocator_name}'. "
                f"Disponibles: {', '.join(allocators.names())}")

        info = request.map.info
        frame = request.map.header.frame_id or 'map'

        # Find candidate cells in the region, and check there are enough for all robots
        candidates = region.candidate_cells(
            request.region.points, request.map, request.min_separation)
        
        # Check if there are enough candidates for all robots
        # Published before anything can fail: when there are too few
        # candidates, seeing the region and the candidates in RViz is
        # the whole diagnosis
        self._publish_region(request.region, frame)
        self._publish_candidates(candidates, info, frame)

        if len(candidates) < len(request.robots):
            return self._fail(
                response,
                f'No hay suficientes candidatos para los robots. '
                f'Solo hay {len(candidates)} candidatos para '
                f'{len(request.robots)} robots')

        # Convert the robots' positions to cells, and compute the separation in cells
        robot_cells = [grid.world_to_cell(r.x, r.y, info)
                       for r in request.robots]
        
        # Compute the minimum separation in cells, so the allocator can use it
        separation_in_cells = grid.cells_for_distance(request.min_separation, info)

        # The allocator may fail, so we catch any exception and return an error message 
        # to understand why it failed and account it
        try:
            goal_cells = allocate(
                robot_cells, candidates, request.map, separation_in_cells
            )
        except Exception as exc:
            # An exception raised in a service callback propagates up to
            # spin() and kills the node, leaving the client hanging
            self.get_logger().error(f'El allocator falló: {exc}')
            return self._fail(response, f'El allocator falló: {exc}')

        if goal_cells is None:
            return self._fail(
                response, 'No existe una asignación completa para estos robots')

        # Back to metres, at the centre of each cell.
        # Second half of the snapping that costs the separation its eps,
        # see region.candidate_cells
        response.goals = [
            Point32(x=float(x), y=float(y), z=0.0)
            for x, y in (grid.cell_to_world(i, j, info) for i, j in goal_cells)]
        response.success = True
        response.error_message = ''

        self._publish_markers(response.goals, frame, request.min_separation)
        self.get_logger().info(
            f'{len(response.goals)} metas asignadas de {len(candidates)} '
            f'candidatos, con {self.allocator_name}')
        return response

    def _fail(self, response, message):
        """
        Fill the response as a failure and log why

        Args:
            response: the response being built
            message: what to report in error_message

        Returns:
            The same response, so the callback can return it directly
        """
        response.success = False
        response.error_message = message
        response.goals = []
        self.get_logger().warn(message)
        return response

    def _publish_candidates(self, cells, info, frame):
        """
        Publish every admissible candidate, assigned or not

        This is what makes a failure readable: if the answer is that
        there are too few candidates, RViz shows the region and the
        handful of cells that survived the polygon and the inflated
        map, which is the difference between "my region is too small"
        and "my region is on top of a wall"

        Args:
            cells: the (i, j) cells returned by region.candidate_cells
            info: nav_msgs/MapMetaData of the map in the request
            frame: tf frame the coordinates belong to
        """
        msg = Marker()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = 'candidates'
        msg.id = 0
        msg.type = Marker.CUBE_LIST
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.scale.x = msg.scale.y = msg.scale.z = max(info.resolution * 2.0, 0.1)
        msg.color.r = 1.0
        msg.color.g = 0.85
        msg.color.a = 0.9
        msg.points = [
            Point(x=float(x), y=float(y), z=0.05)
            for x, y in (grid.cell_to_world(i, j, info) for i, j in cells)]
        self.candidate_pub.publish(msg)

    def _publish_region(self, polygon, frame):
        """
        Publish the goal region so it can be drawn in RViz

        Args:
            polygon: the geometry_msgs/Polygon that came in the request
            frame: tf frame the coordinates belong to
        """
        msg = PolygonStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.polygon = polygon
        self.region_pub.publish(msg)

    def _publish_markers(self, goals, frame, separation):
        """
        Publish the assigned goals as a marker, one sphere per goal

        Args:
            goals: the assigned goals as geometry_msgs/Point32
            frame: tf frame the coordinates belong to
            separation: minimum distance between goals, in metres
        """
        msg = Marker()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        # Namespace
        msg.ns = 'goals'
        msg.id = 0
        msg.type = Marker.SPHERE_LIST
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.scale.x = msg.scale.y = msg.scale.z = separation
        msg.points = [Point(x=g.x, y=g.y, z=0.1) for g in goals]
        # A per-point colour list, so each goal is drawn in the colour of
        # the robot it was assigned to
        msg.colors = self._goal_colors(len(goals))
        self.marker_pub.publish(msg)

    def _goal_colors(self, count):
        """
        Build one colour per goal out of the goal_colors parameter

        Args:
            count: how many goals were assigned

        Returns:
            A list of std_msgs/ColorRGBA, one per goal. With a single
            triplet every goal gets the same colour; with fewer triplets
            than goals the list is cycled, so a mismatch still draws
        """
        flat = [float(v) for v in (self.get_parameter('goal_colors').value or [])]
        triplets = [tuple(flat[n:n + 3]) for n in range(0, len(flat) - 2, 3)]
        if not triplets:
            triplets = [(0.0, 0.8, 0.0)]
        if len(triplets) < count:
            self.get_logger().warn(
                f'goal_colors trae {len(triplets)} colores para {count} '
                f'metas; se repiten')
        return [ColorRGBA(r=triplets[n % len(triplets)][0],
                          g=triplets[n % len(triplets)][1],
                          b=triplets[n % len(triplets)][2], a=0.6)
                for n in range(count)]


def main():
    rclpy.init()
    try:
        rclpy.spin(GoalAllocatorNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        # Without this, Ctrl-C prints a full traceback and exits 1
        pass

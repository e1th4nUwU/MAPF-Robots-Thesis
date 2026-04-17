#
# MOBILE ROBOTS - FI-UNAM, 2026-2
# MAP INFLATION AND COST MAPS
#
# Instructions:
# Write the code necesary to get a cost map given an occupancy grid map and a cost radius.
# Complete the code necessary to inflate the obstacles given an occupancy grid map and
# a number of cells to inflate.
#

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from std_msgs.msg import Bool
from navig_msgs.msg import RobotHealth
import numpy

FULL_NAME = "Jorge Eithan Treviño Selles"

ROBOT_NAMES = ['alvin', 'teodoro', 'simon']

# Radius (metres) used to paint a dead robot as an obstacle on the map.
# Conservative coverage of ~0.5m robot body.
DEAD_ROBOT_RADIUS_M = 0.2

class CostMapNode(Node):
    def get_inflated_map(self, static_map, inflation_cells):
        """
        Inflates the obstacles in the static map by a specified 
        number of cells.
        
        Parameters:
        - static_map: A 2D numpy array representing the occupancy grid map, where 0 indicates free space, 
                      100 indicates occupied space, and -1 indicates unknown space.
        - inflation_cells: The number of cells by which to inflate the obstacles.
        Returns:
        - A 2D numpy array representing the inflated occupancy grid map.
        """
        self.get_logger().debug("Inflating map by " + str(inflation_cells) + " cells")
        inflated = numpy.copy(static_map)
        [height, width] = static_map.shape
        for i in range(height):
            for j in range(width):
                # If the cell is occupied, inflate it
                if static_map[i, j] > 50:
                    # Loop over the neighborhood defined by the inflation radius
                    for k1 in range(-inflation_cells, inflation_cells + 1):
                        for k2 in range(-inflation_cells, inflation_cells + 1):
                            # Check if the neighboring cell is within the bounds of the map
                            if (0 <= i+k1 < height) and (0 <= j+k2 < width):
                                # Mark the neighboring cell as occupied in the inflated map
                                inflated[i+k1, j+k2] = 100

        return inflated
    
    def get_cost_map(self, static_map, cost_radius):
        """
        Calculates a cost map for the given map.
        
        Parameters:
        - static_map: A 2D numpy array representing the occupancy grid map, where
                        0 indicates free space, 100 indicates occupied space, and -1 indicates unknown space.
        - cost_radius: The number of cells around obstacles with costs greater than zero.
        Returns:
        - A 2D numpy array representing the cost map, where each cell's value indicates
            how close it is to an obstacle (higher values indicate closer proximity).
        """
        self.get_logger().debug("Getting cost map with " + str(cost_radius) + " cells")
        cost_map = numpy.copy(static_map)
        [height, width] = static_map.shape
        # Loop through each cell in the map
        for i in range(height):
            for j in range(width):
                # If the cell is occupied, calculate the cost for its neighbors
                if static_map[i, j] > 50:
                    for k1 in range(-cost_radius, cost_radius + 1):
                        for k2 in range(-cost_radius, cost_radius + 1):
                            # Calculate the coordinates of the neighboring cell
                            ni, nj = i + k1, j + k2
                            # Check if the neighboring cell is within the bounds of the map
                            if (0 <= ni < height) and (0 <= nj < width):
                                # Calculate the Chebyshev distance from the obstacle to the neighboring cell
                                chebyshev_distance = max(abs(k1), abs(k2))
                                # Calculate the cost based on the distance
                                cost = cost_radius - chebyshev_distance + 1
                                # Update the cost map with the maximum cost
                                cost_map[ni, nj] = max(cost, cost_map[ni, nj])

        return cost_map

    def callback_inflated_map(self, request, response):
        response.map = self.inflated_map
        return response
        
    def callback_cost_map(self, request, response):
        response.map = self.cost_map
        return response

    def _paint_dead_robots(self, grid):
        """Paint dead robot positions as occupied cells on a copy of the grid."""
        info = self.map_static.info
        res  = info.resolution
        if res == 0.0:
            return numpy.copy(grid)
        ox   = info.origin.position.x
        oy   = info.origin.position.y
        radius_cells = round(DEAD_ROBOT_RADIUS_M / res)
        height, width = grid.shape

        result = numpy.copy(grid)
        for _, pos in self.dead_robots.items():
            if pos is None:
                continue
            cx = round((pos[0] - ox) / res)
            cy = round((pos[1] - oy) / res)
            for dr in range(-radius_cells, radius_cells + 1):
                for dc in range(-radius_cells, radius_cells + 1):
                    r, c = cy + dr, cx + dc
                    if 0 <= r < height and 0 <= c < width:
                        result[r, c] = 100
        return result

    def _precompute_static_maps(self):
        """Inflate and cost-grade the static map once at startup.

        callback_timer then only needs to paint small dead-robot circles on
        top of these precomputed grids, reducing per-tick work from seconds
        to milliseconds.
        """
        info = self.map_static.info
        height, width = info.height, info.width
        res = info.resolution
        raw = numpy.reshape(
            numpy.asarray(self.map_static.data, dtype='int'),
            (height, width))

        inflation_radius = self.get_parameter('inflation_radius').get_parameter_value().double_value
        cost_radius      = self.get_parameter('cost_radius').get_parameter_value().double_value

        self._inflation_cells = round(inflation_radius / res)
        self._cost_cells      = round(cost_radius      / res)
        self._dead_cells      = round(DEAD_ROBOT_RADIUS_M / res)

        self.get_logger().info(
            f'Precomputing static maps '
            f'(inflation={self._inflation_cells}c, cost={self._cost_cells}c, '
            f'dead={self._dead_cells}c) — please wait…')
        self._static_inflated = self.get_inflated_map(raw, self._inflation_cells)
        self._static_cost     = self.get_cost_map(raw, self._cost_cells)
        self.get_logger().info('Static maps precomputed.')

    def callback_timer(self):
        info   = self.map_static.info
        height = info.height
        width  = info.width
        ox     = info.origin.position.x
        oy     = info.origin.position.y
        res    = info.resolution

        inflated = numpy.copy(self._static_inflated)
        cost     = numpy.copy(self._static_cost)

        # Paint each dead robot as an obstacle + cost gradient.
        # Total inflation radius = physical footprint + configured inflation.
        total_inf  = self._dead_cells + self._inflation_cells
        total_cost = self._dead_cells + self._cost_cells

        for name, pos in self.dead_robots.items():
            if pos is None:
                continue
            # pos is in map frame (supplied by health_monitor via TF lookup).
            cx = round((pos[0] - ox) / res)
            cy = round((pos[1] - oy) / res)

            if not (0 <= cx < width and 0 <= cy < height):
                self.get_logger().warn(
                    f'[COST MAP] {name} at map coords ({cx},{cy}) out of '
                    f'bounds ({width}×{height}) — skipping paint')
                continue

            # Inflated obstacle: square of half-width total_inf
            for dr in range(-total_inf, total_inf + 1):
                for dc in range(-total_inf, total_inf + 1):
                    r, c = cy + dr, cx + dc
                    if 0 <= r < height and 0 <= c < width:
                        inflated[r, c] = 100

            # Cost gradient: value decreases linearly from the robot edge outward
            for dr in range(-total_cost, total_cost + 1):
                for dc in range(-total_cost, total_cost + 1):
                    r, c = cy + dr, cx + dc
                    if 0 <= r < height and 0 <= c < width:
                        dist = max(abs(dr), abs(dc))
                        if dist <= self._dead_cells:
                            cost[r, c] = 100
                        elif dist <= total_cost:
                            dist_from_edge = dist - self._dead_cells
                            cost_val = self._cost_cells - dist_from_edge + 1
                            cost[r, c] = max(int(cost_val), int(cost[r, c]))

        inflated_data = numpy.ravel(inflated)
        cost_data     = numpy.ravel(cost)
        now = self.get_clock().now()

        self.inflated_map = OccupancyGrid(info=info, data=inflated_data)
        self.inflated_map.header.frame_id = "map"
        self.inflated_map.header.stamp = now.to_msg()
        self.pub_inflated_map.publish(self.inflated_map)

        self.cost_map = OccupancyGrid(info=info, data=cost_data)
        self.cost_map.header.frame_id = "map"
        self.cost_map.header.stamp = now.to_msg()
        self.pub_cost_map.publish(self.cost_map)

    def __init__(self):
        super().__init__("cost_map_node")
        self.get_logger().info("INITIALIZING MAP INFLATER AND COST MAP NODE - " + FULL_NAME)
        self.clt_static_map = self.create_client(GetMap, '/map_server/map')
        self.get_logger().info("Waiting for static map service...")
        while not self.clt_static_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for static map service...')
        self.get_logger().info("Static map service is now available...")
        self.get_logger().info("Trying to get first static map...")
        self.map_static = None
        max_retries = 5
        for attempt in range(max_retries):
            try:
                future = self.clt_static_map.call_async(GetMap.Request())
                rclpy.spin_until_future_complete(self, future)
                response = future.result()
                if response and response.map and response.map.info.resolution > 0:
                    self.map_static = response.map
                    self.get_logger().info("Got static map.")
                    break
            except Exception as e:
                self.get_logger().warn(f"Attempt {attempt+1} failed: {e}")
            if self.map_static is None:
                self.get_logger().info(f"Retrying map service (attempt {attempt+1}/{max_retries})...")
                self.get_clock().sleep_for(Duration(seconds=1.0))
        self.declare_parameter('inflation_radius', 0.05)
        self.declare_parameter('cost_radius', 0.05)

        # Dead robot tracking: {robot_name: (x, y)} or None if alive
        self.dead_robots   = {name: None for name in ROBOT_NAMES}
        self.prev_alive    = {name: True  for name in ROBOT_NAMES}


        # Pre-initialize maps so service callbacks don't crash before first timer tick
        self.inflated_map = self.map_static
        self.cost_map     = self.map_static

        # Publishers — must be created before the timer fires
        self.pub_replan       = self.create_publisher(Bool,          '/replan_needed', 10)
        self.pub_inflated_map = self.create_publisher(OccupancyGrid, '/inflated_map',  10)
        self.pub_cost_map     = self.create_publisher(OccupancyGrid, '/cost_map',      10)

        # Services
        self.get_clock().sleep_for(Duration(seconds=2.0))
        self.srv_inflate_map = self.create_service(GetMap, '/get_inflated_map', self.callback_inflated_map)
        self.srv_cost_map    = self.create_service(GetMap, '/get_cost_map',     self.callback_cost_map)

        # Subscribe to each robot's health
        for name in ROBOT_NAMES:
            self.create_subscription(
                RobotHealth, f'/{name}/health',
                lambda msg, n=name: self._health_cb(msg, n), 10)

        # Precompute static inflation/cost maps once — subsequent ticks are fast
        self._precompute_static_maps()

        # Publish initial maps immediately so A* can start without waiting
        self.callback_timer()

        # Timer runs last — everything it needs is already created
        self.timer = self.create_timer(1.0, self.callback_timer)

    def _health_cb(self, msg: RobotHealth, name: str) -> None:
        """Track dead robots and react immediately on state transitions."""
        was_alive = self.prev_alive[name]
        is_alive  = msg.is_alive
        self.prev_alive[name] = is_alive

        if is_alive:
            if self.dead_robots[name] is not None:
                # Robot came back alive — remove obstacle and replan immediately
                self.dead_robots[name] = None
                self.get_logger().info(f'[COST MAP] {name} alive again — removing obstacle')
                self.callback_timer()
                self.pub_replan.publish(Bool(data=True))
        else:
            # Always keep position updated while dead
            self.dead_robots[name] = (msg.position.x, msg.position.y)
            if was_alive:
                # First detection of death — rebuild maps NOW and signal replan
                self.get_logger().warn(
                    f'[COST MAP] {name} dead at '
                    f'({msg.position.x:.2f}, {msg.position.y:.2f}) '
                    f'— rebuilding maps and requesting replan'
                )
                self.callback_timer()
                self.pub_replan.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    cost_map_node = CostMapNode()
    rclpy.spin(cost_map_node)
    cost_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

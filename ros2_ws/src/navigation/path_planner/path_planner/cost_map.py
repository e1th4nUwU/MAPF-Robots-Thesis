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
import numpy

FULL_NAME = "Jorge Eithan Treviño Selles"

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

    def callback_timer(self):
        self.map_info   = self.map_static.info
        self.map_width  = self.map_info.width
        self.map_height = self.map_info.height
        self.map_res    = self.map_info.resolution
        self.map_data = numpy.reshape(numpy.asarray(self.map_static.data, dtype='int'), (self.map_height, self.map_width))
        inflation_radius  = self.get_parameter('inflation_radius').get_parameter_value().double_value
        inflated_map_data = self.get_inflated_map(self.map_data, round(inflation_radius/self.map_res))
        inflated_map_data = numpy.ravel(numpy.reshape(inflated_map_data, (self.map_width*self.map_height, 1)))
        cost_radius  = self.get_parameter('cost_radius').get_parameter_value().double_value
        cost_map_data = self.get_cost_map(self.map_data, round(cost_radius/self.map_res))
        cost_map_data = numpy.ravel(numpy.reshape(cost_map_data, (self.map_width*self.map_height, 1)))
        self.inflated_map = OccupancyGrid(info=self.map_info, data=inflated_map_data)
        self.inflated_map.header.frame_id = "map"
        self.inflated_map.header.stamp = self.get_clock().now().to_msg()
        self.pub_inflated_map.publish(self.inflated_map)
        self.cost_map = OccupancyGrid(info=self.map_info, data=cost_map_data)
        self.cost_map.header.frame_id = "map"
        self.cost_map.header.stamp = self.get_clock().now().to_msg()
        self.pub_cost_map.publish(self.cost_map)
        return

    def __init__(self):
        super().__init__("cost_map_node")
        self.get_logger().info("INITIALIZING MAP INFLATER AND COST MAP NODE - " + FULL_NAME)
        self.clt_static_map = self.create_client(GetMap, '/map_server/map')
        self.get_logger().info("Waiting for static map service...")
        while not self.clt_static_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for static map service...')
        self.get_logger().info("Static map service is now available...")
        self.get_logger().info("Trying to get first static map...")
        future = self.clt_static_map.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.map_static = response.map
        self.get_logger().info("Got static map.")
        self.declare_parameter('inflation_radius', 0.05)
        self.declare_parameter('cost_radius', 0.05)
        self.timer = self.create_timer(1.0, self.callback_timer)
        self.get_clock().sleep_for(Duration(seconds=2.0))
        self.srv_inflate_map  = self.create_service(GetMap, '/get_inflated_map', self.callback_inflated_map)
        self.srv_cost_map  = self.create_service(GetMap, '/get_cost_map', self.callback_cost_map)
        self.pub_inflated_map = self.create_publisher(OccupancyGrid, '/inflated_map', 10)
        self.pub_cost_map = self.create_publisher(OccupancyGrid, '/cost_map', 10)


def main(args=None):
    rclpy.init(args=args)
    cost_map_node = CostMapNode()
    rclpy.spin(cost_map_node)
    cost_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

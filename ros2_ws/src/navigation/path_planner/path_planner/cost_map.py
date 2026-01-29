#
# MOBILE ROBOTS - FI-UNAM, 2026-1
# COST MAPS
#
# Instructions:
# Write the code necesary to get a cost map given
# an occupancy grid map and a cost radius.
#

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import numpy

FULL_NAME = "FULL NAME"

class CostMapNode(Node):
    def get_cost_map(self, static_map, cost_radius):
        print("Getting cost map with " + str(cost_radius) + " cells")
        cost_map = numpy.copy(static_map)
        [height, width] = static_map.shape
        #
        # TODO:
        # Write the code necessary to calculate a cost map for the given map.
        # To calculate cost, consider as example the following map:    
        # [[ 0 0 0 0 0 0]
        #  [ 0 X 0 0 0 0]
        #  [ 0 X X 0 0 0]
        #  [ 0 X X 0 0 0]
        #  [ 0 X 0 0 0 0]
        #  [ 0 0 0 X 0 0]]
        # Where occupied cells 'X' have a value of 100 and free cells have a value of 0.
        # Cost is an integer indicating how near cells and obstacles are:
        # [[ 3 3 3 2 2 1]
        #  [ 3 X 3 3 2 1]
        #  [ 3 X X 3 2 1]
        #  [ 3 X X 3 2 2]
        #  [ 3 X 3 3 3 2]
        #  [ 3 3 3 X 3 2]]
        # Cost_radius indicate the number of cells around obstacles with costs greater than zero.
        
        return cost_map
        
    def callback_cost_map(self, request, response):
        response.map = self.cost_map
        return response

    def callback_timer(self):
        self.map_info   = self.map_static.info
        self.map_width  = self.map_info.width
        self.map_height = self.map_info.height
        self.map_res    = self.map_info.resolution
        self.map_data = numpy.reshape(numpy.asarray(self.map_static.data, dtype='int'), (self.map_height, self.map_width))
        cost_radius  = self.get_parameter('cost_radius').get_parameter_value().double_value
        cost_map_data = self.get_cost_map(self.map_data, round(cost_radius/self.map_res))
        cost_map_data = numpy.ravel(numpy.reshape(cost_map_data, (self.map_width*self.map_height, 1)))
        self.cost_map = OccupancyGrid(info=self.map_info, data=cost_map_data)
        self.cost_map.header.frame_id = "map"
        self.cost_map.header.stamp = self.get_clock().now().to_msg()
        self.pub_cost_map.publish(self.cost_map)
        return

    def __init__(self):
        print("INITIALIZING COST MAP NODE - ", FULL_NAME)
        super().__init__("cost_map_node")
        self.clt_static_map = self.create_client(GetMap, '/map_server/map')
        print("Waiting for static map service...")
        while not self.clt_static_map.wait_for_service(timeout_sec=1.0):
            print('Waiting for static map service...')
        print("Static map service is now available...")
        print("Trying to get first static map...")
        future = self.clt_static_map.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.map_static = response.map
        print("Got static map.")
        self.declare_parameter('cost_radius', 0.05)
        self.srv_cost_map  = self.create_service(GetMap, '/get_cost_map', self.callback_cost_map)
        self.pub_cost_map = self.create_publisher(OccupancyGrid, '/cost_map', 10)
        self.timer = self.create_timer(1.0, self.callback_timer)


def main(args=None):
    rclpy.init(args=args)
    cost_map_node = CostMapNode()
    rclpy.spin(cost_map_node)
    cost_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

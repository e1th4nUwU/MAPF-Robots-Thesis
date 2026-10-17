"""
Test client for the goal allocation service
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Point32
from nav_msgs.srv import GetMap
from navig_msgs.srv import GetGoalPoints

import tf2_ros


class AllocationClientNode(Node):
    """One-shot client of /goal_allocator/get_goal_points"""

    def __init__(self):
        super().__init__('allocation_client')

        self.declare_parameter('robots', ['alvin', 'teodoro', 'simon'])
        self.declare_parameter('region_x_min', 7.0)
        self.declare_parameter('region_x_max', 9.0)
        self.declare_parameter('region_y_min', -1.5)
        self.declare_parameter('region_y_max', 1.5)
        self.declare_parameter('min_separation', 0.8)
        self.declare_parameter('map_service', '/get_inflated_map')
        self.declare_parameter('allocation_service',
                               '/goal_allocator/get_goal_points')

        self.robots = self.get_parameter('robots').value
        self.min_separation = self.get_parameter('min_separation').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_client = self.create_client(
            GetMap, self.get_parameter('map_service').value)
        self.allocation_client = self.create_client(
            GetGoalPoints, self.get_parameter('allocation_service').value)

    def wait_for_tf(self, seconds=30.0):
        """
        Spin until every robot can be located in tf, or until time runs out
        
        Args:
            seconds: how long to keep trying before giving up

        Returns:
            True if every robot became reachable in tf
        """
        # time.monotonic() and not get_clock(): with use_sim_time and no
        # /clock publisher yet, the node clock stays at zero and a
        # deadline computed from it is never reached
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(self.tf_buffer.can_transform(
                    'map', f'{name}/base_link', rclpy.time.Time())
                    for name in self.robots):
                return True
        self.get_logger().error(
            f'Tras {seconds:.0f} s, tf todavia no ubica a todos los robots')
        return False

    def run(self):
        """
        Assemble the request, send it, and report the answer

        Returns:
            True if goals were assigned
        """
        occupancy_grid = self._fetch_map()
        if occupancy_grid is None:
            return False

        robot_points = self._fetch_robot_positions()
        if robot_points is None:
            return False

        request = GetGoalPoints.Request()
        request.region.points = self._region_polygon()
        request.map = occupancy_grid
        request.robots = robot_points
        request.min_separation = self.min_separation

        self.get_logger().info(
            f'Pidiendo metas para {len(robot_points)} robots, '
            f's = {self.min_separation} m')
        response = self._call(self.allocation_client, request, 'asignacion')
        if response is None:
            return False

        if not response.success:
            self.get_logger().error(f'Sin asignacion: {response.error_message}')
            return False

        for name, goal in zip(self.robots, response.goals):
            self.get_logger().info(
                f'  {name:<10} -> ({goal.x:+.2f}, {goal.y:+.2f})')
        return True

    def _region_polygon(self):
        """
        Build the region as a rectangle, counter-clockwise

        Returns:
            The four corners as geometry_msgs/Point32
        """
        x_min = self.get_parameter('region_x_min').value
        x_max = self.get_parameter('region_x_max').value
        y_min = self.get_parameter('region_y_min').value
        y_max = self.get_parameter('region_y_max').value
        corners = [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]
        return [Point32(x=float(x), y=float(y), z=0.0) for x, y in corners]

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
            f'Mapa inflado: {grid.info.width}x{grid.info.height} celdas '
            f'de {grid.info.resolution} m')
        return grid

    def _fetch_robot_positions(self):
        """
        Look up each robot in tf, in the order the parameter lists them

        Returns:
            One geometry_msgs/Point32 per robot, in priority order, or
            None if any robot could not be located
        """
        points = []
        for name in self.robots:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', f'{name}/base_link', rclpy.time.Time())
            except tf2_ros.TransformException as exc:
                self.get_logger().error(f'No encuentro a {name} en tf: {exc}')
                return None
            t = tf.transform.translation
            points.append(Point32(x=float(t.x), y=float(t.y), z=0.0))
            self.get_logger().info(f'{name:<10} en ({t.x:+.2f}, {t.y:+.2f})')
        return points

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
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                f'El servicio de {label} no apareció: {client.srv_name}')
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if not future.done():
            self.get_logger().error(f'Sin respuesta de {label}')
            return None
        return future.result()


def main():
    rclpy.init()
    node = AllocationClientNode()
    try:
        if node.wait_for_tf():
            node.run()
        # Stay alive so the latched topics of the allocator keep a
        # publisher on the other side while RViz is being looked at
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

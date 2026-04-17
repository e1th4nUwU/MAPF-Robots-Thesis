#!/usr/bin/env python3
"""
Visualize the goal zone as a rectangle in RViz.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class GoalZoneVisualizerNode(Node):
    def __init__(self):
        super().__init__('goal_zone_visualizer')
        self.get_logger().info("INITIALIZING GOAL ZONE VISUALIZER")

        # Declare parameters
        self.declare_parameter('goal_zone_x_min', 7.5)
        self.declare_parameter('goal_zone_x_max', 9.0)
        self.declare_parameter('goal_zone_y_min', -1.0)
        self.declare_parameter('goal_zone_y_max', 1.0)
        self.declare_parameter('publish_rate_hz', 1.0)

        # Read parameters
        self.x_min = self.get_parameter('goal_zone_x_min').value
        self.x_max = self.get_parameter('goal_zone_x_max').value
        self.y_min = self.get_parameter('goal_zone_y_min').value
        self.y_max = self.get_parameter('goal_zone_y_max').value

        # Publisher for markers (RViz)
        self.marker_pub = self.create_publisher(Marker, 'goal_zone_marker', 10)


        # Timer to publish marker periodically
        publish_rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / publish_rate, self._publish_marker)

        self.get_logger().info(
            f"Goal zone: [{self.x_min:.1f}, {self.x_max:.1f}] × "
            f"[{self.y_min:.1f}, {self.y_max:.1f}] (RViz visualization enabled)"
        )

    def _publish_marker(self):
        """Publish a rectangle marker for the goal zone."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal_zone'
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # Rectangle outline
        marker.action = Marker.ADD

        # Rectangle corners (in order to form a closed loop)
        corners = [
            Point(x=self.x_min, y=self.y_min, z=0.1),
            Point(x=self.x_max, y=self.y_min, z=0.1),
            Point(x=self.x_max, y=self.y_max, z=0.1),
            Point(x=self.x_min, y=self.y_max, z=0.1),
            Point(x=self.x_min, y=self.y_min, z=0.1),  # Close the loop
        ]
        marker.points = corners

        # Style: bright green, 0.1m line width
        marker.scale.x = 0.1  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8  # Transparency

        marker.lifetime.sec = 2  # Disappear if not updated

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    visualizer = GoalZoneVisualizerNode()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

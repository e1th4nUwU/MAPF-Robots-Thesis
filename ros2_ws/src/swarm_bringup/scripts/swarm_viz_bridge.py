#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


SCENARIO_STARTS = {
    'towers': {
        'alvin': (-8.0, 1.5),
        'teodoro': (-8.0, 0.0),
        'simon': (-8.0, -1.5),
    },
    'rocks': {
        'alvin': (-8.0, 1.5),
        'teodoro': (-8.0, 0.0),
        'simon': (-8.0, -1.5),
    },
    'maze': {
        'alvin': (-8.0, 2.5),
        'simon': (-8.0, 1.0),
        'teodoro': (-8.0, -2.5),
    },
}


class SwarmVizBridge(Node):
    def __init__(self):
        super().__init__('swarm_viz_bridge')
        self.declare_parameter('scenario', 'towers')
        scenario = self.get_parameter('scenario').get_parameter_value().string_value
        if scenario not in SCENARIO_STARTS:
            self.get_logger().warn(f"Unknown scenario '{scenario}', fallback to towers")
            scenario = 'towers'
        self.starts = SCENARIO_STARTS[scenario]

        self.odom_pubs = {
            name: self.create_publisher(Odometry, f'/{name}/odom_map', 10)
            for name in self.starts.keys()
        }
        for name in self.starts.keys():
            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, robot=name: self._odom_cb(msg, robot),
                10,
            )

        self.goal_pub = self.create_publisher(MarkerArray, '/swarm/goal_areas', 1)
        self.goal_timer = self.create_timer(0.5, self._publish_goal_markers)

        self.get_logger().info(f"Swarm viz bridge ready for scenario: {scenario}")

    def _odom_cb(self, msg: Odometry, robot: str):
        ox, oy = self.starts[robot]
        out = copy.deepcopy(msg)
        out.header.frame_id = 'map'
        out.child_frame_id = f'{robot}/base_link'
        out.pose.pose.position.x += ox
        out.pose.pose.position.y += oy
        self.odom_pubs[robot].publish(out)

    def _publish_goal_markers(self):
        markers = MarkerArray()

        def goal_marker(mid, x, y, sx, sy):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'goal_areas'
            m.id = mid
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.02
            m.pose.orientation.w = 1.0
            m.scale.x = sx
            m.scale.y = sy
            m.scale.z = 0.04
            m.color.r = 0.1
            m.color.g = 0.9
            m.color.b = 0.2
            m.color.a = 0.45
            return m

        is_maze = abs(self.starts['alvin'][1] - 2.5) < 1e-6
        if is_maze:
            markers.markers.append(goal_marker(0, 8.5, 2.5, 1.0, 4.5))
            markers.markers.append(goal_marker(1, 8.5, -2.5, 1.0, 4.5))
        else:
            markers.markers.append(goal_marker(0, 8.0, 0.0, 1.0, 12.0))

        self.goal_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmVizBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

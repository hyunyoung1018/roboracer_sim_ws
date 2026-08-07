#!/usr/bin/env python3
"""Publish a speed-updated copy of the scaled global waypoints."""

from copy import deepcopy

import numpy as np
import rclpy
from f110_msgs.msg import WpntArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class UpdateWaypoints(Node):
    def __init__(self):
        super().__init__('waypoint_updater')
        self.declare_parameter('update_waypoints', True)
        self.declare_parameter('speed_offset', 0.0)
        self.waypoints = None
        self.s_values = None
        self.create_subscription(WpntArray, '/global_waypoints_scaled', self._waypoints_cb, 10)
        self.create_subscription(Odometry, '/car_state/odom_frenet', self._odom_cb, 10)
        self.publisher = self.create_publisher(WpntArray, '/global_waypoints_updated', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/updated_waypoints_marker', 10)
        self.create_timer(1.0, self._publish)

    def _waypoints_cb(self, msg):
        self.waypoints = deepcopy(msg)
        self.s_values = np.asarray([waypoint.s_m for waypoint in msg.wpnts])

    def _odom_cb(self, msg):
        if self.waypoints is None or not self.get_parameter('update_waypoints').value:
            return
        idx = int(np.argmin(np.abs(self.s_values - msg.pose.pose.position.x)))
        speed = msg.twist.twist.linear.x + float(self.get_parameter('speed_offset').value)
        self.waypoints.wpnts[idx].vx_mps = max(0.0, speed)

    def _publish(self):
        if self.waypoints is None:
            return
        self.waypoints.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.waypoints)
        markers = MarkerArray()
        max_speed = max(0.1, max(w.vx_mps for w in self.waypoints.wpnts))
        for waypoint in self.waypoints.wpnts:
            marker = Marker()
            marker.header = self.waypoints.header
            marker.ns = 'updated_waypoints'
            marker.id = waypoint.id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint.x_m
            marker.pose.position.y = waypoint.y_m
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 0.08
            marker.scale.z = max(0.01, waypoint.vx_mps / max_speed)
            marker.pose.position.z = marker.scale.z / 2.0
            marker.color.a = 1.0
            marker.color.g = 1.0
            markers.markers.append(marker)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = UpdateWaypoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

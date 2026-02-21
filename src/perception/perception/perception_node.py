"""Perception node for the RC car.

Processes raw 2-D LiDAR scans to detect the closest obstacle within a
configurable threshold distance and publishes its range and bearing.
"""

# Copyright (c) 2024 Maintainer
#
# SPDX-License-Identifier: MIT

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped


class PerceptionNode(Node):
    """Detect obstacles from LiDAR scans.

    Subscriptions
    -------------
    /scan : sensor_msgs/LaserScan
        Raw 2-D LiDAR scan from the robot.

    Publications
    ------------
    /obstacles : nav_msgs/PointStamped
        Closest detected obstacle encoded in the point field:
        point.x = distance (m), point.y = bearing (rad).
        Only published when an obstacle is within the threshold distance.
    """

    def __init__(self):
        super().__init__('perception_node')

        # Obstacle detection threshold (m) – configurable via ROS parameter.
        self.declare_parameter('obstacle_threshold', 1.0)

        from geometry_msgs.msg import PointStamped
        self._obstacle_pub = self.create_publisher(
            PointStamped, '/obstacles', 10
        )

        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10
        )

        self.get_logger().info('Perception node started.')

    def _scan_callback(self, msg: LaserScan) -> None:
        """Detect the closest obstacle within the threshold distance."""
        threshold = self.get_parameter('obstacle_threshold').value

        min_range = float('inf')
        min_index = -1

        for i, r in enumerate(msg.ranges):
            if msg.range_min <= r <= msg.range_max and r < min_range:
                min_range = r
                min_index = i

        if min_index == -1 or min_range > threshold:
            return

        bearing = msg.angle_min + min_index * msg.angle_increment

        obstacle_msg = PointStamped()
        obstacle_msg.header.stamp = self.get_clock().now().to_msg()
        obstacle_msg.header.frame_id = 'lidar_link'
        # Encode distance as x, bearing as y for easy consumption downstream.
        obstacle_msg.point.x = float(min_range)
        obstacle_msg.point.y = float(bearing)
        obstacle_msg.point.z = 0.0
        self._obstacle_pub.publish(obstacle_msg)

        self.get_logger().debug(
            f'Obstacle detected: distance={min_range:.2f} m, '
            f'bearing={math.degrees(bearing):.1f} deg'
        )


def main(args=None):
    """Entry point for the perception node."""
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

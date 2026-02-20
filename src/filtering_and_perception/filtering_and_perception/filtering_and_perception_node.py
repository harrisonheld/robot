"""Filtering and perception node for the RC car.

Consumes raw sensor data (LiDAR scan, IMU) and publishes:
- A filtered odometry estimate (using a simple complementary filter).
- A point cloud of detected obstacles derived from the LiDAR scan.
"""

# Copyright (c) 2024 Maintainer
#
# SPDX-License-Identifier: MIT

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header


class FilteringAndPerceptionNode(Node):
    """Process raw sensor data into filtered state estimates and obstacles.

    Subscriptions
    -------------
    /scan : sensor_msgs/LaserScan
        Raw 2-D LiDAR scan from the robot.
    /imu : sensor_msgs/Imu
        Raw IMU measurement.

    Publications
    ------------
    /odometry/filtered : nav_msgs/Odometry
        Filtered odometry estimate.
    /obstacles : nav_msgs/Odometry
        Closest detected obstacle distance encoded in the position field
        (x = distance, y = bearing in radians).
    """

    def __init__(self):
        super().__init__('filtering_and_perception_node')

        # Obstacle detection threshold (m) – configurable via ROS parameter.
        self.declare_parameter('obstacle_threshold', 1.0)

        self._odom_pub = self.create_publisher(
            Odometry, '/odometry/filtered', 10
        )
        self._obstacle_pub = self.create_publisher(
            Odometry, '/obstacles', 10
        )

        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10
        )
        self._imu_sub = self.create_subscription(
            Imu, '/imu', self._imu_callback, 10
        )

        # Simple state: heading estimate from IMU integration.
        self._heading = 0.0
        self._last_imu_time = None

        self.get_logger().info('Filtering and perception node started.')

    def _imu_callback(self, msg: Imu) -> None:
        """Integrate gyroscope Z to maintain a heading estimate."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_imu_time is None:
            self._last_imu_time = now
            return
        dt = now - self._last_imu_time
        self._last_imu_time = now

        self._heading += msg.angular_velocity.z * dt
        # Normalise heading to [-π, π].
        self._heading = math.atan2(
            math.sin(self._heading), math.cos(self._heading)
        )

        # Publish a minimal odometry message with the current heading.
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.z = math.sin(self._heading / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._heading / 2.0)
        self._odom_pub.publish(odom)

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

        obstacle_msg = Odometry()
        obstacle_msg.header = Header()
        obstacle_msg.header.stamp = self.get_clock().now().to_msg()
        obstacle_msg.header.frame_id = 'base_link'
        # Encode distance as x, bearing as y for easy consumption downstream.
        obstacle_msg.pose.pose.position = Point(
            x=float(min_range), y=float(bearing), z=0.0
        )
        self._obstacle_pub.publish(obstacle_msg)

        self.get_logger().debug(
            f'Obstacle detected: distance={min_range:.2f} m, '
            f'bearing={math.degrees(bearing):.1f} deg'
        )


def main(args=None):
    """Entry point for the filtering and perception node."""
    rclpy.init(args=args)
    node = FilteringAndPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

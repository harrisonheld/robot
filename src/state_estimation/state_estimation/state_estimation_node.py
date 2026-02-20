"""State estimation node for the RC car.

Integrates IMU gyroscope data to maintain a heading estimate and publishes a
filtered odometry message.
"""

# Copyright (c) 2024 Maintainer
#
# SPDX-License-Identifier: MIT

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class StateEstimationNode(Node):
    """Estimate robot heading by integrating IMU gyroscope measurements.

    Subscriptions
    -------------
    /imu : sensor_msgs/Imu
        Raw IMU measurement from the robot.

    Publications
    ------------
    /odometry/filtered : nav_msgs/Odometry
        Filtered odometry estimate (orientation from gyro integration).
    """

    def __init__(self):
        super().__init__('state_estimation_node')

        self._odom_pub = self.create_publisher(
            Odometry, '/odometry/filtered', 10
        )

        self._imu_sub = self.create_subscription(
            Imu, '/imu', self._imu_callback, 10
        )

        # Heading estimate (rad), integrated from gyro Z.
        self._heading = 0.0
        self._last_imu_time = None

        self.get_logger().info('State estimation node started.')

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

        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.z = math.sin(self._heading / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._heading / 2.0)
        self._odom_pub.publish(odom)


def main(args=None):
    """Entry point for the state estimation node."""
    rclpy.init(args=args)
    node = StateEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

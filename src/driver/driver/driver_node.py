"""Driver node for the RC car.

Subscribes to Ackermann drive commands from the planning layer and translates
them into throttle and steering actuator commands.  In simulation the commands
are forwarded to the Gazebo Ackermann drive plugin via the standard
``/cmd_vel`` topic.
"""

# Copyright (c) 2024 Maintainer
#
# SPDX-License-Identifier: MIT

import math

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist


class DriverNode(Node):
    """Translate high-level Ackermann commands into actuator commands.

    Subscriptions
    -------------
    /ackermann_cmd : ackermann_msgs/AckermannDriveStamped
        Desired steering angle (rad) and speed (m/s) from the planning layer.

    Publications
    ------------
    /cmd_vel : geometry_msgs/Twist
        Velocity command forwarded to the Gazebo differential/Ackermann plugin.
    """

    def __init__(self):
        super().__init__('driver_node')

        # Maximum allowable speed (m/s) – configurable via ROS parameter.
        self.declare_parameter('max_speed', 2.0)
        # Maximum steering angle (rad) – configurable via ROS parameter.
        self.declare_parameter('max_steering_angle', 0.5)
        # Wheel base (m) used for Ackermann → differential conversion.
        self.declare_parameter('wheelbase', 0.3)

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._ackermann_sub = self.create_subscription(
            AckermannDriveStamped,
            '/ackermann_cmd',
            self._ackermann_callback,
            10,
        )

        self.get_logger().info('Driver node started.')

    def _ackermann_callback(self, msg: AckermannDriveStamped) -> None:
        """Convert an Ackermann command to a Twist and publish it."""
        max_speed = self.get_parameter('max_speed').value
        max_steering = self.get_parameter('max_steering_angle').value
        wheelbase = self.get_parameter('wheelbase').value

        speed = float(
            max(-max_speed, min(max_speed, msg.drive.speed))
        )
        steering_angle = float(
            max(-max_steering, min(max_steering, msg.drive.steering_angle))
        )

        twist = Twist()
        twist.linear.x = speed

        # Approximate angular velocity from steering angle using bicycle model:
        #   turning_radius = wheelbase / tan(steering_angle)
        if abs(steering_angle) > 1e-6 and abs(wheelbase) > 1e-6:
            turning_radius = wheelbase / math.tan(abs(steering_angle))
            angular_z = speed / turning_radius
            if steering_angle < 0:
                angular_z = -angular_z
        else:
            angular_z = 0.0

        twist.angular.z = angular_z

        self._cmd_vel_pub.publish(twist)


def main(args=None):
    """Entry point for the driver node."""
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

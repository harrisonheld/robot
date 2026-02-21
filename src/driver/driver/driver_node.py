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
    """Translate tank drive commands into actuator commands.

    Subscriptions
    -------------
    /tank_cmd : geometry_msgs/Twist
        Desired left and right wheel velocities (m/s) from the planning layer.

    Publications
    ------------
    /cmd_vel : geometry_msgs/Twist
        Velocity command forwarded to the Gazebo differential drive plugin.
    """

    def __init__(self):
        super().__init__('driver_node')

        # Maximum allowable speed (m/s) – configurable via ROS parameter.
        self.declare_parameter('max_speed', 2.0)
        # Track width (m) used for tank drive conversion.
        self.declare_parameter('track_width', 0.2)

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._tank_sub = self.create_subscription(
            Twist,
            '/tank_cmd',
            self._tank_callback,
            10,
        )

        self.get_logger().info('Driver node started (tank drive).')

    def _tank_callback(self, msg: Twist) -> None:
        """Convert tank drive left/right velocities to Twist and publish."""
        max_speed = self.get_parameter('max_speed').value
        track_width = self.get_parameter('track_width').value

        # Expect left velocity in msg.linear.x, right velocity in msg.linear.y
        left_vel = float(max(-max_speed, min(max_speed, msg.linear.x)))
        right_vel = float(max(-max_speed, min(max_speed, msg.linear.y)))

        self.get_logger().debug(f"Received tank_cmd: left={left_vel}, right={right_vel}")

        # Tank drive: linear.x = (left + right) / 2, angular.z = (right - left) / track_width
        twist = Twist()
        twist.linear.x = (left_vel + right_vel) / 2.0
        twist.angular.z = (right_vel - left_vel) / track_width

        self.get_logger().debug(f"Publishing cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

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

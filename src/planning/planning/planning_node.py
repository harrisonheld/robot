"""Planning node for the RC car.

Implements a simple reactive obstacle-avoidance planner.  The node listens for
obstacle detections from the perception layer and publishes Ackermann drive
commands to the driver layer.

When no obstacle is detected the car drives straight at a constant cruise
speed.  When an obstacle is detected within the safety threshold the car steers
away from it proportionally to the obstacle's proximity and bearing.
"""

# Copyright (c) 2024 Maintainer
#
# SPDX-License-Identifier: MIT

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist


class PlanningNode(Node):
    """Reactive obstacle-avoidance planner.

    Subscriptions
    -------------
    /obstacles : geometry_msgs/PointStamped
        Closest obstacle: point.x = distance (m), point.y = bearing (rad).

    Publications
    ------------
    /tank_cmd : geometry_msgs/Twist
        Desired left and right wheel velocities sent to the driver layer.
    """

    def __init__(self):
        super().__init__('planning_node')

        # Cruise speed (m/s) when no obstacle is present.
        self.declare_parameter('cruise_speed', 1.0)
        # Distance at which obstacle avoidance kicks in (m).
        self.declare_parameter('safety_distance', 3.0)
        # Maximum steering angle used during avoidance (rad).
        self.declare_parameter('max_avoidance_steering', 0.4)
        # Timer period (s) for the control loop.
        self.declare_parameter('control_period', 0.05)

        self._cmd_pub = self.create_publisher(
            Twist, '/tank_cmd', 10
        )

        self._obstacle_sub = self.create_subscription(
            PointStamped, '/obstacles', self._obstacle_callback, 10
        )

        # Latest obstacle info: (distance, bearing).  None = no obstacle.
        self._latest_obstacle = None

        control_period = self.get_parameter('control_period').value
        self._timer = self.create_timer(control_period, self._control_loop)

        self.get_logger().info('Planning node started.')

    def _obstacle_callback(self, msg: PointStamped) -> None:
        """Store the latest obstacle detection."""
        distance = msg.point.x
        bearing = msg.point.y
        self._latest_obstacle = (distance, bearing)

    def _control_loop(self) -> None:
        """Publish a tank drive command on every timer tick."""
        cruise_speed = self.get_parameter('cruise_speed').value
        safety_distance = self.get_parameter('safety_distance').value
        max_steering = self.get_parameter('max_avoidance_steering').value

        left_vel = cruise_speed
        right_vel = cruise_speed

        if self._latest_obstacle is not None:
            distance, bearing = self._latest_obstacle
            if distance < safety_distance and distance > 0.0:
                proximity = 1.0 - (distance / safety_distance)
                # If obstacle ahead, slow down and turn
                if abs(bearing) < math.radians(20):
                    left_vel = 0.0
                    right_vel = 0.0
                elif bearing > 0:
                    # Obstacle left: turn right
                    left_vel = cruise_speed * (1.0 - proximity)
                    right_vel = cruise_speed
                else:
                    # Obstacle right: turn left
                    left_vel = cruise_speed
                    right_vel = cruise_speed * (1.0 - proximity)

            self._latest_obstacle = None

        cmd = Twist()
        cmd.linear.x = float(left_vel)
        cmd.linear.y = float(right_vel)
        self._cmd_pub.publish(cmd)


def main(args=None):
    """Entry point for the planning node."""
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

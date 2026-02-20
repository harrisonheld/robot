"""Unit tests for the planning node."""

# Copyright (c) 2024 Maintainer
#
# SPDX-License-Identifier: MIT

import math
import pytest


def _compute_command(obstacle, cruise_speed=1.0, safety_distance=0.8,
                     max_avoidance_steering=0.4):
    """Replicate the control-loop logic from PlanningNode._control_loop."""
    speed = cruise_speed
    steering_angle = 0.0

    if obstacle is not None:
        distance, bearing = obstacle
        if distance < safety_distance and distance > 0.0:
            proximity = 1.0 - (distance / safety_distance)
            steering_angle = -math.copysign(
                max_avoidance_steering * proximity, bearing
            )
            speed = cruise_speed * (distance / safety_distance)

    return speed, steering_angle


class TestPlanningControlLoop:
    """Tests for the reactive planner control logic."""

    def test_no_obstacle_cruise_straight(self):
        """Without obstacles the car should drive straight at cruise speed."""
        speed, steering = _compute_command(None)
        assert speed == pytest.approx(1.0)
        assert steering == pytest.approx(0.0)

    def test_obstacle_ahead_slows_down(self):
        """Obstacle straight ahead should reduce speed."""
        speed, _ = _compute_command((0.4, 0.0))
        assert speed < 1.0

    def test_obstacle_on_right_steers_left(self):
        """Obstacle on the right (negative bearing) should produce positive
        (left) steering."""
        _, steering = _compute_command((0.4, -0.5))
        assert steering > 0.0

    def test_obstacle_on_left_steers_right(self):
        """Obstacle on the left (positive bearing) should produce negative
        (right) steering."""
        _, steering = _compute_command((0.4, 0.5))
        assert steering < 0.0

    def test_obstacle_beyond_safety_distance_ignored(self):
        """Obstacle beyond safety_distance should not affect the command."""
        speed, steering = _compute_command((1.5, 0.3),
                                           safety_distance=0.8)
        assert speed == pytest.approx(1.0)
        assert steering == pytest.approx(0.0)

    def test_steering_proportional_to_proximity(self):
        """Closer obstacle should cause larger steering correction."""
        _, steering_close = _compute_command((0.2, 0.5))
        _, steering_far = _compute_command((0.7, 0.5))
        assert abs(steering_close) > abs(steering_far)

    def test_speed_proportional_to_distance(self):
        """Speed should be proportional to obstacle distance."""
        speed_close, _ = _compute_command((0.2, 0.0))
        speed_far, _ = _compute_command((0.7, 0.0))
        assert speed_close < speed_far

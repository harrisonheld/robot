"""Unit tests for the driver node."""

# Copyright (c) 2024 Maintainer
#
# SPDX-License-Identifier: MIT

import math
import pytest


def _tank_to_twist(left_vel, right_vel, track_width=0.2, max_speed=2.0):
    """Replicate the conversion logic from DriverNode._tank_callback."""
    left_vel = max(-max_speed, min(max_speed, left_vel))
    right_vel = max(-max_speed, min(max_speed, right_vel))
    linear_x = (left_vel + right_vel) / 2.0
    angular_z = (right_vel - left_vel) / track_width
    return linear_x, angular_z


class TestTankToTwistConversion:
    """Tests for the tank drive → Twist conversion logic."""

    def test_straight_ahead(self):
        """Equal left/right velocities should produce straight motion."""
        linear_x, angular_z = _tank_to_twist(1.0, 1.0)
        assert linear_x == pytest.approx(1.0)
        assert angular_z == pytest.approx(0.0)

    def test_turn_left(self):
        """Left turn: right wheel faster than left."""
        linear_x, angular_z = _tank_to_twist(1.0, 2.0)
        assert linear_x == pytest.approx(1.5)
        assert angular_z > 0.0

    def test_turn_right(self):
        """Right turn: left wheel faster than right."""
        linear_x, angular_z = _tank_to_twist(2.0, 1.0)
        assert linear_x == pytest.approx(1.5)
        assert angular_z < 0.0

    def test_speed_clamped_to_max(self):
        """Wheel velocities above max_speed should be clamped."""
        linear_x, _ = _tank_to_twist(10.0, 10.0, max_speed=2.0)
        assert linear_x == pytest.approx(2.0)

    def test_speed_clamped_negative(self):
        """Wheel velocities below -max_speed should be clamped."""
        linear_x, _ = _tank_to_twist(-10.0, -10.0, max_speed=2.0)
        assert linear_x == pytest.approx(-2.0)

    def test_spin_in_place(self):
        """Spin in place: left forward, right backward."""
        linear_x, angular_z = _tank_to_twist(1.0, -1.0)
        assert linear_x == pytest.approx(0.0)
        assert angular_z < 0.0

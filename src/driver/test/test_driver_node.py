"""Unit tests for the driver node."""

# Copyright (c) 2024 Maintainer
#
# SPDX-License-Identifier: MIT

import math
import pytest


def _ackermann_to_twist(speed, steering_angle, wheelbase=0.3,
                         max_speed=2.0, max_steering=0.5):
    """Replicate the conversion logic from DriverNode._ackermann_callback."""
    speed = max(-max_speed, min(max_speed, speed))
    steering_angle = max(-max_steering, min(max_steering, steering_angle))

    if abs(steering_angle) > 1e-6 and abs(wheelbase) > 1e-6:
        turning_radius = wheelbase / math.tan(abs(steering_angle))
        angular_z = speed / turning_radius
        if steering_angle < 0:
            angular_z = -angular_z
    else:
        angular_z = 0.0

    return speed, angular_z


class TestAckermannToTwistConversion:
    """Tests for the Ackermann → Twist conversion logic."""

    def test_straight_ahead(self):
        """Zero steering should produce zero angular velocity."""
        speed, angular_z = _ackermann_to_twist(1.0, 0.0)
        assert speed == pytest.approx(1.0)
        assert angular_z == pytest.approx(0.0)

    def test_positive_steering_positive_angular_z(self):
        """Positive (left) steering should produce positive angular velocity."""
        _, angular_z = _ackermann_to_twist(1.0, 0.3)
        assert angular_z > 0.0

    def test_negative_steering_negative_angular_z(self):
        """Negative (right) steering should produce negative angular velocity."""
        _, angular_z = _ackermann_to_twist(1.0, -0.3)
        assert angular_z < 0.0

    def test_speed_clamped_to_max(self):
        """Speed above max_speed should be clamped."""
        speed, _ = _ackermann_to_twist(10.0, 0.0, max_speed=2.0)
        assert speed == pytest.approx(2.0)

    def test_speed_clamped_negative(self):
        """Speed below -max_speed should be clamped."""
        speed, _ = _ackermann_to_twist(-10.0, 0.0, max_speed=2.0)
        assert speed == pytest.approx(-2.0)

    def test_steering_clamped_to_max(self):
        """Steering angle above max_steering should be clamped."""
        _, angular_z_clamped = _ackermann_to_twist(1.0, 5.0, max_steering=0.5)
        _, angular_z_unclamped = _ackermann_to_twist(1.0, 0.5, max_steering=0.5)
        assert abs(angular_z_clamped) == pytest.approx(abs(angular_z_unclamped),
                                                        rel=1e-3)

    def test_reverse_with_steering(self):
        """Negative speed with left steering should give negative angular_z."""
        _, angular_z = _ackermann_to_twist(-1.0, 0.3)
        assert angular_z < 0.0

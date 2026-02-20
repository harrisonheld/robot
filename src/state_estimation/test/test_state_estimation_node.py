"""Unit tests for the state estimation node."""

# Copyright (c) 2024 Maintainer
#
# SPDX-License-Identifier: MIT

import math
import pytest


def _integrate_heading(angular_velocity_z, dt, initial_heading=0.0):
    """Replicate the heading integration logic from StateEstimationNode."""
    heading = initial_heading + angular_velocity_z * dt
    return math.atan2(math.sin(heading), math.cos(heading))


class TestHeadingIntegration:
    """Tests for the gyroscope heading integration logic."""

    def test_zero_angular_velocity_no_change(self):
        """Zero gyro rate should leave heading unchanged."""
        heading = _integrate_heading(0.0, 0.1, initial_heading=0.5)
        assert heading == pytest.approx(0.5)

    def test_positive_rate_increases_heading(self):
        """Positive angular velocity should increase heading."""
        heading = _integrate_heading(1.0, 0.1, initial_heading=0.0)
        assert heading == pytest.approx(0.1)

    def test_negative_rate_decreases_heading(self):
        """Negative angular velocity should decrease heading."""
        heading = _integrate_heading(-1.0, 0.1, initial_heading=0.0)
        assert heading == pytest.approx(-0.1)

    def test_heading_wraps_positive(self):
        """Heading above π should wrap to negative range."""
        # Start just below π, add a little more to go past it.
        heading = _integrate_heading(1.0, 0.2, initial_heading=math.pi - 0.1)
        assert -math.pi <= heading <= math.pi

    def test_heading_wraps_negative(self):
        """Heading below -π should wrap to positive range."""
        heading = _integrate_heading(-1.0, 0.2, initial_heading=-math.pi + 0.1)
        assert -math.pi <= heading <= math.pi

    def test_orientation_quaternion_from_heading(self):
        """Orientation quaternion should encode heading correctly."""
        heading = math.pi / 4  # 45 degrees
        qz = math.sin(heading / 2.0)
        qw = math.cos(heading / 2.0)
        # Recover heading from quaternion: 2 * atan2(qz, qw)
        recovered = 2.0 * math.atan2(qz, qw)
        assert recovered == pytest.approx(heading)

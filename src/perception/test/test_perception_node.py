"""Unit tests for the perception node."""

# Copyright (c) 2024 Maintainer
#
# SPDX-License-Identifier: MIT

import math
import pytest


def _find_closest_obstacle(ranges, angle_min, angle_increment,
                            range_min, range_max, threshold):
    """Replicate obstacle detection logic from PerceptionNode."""
    min_range = float('inf')
    min_index = -1

    for i, r in enumerate(ranges):
        if range_min <= r <= range_max and r < min_range:
            min_range = r
            min_index = i

    if min_index == -1 or min_range > threshold:
        return None

    bearing = angle_min + min_index * angle_increment
    return min_range, bearing


class TestObstacleDetection:
    """Tests for the obstacle detection logic."""

    def _make_empty_scan(self, n=360):
        """Return a scan where all ranges are at max range (10 m)."""
        return [10.0] * n

    def test_no_obstacle_returns_none(self):
        """Scan with all ranges at max should return None."""
        ranges = self._make_empty_scan()
        result = _find_closest_obstacle(
            ranges, -math.pi, 2 * math.pi / 360,
            0.12, 10.0, threshold=1.0
        )
        assert result is None

    def test_obstacle_within_threshold(self):
        """A range reading below threshold should be detected."""
        ranges = self._make_empty_scan()
        ranges[0] = 0.5
        result = _find_closest_obstacle(
            ranges, -math.pi, 2 * math.pi / 360,
            0.12, 10.0, threshold=1.0
        )
        assert result is not None
        distance, _ = result
        assert distance == pytest.approx(0.5)

    def test_obstacle_bearing_correct(self):
        """Bearing for index 0 should equal angle_min."""
        ranges = self._make_empty_scan()
        ranges[0] = 0.5
        angle_min = -math.pi
        angle_increment = 2 * math.pi / 360
        result = _find_closest_obstacle(
            ranges, angle_min, angle_increment,
            0.12, 10.0, threshold=1.0
        )
        assert result is not None
        _, bearing = result
        assert bearing == pytest.approx(angle_min)

    def test_closest_obstacle_selected(self):
        """When multiple obstacles present, the closest is returned."""
        ranges = self._make_empty_scan()
        ranges[10] = 0.8
        ranges[50] = 0.4
        result = _find_closest_obstacle(
            ranges, -math.pi, 2 * math.pi / 360,
            0.12, 10.0, threshold=1.0
        )
        assert result is not None
        distance, _ = result
        assert distance == pytest.approx(0.4)

    def test_range_below_min_ignored(self):
        """Ranges below range_min should be ignored."""
        ranges = self._make_empty_scan()
        ranges[0] = 0.05  # Below range_min of 0.12
        result = _find_closest_obstacle(
            ranges, -math.pi, 2 * math.pi / 360,
            0.12, 10.0, threshold=1.0
        )
        assert result is None

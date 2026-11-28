"""Position.distance_to and the shared distance helper."""

from __future__ import annotations

import pytest

from flypi._geo import distance_m
from flypi.telemetry import Position


def test_distance_to_is_none_for_an_empty_position() -> None:
    assert Position().distance_to(0, 0) is None
    assert Position(lat=1.0).distance_to(0, 0) is None
    assert Position(lon=1.0).distance_to(0, 0) is None


def test_distance_to_known_distance() -> None:
    # One degree of longitude at the equator is about 111,319.5 m.
    assert Position(lat=0.0, lon=0.0).distance_to(0, 1) == pytest.approx(111_319.5, rel=1e-4)


def test_distance_to_matches_the_shared_helper() -> None:
    pos = Position(lat=-35.3632, lon=149.1652)
    assert pos.distance_to(-35.3629, 149.1655) == distance_m(-35.3632, 149.1652, -35.3629, 149.1655)


def test_distance_to_same_point_is_zero() -> None:
    assert Position(lat=10.0, lon=20.0).distance_to(10.0, 20.0) == 0.0

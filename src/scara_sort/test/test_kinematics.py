"""Unit tests for kinematics.py (no ROS or Gazebo needed). Copied from the driller."""

from math import radians

import pytest

from scara_sort.kinematics import fk, ik


def test_fk_straight_arm():
    # All joints at 0: arm straight along x, quill fully up
    assert fk(0, 0, 0) == pytest.approx((0.55, 0, 0.13))


def test_fk_elbow_bent_90():
    # Link 1 along x, link 2 turned 90° to the left, cup 50 mm down
    assert fk(0, radians(90), 0.05) == pytest.approx((0.30, 0.25, 0.08))


@pytest.mark.parametrize('point', [
    (0.40, 0.0, 0.10),     # ready pose above the pick line
    (0.32, -0.15, 0.042),  # corners of the pick zone, 2 mm over an item top
    (0.48, -0.15, 0.042),
    (0.32, 0.15, 0.042),
    (0.48, 0.15, 0.042),
    (0.55, 0.0, 0.13),     # full reach, quill up
    (0.0, 0.40, -0.02),    # to the left, cup at its lowest
])
def test_ik_then_fk_gives_the_same_point(point):
    assert fk(*ik(*point)) == pytest.approx(point)


def test_ik_always_uses_the_same_elbow():
    for x in (0.2, 0.3, 0.4, 0.5):
        for y in (-0.1, 0.0, 0.1):
            j1, j2, j3 = ik(x, y, 0.05)
            assert j2 > 0


@pytest.mark.parametrize('point, reason', [
    ((0.60, 0.0, 0.05), 'from the base'),        # too far
    ((0.10, 0.0, 0.05), 'from the base'),        # too close to the column
    ((-0.31, -0.26, 0.05), 'shoulder'),          # behind on the right: j1 past -150°
    ((0.35, 0.0, 0.20), 'height'),               # too high
    ((0.35, 0.0, -0.05), 'height'),              # too low
])
def test_unreachable_points_are_refused(point, reason):
    with pytest.raises(ValueError, match=reason):
        ik(*point)

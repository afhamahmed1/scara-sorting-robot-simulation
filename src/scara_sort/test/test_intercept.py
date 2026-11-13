"""Unit tests for intercept.py (no ROS or Gazebo needed)."""

from math import pi

import pytest

from scara_sort.intercept import MARGIN, intercept, move_time
from scara_sort.kinematics import ik

READY = ik(0.40, 0.0, 0.10)   # the ready pose above the pick line
HOVER = 0.06                   # meet items with the cup 20 mm over their tops
FOLLOW = 0.8                   # s of following: lower, grip, lift


def test_move_time_is_the_slowest_joint():
    assert move_time((0, 0, 0), (1.0, 0.5, 0.0)) == pytest.approx(0.5)     # j1: 1 rad at 2 rad/s
    assert move_time((0, 0, 0), (0.1, 0.1, 0.15)) == pytest.approx(0.75)   # j3: 0.15 m at 0.2 m/s
    assert move_time((0, pi / 2, 0), (0, -pi / 2, 0)) == pytest.approx(pi / 2)


def test_item_on_a_stopped_belt_in_the_zone_is_met_where_it_lies():
    t, y = intercept(READY, 0.41, -0.05, 0.0, HOVER, FOLLOW)
    assert y == pytest.approx(-0.05)
    assert t == pytest.approx(move_time(READY, ik(0.41, -0.05, HOVER)) + MARGIN, abs=0.011)


def test_arm_gets_there_in_time():
    joints = ik(0.12, 0.40, 0.13)   # over bin 1, cup fully up
    t, y = intercept(joints, 0.40, -0.30, 0.05, HOVER, FOLLOW)
    assert move_time(joints, ik(0.40, y, HOVER)) + MARGIN <= t + 1e-9
    assert y == pytest.approx(-0.30 + 0.05 * t)


def test_earliest_meeting_is_at_the_start_of_the_zone_for_an_item_far_upstream():
    t, y = intercept(READY, 0.40, -0.50, 0.05, HOVER, FOLLOW)
    assert y == pytest.approx(-0.15, abs=0.001)   # waits for it at the start of the pick zone
    assert t == pytest.approx(7.0, abs=0.02)


def test_no_meeting_when_the_item_is_about_to_leave_the_zone():
    assert intercept(READY, 0.40, 0.12, 0.05, HOVER, FOLLOW) is None


def test_far_away_arm_cannot_catch_a_fast_item_near_the_end():
    joints = ik(-0.38, 0.06, 0.13)   # over the reject bin
    assert intercept(joints, 0.40, 0.0, 0.10, HOVER, FOLLOW) is None
    assert intercept(joints, 0.40, -0.30, 0.10, HOVER, FOLLOW) is not None   # but an early one


def test_follow_time_is_kept_inside_the_zone():
    t, y = intercept(READY, 0.40, 0.0, 0.10, HOVER, FOLLOW)
    assert y + 0.10 * FOLLOW <= 0.15 + 1e-9

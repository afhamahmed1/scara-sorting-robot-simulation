"""Unit tests for tracker.py (no ROS or Gazebo needed)."""

import pytest

from scara_sort.tracker import Tracker


def see(tracker, belt, *items):
    """One camera frame at encoder reading belt; items: (colour, shape, x, y)."""
    return tracker.update(list(items), belt)


def test_item_is_confirmed_after_3_frames():
    t = Tracker()
    see(t, 0.000, ('red', 'cube', 0.40, -0.60))
    see(t, 0.005, ('red', 'cube', 0.40, -0.595))
    assert t.items(0.005) == []
    see(t, 0.010, ('red', 'cube', 0.40, -0.59))
    [item] = t.items(0.010)
    assert (item.id, item.colour, item.shape, item.times_seen) == (1, 'red', 'cube', 3)


def test_position_is_predicted_from_the_encoder_after_the_item_leaves_the_view():
    t = Tracker()
    for k in range(5):
        see(t, 0.01 * k, ('blue', 'prism', 0.41, -0.50 + 0.01 * k))
    [item] = t.items(0.04)
    assert item.y(0.04) == pytest.approx(-0.46)
    # The belt moves on another 0.46 m: the item is now at the pick line
    assert item.y(0.50) == pytest.approx(0.0)
    assert item.x == pytest.approx(0.41)


def test_prediction_holds_while_the_belt_stops():
    t = Tracker()
    for k in range(3):
        see(t, 1.0 + 0.01 * k, ('green', 'cylinder', 0.40, -0.40 + 0.01 * k))
    [item] = t.items(1.02)
    assert item.y(1.02) == pytest.approx(-0.38)
    assert item.y(1.02) == item.y(1.02)   # belt stopped: no change


def test_noise_is_averaged_out():
    t = Tracker()
    noise = [0.002, -0.002, 0.001, -0.001, 0.0005, -0.0005]
    for k, n in enumerate(noise):
        see(t, 0.01 * k, ('red', 'cube', 0.40 + n, -0.50 + 0.01 * k + n))
    [item] = t.items(0.05)
    assert item.y(0.05) == pytest.approx(-0.45, abs=1e-9)
    assert item.x == pytest.approx(0.40, abs=1e-9)


def test_two_items_keep_their_own_ids():
    t = Tracker()
    for k in range(3):
        ids = see(t, 0.01 * k, ('red', 'cube', 0.40, -0.60 + 0.01 * k),
                  ('yellow', 'prism', 0.42, -0.35 + 0.01 * k))
        assert ids == [1, 2]
    assert [i.id for i in t.items(0.02)] == [1, 2]


def test_far_detection_is_a_new_item():
    t = Tracker()
    see(t, 0.0, ('red', 'cube', 0.40, -0.60))
    ids = see(t, 0.0, ('red', 'cube', 0.40, -0.58))   # 20 mm away: more than 15 mm
    assert ids == [2]


def test_other_colour_is_a_new_item():
    t = Tracker()
    see(t, 0.0, ('red', 'cube', 0.40, -0.60))
    assert see(t, 0.0, ('green', 'cube', 0.40, -0.60)) == [2]


def test_shape_is_the_most_common_answer():
    t = Tracker()
    for shape in ('cube', 'unknown', 'cube', 'cube', 'unknown'):
        see(t, 0.0, ('red', shape, 0.40, -0.50))
    [item] = t.items(0.0)
    assert item.shape == 'cube'


def test_item_that_was_seen_once_is_dropped_when_not_seen_again():
    t = Tracker()
    see(t, 0.0, ('red', 'cube', 0.40, -0.50))   # a one-off glitch
    see(t, 0.05)                                 # 50 mm later, nothing there
    assert t.tracks == []


def test_items_past_the_end_of_the_belt_are_forgotten():
    t = Tracker()
    for k in range(3):
        see(t, 0.01 * k, ('red', 'cube', 0.40, -0.50 + 0.01 * k))
    assert len(t.items(1.0)) == 1    # predicted at y 0.48: still on the belt
    assert t.items(1.4) == []        # predicted at y 0.88: gone

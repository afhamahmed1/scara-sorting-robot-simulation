"""Unit tests for grip.py: which item the suction cup would grip."""

import pytest

from scara_sort.grip import item_under_cup

CUP = (0.40, 0.0, 0.242)   # cup tip 2 mm over the top of an item lying on the belt (z 0.20)


def on_belt(x, y):
    return (x, y, 0.22)    # an item's centre: 20 mm over the belt


def test_item_right_under_the_cup():
    assert item_under_cup(CUP, {'item_03': on_belt(0.40, 0.0)}) == 'item_03'


@pytest.mark.parametrize('dx', [0.014, -0.014])
def test_up_to_15_mm_sideways_is_still_under_the_cup(dx):
    assert item_under_cup(CUP, {'item_03': on_belt(0.40 + dx, 0.0)}) == 'item_03'


def test_more_than_15_mm_sideways_is_not():
    assert item_under_cup(CUP, {'item_03': on_belt(0.40, 0.016)}) is None


def test_cup_too_high_grips_nothing():
    assert item_under_cup((0.40, 0.0, 0.250), {'item_03': on_belt(0.40, 0.0)}) is None   # 10 mm gap


def test_nothing_there():
    assert item_under_cup(CUP, {}) is None


def test_items_elsewhere_are_ignored():
    items = {'item_01': (0.12, 0.40, 0.17), 'item_02': on_belt(0.40, -0.25)}
    assert item_under_cup(CUP, items) is None


def test_the_nearest_of_two_wins():
    items = {'item_01': on_belt(0.40, 0.010), 'item_02': on_belt(0.40, -0.004)}
    assert item_under_cup(CUP, items) == 'item_02'

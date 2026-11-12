"""Unit tests for items.py: shapes, Gazebo models and the seeded sequence."""

import xml.etree.ElementTree as ET
from itertools import islice
from math import cos, dist, hypot, sin, sqrt

import pytest

from scara_sort.items import (COLOURS, MASS, PRISM_CORNERS, SHAPES, inertia, item_sdf,
                              item_sequence, prism_collision_boxes)

SETTINGS = {'colours': list(COLOURS), 'shapes': SHAPES, 'spacing': 250, 'lateral': 30,
            'count': 30, 'seed': 1}


def test_prism_is_an_equilateral_triangle_with_40_mm_sides_around_its_centre():
    a, b, c = PRISM_CORNERS
    for p, q in ((a, b), (b, c), (c, a)):
        assert dist(p, q) == pytest.approx(0.04)
    centre = (sum(x for x, _ in PRISM_CORNERS) / 3, sum(y for _, y in PRISM_CORNERS) / 3)
    assert centre == pytest.approx((0, 0), abs=1e-12)


@pytest.mark.parametrize('shape', SHAPES)
def test_item_sdf_is_valid_xml_with_the_right_name_link_and_mass(shape):
    sdf = item_sdf('item_07', 'red', shape)
    model = ET.fromstring(sdf).find('model')
    assert model.get('name') == 'item_07'
    assert model.find('link').get('name') == 'link'   # the suction cup welds this link
    assert float(model.find('link/inertial/mass').text) == MASS


def test_item_sdf_uses_the_colour():
    sdf = item_sdf('item_00', 'blue', 'cube')
    r, g, b = COLOURS['blue']
    assert f'<diffuse>{r} {g} {b} 1</diffuse>' in sdf


def test_inertia_of_a_cube():
    i = MASS * 2 * 0.04**2 / 12
    assert inertia('cube') == pytest.approx((i, i, i))


def test_unknown_shape_is_refused():
    with pytest.raises(ValueError):
        item_sdf('item_00', 'red', 'sphere')


def test_same_seed_gives_the_same_items():
    assert list(item_sequence(SETTINGS)) == list(item_sequence(SETTINGS))


def test_another_seed_gives_other_items():
    other = dict(SETTINGS, seed=2)
    assert list(item_sequence(SETTINGS)) != list(item_sequence(other))


def test_sequence_has_count_items_numbered_from_1():
    items = list(item_sequence(SETTINGS))
    assert [item['number'] for item in items] == list(range(1, 31))


def test_sequence_mixes_every_colour_and_shape_within_limits():
    items = list(item_sequence(SETTINGS))
    assert {item['colour'] for item in items} == set(COLOURS)
    assert {item['shape'] for item in items} == set(SHAPES)
    for item in items:
        assert -0.030 <= item['lateral'] <= 0.030
        assert 0.200 <= item['gap'] <= 0.300


def test_count_0_keeps_going():
    endless = item_sequence(dict(SETTINGS, count=0))
    assert len(list(islice(endless, 100))) == 100


def test_prism_corner_distance_from_centre():
    assert dist((0, 0), PRISM_CORNERS[0]) == pytest.approx(0.04 / sqrt(3))


def corners_of(x, y, yaw, length, width):
    c, s = cos(yaw), sin(yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy)
            for dx in (-length / 2, length / 2) for dy in (-width / 2, width / 2)]


def inside_triangle(px, py):
    """Inside (or on) the prism's triangle: on the inner side of all three sides."""
    for (x1, y1), (x2, y2) in zip(PRISM_CORNERS, PRISM_CORNERS[1:] + PRISM_CORNERS[:1]):
        if (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1) < -1e-9:  # corners go anticlockwise
            return False
    return True


def test_prism_collision_boxes_stay_inside_the_triangle_and_touch_each_side():
    boxes = prism_collision_boxes()
    assert len(boxes) == 3
    for box in boxes:
        assert all(inside_triangle(px, py) for px, py in corners_of(*box))
    # The outer face of each box lies on a side: 2 of its corners are exactly on the outline
    inradius = 0.04 / (2 * sqrt(3))
    for x, y, yaw, length, width in boxes:
        assert hypot(x, y) + width / 2 == pytest.approx(inradius)


def test_prism_sdf_has_three_box_collisions():
    model = ET.fromstring(item_sdf('item_00', 'red', 'prism')).find('model')
    collisions = model.findall('link/collision')
    assert len(collisions) == 3
    assert all(c.find('geometry/box') is not None for c in collisions)
    assert model.find('link/visual/geometry/polyline') is not None  # but it looks exact

"""Unit tests for vision.py on synthetic images drawn with OpenCV (no ROS or Gazebo needed).

The images imitate the Gazebo camera: a grey belt between light rails, items drawn as their
brightly lit top faces, some with a dark side face next to them.
"""

import os
from math import cos, pi, sin

import cv2
import numpy as np
import pytest
import yaml

from scara_sort.vision import Camera, classify, detect

CAMERA = Camera(fx=879.19, fy=879.19, cx=320.0, cy=240.0)   # as Gazebo's camera_info says

with open(os.path.join(os.path.dirname(__file__), '..', 'config', 'colours.yaml')) as f:
    RANGES = yaml.safe_load(f)

# Top-face colours as Gazebo renders them (HSV, OpenCV units), and a dark side face
TOP_HSV = {'red': (0, 186, 206), 'green': (66, 176, 181), 'blue': (110, 188, 212),
           'yellow': (27, 189, 217)}


def bgr(hsv):
    pixel = np.uint8([[hsv]])
    return tuple(int(c) for c in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0])


def belt_image():
    image = np.full((480, 640, 3), (181, 182, 178), np.uint8)       # floor
    u0, v_top = CAMERA.to_pixel(0.32, -0.45)
    u0, v_bottom = CAMERA.to_pixel(0.48, -0.45)
    image[int(v_top) - 12:int(v_top)] = (196, 192, 192)               # rails
    image[int(v_bottom):int(v_bottom) + 12] = (196, 192, 192)
    image[int(v_top):int(v_bottom)] = (86, 86, 86)                    # belt
    return image


def outline(shape, x, y, yaw):
    """Pixel outline of an item's top face centred at (x, y) m, turned by yaw."""
    if shape == 'cube':
        corners = [(0.02 * cos(yaw + a), 0.02 * sin(yaw + a)) for a in (0, pi / 2, pi, 3 * pi / 2)]
        corners = [(0.02 * 2 ** 0.5 * cos(yaw + pi / 4 + k * pi / 2),
                    0.02 * 2 ** 0.5 * sin(yaw + pi / 4 + k * pi / 2)) for k in range(4)]
    elif shape == 'prism':
        r = 0.04 / 3 ** 0.5
        corners = [(r * cos(yaw + pi / 2 + k * 2 * pi / 3), r * sin(yaw + pi / 2 + k * 2 * pi / 3))
                   for k in range(3)]
    else:
        corners = [(0.02 * cos(a), 0.02 * sin(a)) for a in np.linspace(0, 2 * pi, 72, endpoint=False)]
    return np.array([CAMERA.to_pixel(x + dx, y + dy) for dx, dy in corners])


def draw_item(image, colour, shape, x, y, yaw=0.3, side=0):
    """Draw an item's top face; side > 0 adds a dark side face that many pixels towards +u."""
    points = outline(shape, x, y, yaw)
    if side:
        h, s, v = TOP_HSV[colour]
        shifted = points + np.array([side, 0])
        cv2.fillPoly(image, [np.round(shifted * 16).astype(np.int32)], bgr((h, s, 100)),
                     cv2.LINE_AA, shift=4)
    cv2.fillPoly(image, [np.round(points * 16).astype(np.int32)], bgr(TOP_HSV[colour]),
                 cv2.LINE_AA, shift=4)


def test_pixel_to_belt_and_back():
    assert CAMERA.to_belt(320, 240) == pytest.approx((0.40, -0.45))
    u, v = CAMERA.to_pixel(0.43, -0.30)
    assert CAMERA.to_belt(u, v) == pytest.approx((0.43, -0.30))


def test_image_right_is_along_the_belt_and_down_is_away_from_the_robot():
    x, y = CAMERA.to_belt(420, 240)
    assert y > -0.45 and x == pytest.approx(0.40)
    x, y = CAMERA.to_belt(320, 300)
    assert x > 0.40 and y == pytest.approx(-0.45)


def test_scale_is_about_0_7_mm_per_pixel():
    x1, y1 = CAMERA.to_belt(100, 240)
    x2, y2 = CAMERA.to_belt(101, 240)
    assert (y2 - y1) * 1000 == pytest.approx(0.694, abs=0.002)


@pytest.mark.parametrize('colour', sorted(TOP_HSV))
@pytest.mark.parametrize('shape', ['cube', 'cylinder', 'prism'])
def test_every_colour_and_shape_is_found_where_it_is(colour, shape):
    image = belt_image()
    draw_item(image, colour, shape, 0.41, -0.40, yaw=0.5)
    [found] = detect(image, RANGES, CAMERA)
    assert (found.colour, found.shape) == (colour, shape)
    assert found.x == pytest.approx(0.41, abs=0.001)
    assert found.y == pytest.approx(-0.40, abs=0.001)


@pytest.mark.parametrize('yaw', [0.0, 0.4, 0.8, 1.2, 2.0, 3.0])
@pytest.mark.parametrize('shape', ['cube', 'prism'])
def test_shapes_are_recognised_at_any_angle(shape, yaw):
    image = belt_image()
    draw_item(image, 'blue', shape, 0.38, -0.50, yaw=yaw)
    [found] = detect(image, RANGES, CAMERA)
    assert found.shape == shape


def test_dark_side_faces_are_left_out():
    image = belt_image()
    draw_item(image, 'green', 'cube', 0.40, -0.30, side=12)   # far from the centre: side visible
    [found] = detect(image, RANGES, CAMERA)
    assert found.shape == 'cube'
    assert found.y == pytest.approx(-0.30, abs=0.001)          # the top face's centre


def test_several_items_at_once():
    image = belt_image()
    draw_item(image, 'red', 'cube', 0.37, -0.60)
    draw_item(image, 'yellow', 'prism', 0.43, -0.45)
    draw_item(image, 'blue', 'cylinder', 0.40, -0.30)
    found = sorted(detect(image, RANGES, CAMERA), key=lambda d: d.y)
    assert [(d.colour, d.shape) for d in found] == [
        ('red', 'cube'), ('yellow', 'prism'), ('blue', 'cylinder')]


def test_item_cut_by_the_image_edge_is_left_until_fully_in_view():
    image = belt_image()
    u_left, _ = CAMERA.to_pixel(0.40, -0.67)
    y_edge = -0.45 + (0 - 320) * 0.61 / 879.19   # the left image edge on the belt
    draw_item(image, 'red', 'cube', 0.40, y_edge + 0.01)   # half in view
    assert detect(image, RANGES, CAMERA) == []


def test_coloured_things_off_the_belt_are_ignored():
    image = belt_image()
    draw_item(image, 'red', 'cube', 0.27, -0.45)   # on the floor beside the belt
    assert detect(image, RANGES, CAMERA) == []


def test_two_touching_items_are_unknown():
    image = belt_image()
    draw_item(image, 'red', 'cube', 0.40, -0.45, yaw=0)
    draw_item(image, 'red', 'cube', 0.40, -0.41, yaw=0)    # 40 mm on: sides touch
    [found] = detect(image, RANGES, CAMERA)
    assert found.shape == 'unknown'


def test_grey_belt_rails_and_floor_match_no_colour():
    assert detect(belt_image(), RANGES, CAMERA) == []


def test_red_found_on_both_sides_of_hue_0():
    image = belt_image()
    TOP_HSV['red'] = (178, 186, 206)   # hue just below 180 (= 360°)
    try:
        draw_item(image, 'red', 'cylinder', 0.40, -0.45)
        [found] = detect(image, RANGES, CAMERA)
        assert found.colour == 'red'
    finally:
        TOP_HSV['red'] = (0, 186, 206)


def test_classify_wrong_size_is_unknown():
    square = outline('cube', 0.40, -0.45, 0)
    half = (square - square.mean(axis=0)) * 0.5 + square.mean(axis=0)  # a 20 mm square
    contour = np.round(half).astype(np.int32).reshape(-1, 1, 2)
    assert classify(contour, CAMERA) == 'unknown'

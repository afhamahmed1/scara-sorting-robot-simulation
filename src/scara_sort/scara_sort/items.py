"""The items: their shapes and colours, the Gazebo model of each, and the seeded sequence.

Plain Python, no ROS: tested in test/test_items.py.

Every item is 40 mm tall and weighs 50 g:
    cube       40 mm sides
    cylinder   Ø40 mm, standing up
    prism      triangular prism, 40 mm sides, standing on a triangle
"""

import random
from math import atan2, cos, pi, sin, sqrt

SIZE = 0.04   # m
MASS = 0.05   # kg
FRICTION = 1.0

SHAPES = ['cube', 'cylinder', 'prism']

# Strong matt colours that are easy to tell apart on the dark belt (red, green, blue: 0-1)
COLOURS = {
    'red': (0.80, 0.05, 0.05),
    'green': (0.05, 0.60, 0.10),
    'blue': (0.05, 0.20, 0.85),
    'yellow': (0.90, 0.75, 0.05),
}

# The prism's corners (x, y) around its centre: an equilateral triangle with 40 mm sides
PRISM_RADIUS = SIZE / sqrt(3)  # centre to corner
PRISM_CORNERS = [(PRISM_RADIUS * cos(a), PRISM_RADIUS * sin(a))
                 for a in (pi / 2, pi / 2 + 2 * pi / 3, pi / 2 + 4 * pi / 3)]


def geometry(shape):
    """SDF geometry of a shape's visual, and the z offset that centres it on the link origin."""
    if shape == 'cube':
        return f'<box><size>{SIZE} {SIZE} {SIZE}</size></box>', 0.0
    if shape == 'cylinder':
        return f'<cylinder><radius>{SIZE / 2}</radius><length>{SIZE}</length></cylinder>', 0.0
    if shape == 'prism':
        # A polyline is a polygon pushed up by <height>, from z = 0: move it down by half
        points = ''.join(f'<point>{x:.6f} {y:.6f}</point>' for x, y in PRISM_CORNERS)
        return f'<polyline>{points}<height>{SIZE}</height></polyline>', -SIZE / 2
    raise ValueError(f'unknown shape {shape!r}')


def prism_collision_boxes():
    """The prism's collision shape: three boxes, one along each side, meeting in the middle.

    Gazebo turns a polyline into a triangle mesh, and a pile of meshes in a bin slows the
    simulation down five times. Boxes are cheap. Each box's outer face lies on a side of the
    triangle; the boxes are shortened so that no corner sticks out of the triangle, which
    leaves the three tips (6.7 mm each) without collision. The prism still looks exact.

    Returns [(x, y, yaw, length, width)] in metres and radians.
    """
    inradius = SIZE / (2 * sqrt(3))       # centre to the middle of a side
    width = inradius                      # the three boxes meet at the centre
    length = SIZE - 2 * width / sqrt(3)   # shortened: inner corners stay inside the triangle
    boxes = []
    for (x1, y1), (x2, y2) in zip(PRISM_CORNERS, PRISM_CORNERS[1:] + PRISM_CORNERS[:1]):
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2       # middle of the side, at distance inradius
        scale = (inradius - width / 2) / inradius   # move inwards by half the box width
        boxes.append((mx * scale, my * scale, atan2(y2 - y1, x2 - x1), length, width))
    return boxes


def collisions(shape):
    """SDF collision elements of a shape, with friction."""
    surface = (f'<surface><friction><ode><mu>{FRICTION}</mu><mu2>{FRICTION}</mu2></ode>'
               '</friction></surface>')
    if shape == 'prism':
        return ''.join(
            f"<collision name='collision_{k}'><pose>{x:.6f} {y:.6f} 0 0 0 {yaw:.6f}</pose>"
            f"<geometry><box><size>{length:.6f} {width:.6f} {SIZE}</size></box></geometry>"
            f"{surface}</collision>"
            for k, (x, y, yaw, length, width) in enumerate(prism_collision_boxes()))
    geo, dz = geometry(shape)
    return (f"<collision name='collision'><pose>0 0 {dz} 0 0 0</pose><geometry>{geo}</geometry>"
            f"{surface}</collision>")


def inertia(shape):
    """Moments of inertia (ixx, iyy, izz) of a solid item about its centre, kg·m²."""
    m, a = MASS, SIZE
    if shape == 'cube':
        i = m * (a * a + a * a) / 12
        return i, i, i
    if shape == 'cylinder':
        r = a / 2
        side = m * (3 * r * r + a * a) / 12
        return side, side, m * r * r / 2
    if shape == 'prism':
        side = m * (a * a / 24 + a * a / 12)
        return side, side, m * a * a / 12
    raise ValueError(f'unknown shape {shape!r}')


def item_sdf(name, colour, shape):
    """The Gazebo model of one item, centred on its origin. Where it goes is given when spawning.

    Its link is called "link": the suction cup's DetachableJoint plugins weld that link.
    """
    geo, dz = geometry(shape)
    ixx, iyy, izz = inertia(shape)
    r, g, b = COLOURS[colour]
    material = (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
                '<specular>0.05 0.05 0.05 1</specular></material>')
    return (f"<sdf version='1.10'><model name='{name}'><link name='link'>"
            f"<inertial><mass>{MASS}</mass>"
            f"<inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz></inertia></inertial>"
            f"{collisions(shape)}"
            f"<visual name='visual'><pose>0 0 {dz} 0 0 0</pose><geometry>{geo}</geometry>"
            f"{material}</visual>"
            f"</link></model></sdf>")


def item_sequence(config):
    """The items of a run, in order, from the settings in items.yaml (a dict).

    Yields dicts: colour, shape, lateral (m, sideways offset from the belt centre),
    yaw (radians) and gap (m of belt travel until the next item). The same seed always gives
    the same items in the same order. count 0 = keep going for ever.
    """
    rng = random.Random(config['seed'])
    lateral = config['lateral'] / 1000
    spacing = config['spacing'] / 1000
    number = 0
    while config['count'] == 0 or number < config['count']:
        number += 1
        yield {
            'number': number,
            'colour': rng.choice(config['colours']),
            'shape': rng.choice(config['shapes']),
            'lateral': rng.uniform(-lateral, lateral),
            'yaw': rng.uniform(-pi, pi),
            'gap': spacing + rng.uniform(-0.05, 0.05),
        }

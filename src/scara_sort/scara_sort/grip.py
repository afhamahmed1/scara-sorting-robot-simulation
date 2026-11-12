"""Which item is right under the suction cup: the suction's stand-in for a vacuum sensor.

Plain Python, no ROS: tested in test/test_grip.py.
"""

from math import hypot

from scara_sort.items import SIZE

SIDEWAYS = 0.015   # m: the item's centre must be this close to the cup's axis
BELOW = 0.005      # m: and its top at most this far under the cup tip
ABOVE = 0.001      # m: (or up to this much above it: rounding)


def item_under_cup(cup, items):
    """Name of the item right under the cup, or None.

    cup: (x, y, z) of the cup tip in the world. items: {name: (x, y, z) of its centre}.
    If several qualify, the one nearest the cup's axis.
    """
    cx, cy, cz = cup
    best, best_distance = None, SIDEWAYS
    for name, (x, y, z) in items.items():
        gap = cz - (z + SIZE / 2)   # cup tip to the item's top
        distance = hypot(x - cx, y - cy)
        if -ABOVE <= gap <= BELOW and distance <= best_distance:
            best, best_distance = name, distance
    return best

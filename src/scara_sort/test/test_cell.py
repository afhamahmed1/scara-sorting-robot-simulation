"""The cell layout fits the arm: every place the sorter needs to reach is reachable."""

from math import degrees

import pytest

from scara_sort.cell import BELT_X, BIN_INSIDE, BIN_WALL_TOP, BELT_TOP, BINS, ITEM_SIZE, PICK_ZONE
from scara_sort.kinematics import J1_MAX, TIP_TOP, ik

# Cup heights above the belt (m) the sorter uses
PICK_HEIGHT = ITEM_SIZE + 0.002                         # 2 mm over an item's top
PLACE_HEIGHT = BIN_WALL_TOP + 0.010 + ITEM_SIZE - BELT_TOP  # carried item 10 mm over the bin walls


def corners_and_centre(cx, cy, half):
    return [(cx + dx, cy + dy) for dx in (-half, 0, half) for dy in (-half, 0, half)]


@pytest.mark.parametrize('name', sorted(BINS))
def test_every_bin_reachable_at_its_centre_and_corners(name):
    for x, y in corners_and_centre(*BINS[name], BIN_INSIDE / 2):
        ik(x, y, PLACE_HEIGHT)   # raises if the arm can't get there
        ik(x, y, TIP_TOP)        # and with the cup fully up, as the arm travels


def test_whole_pick_zone_reachable():
    for x in (BELT_X[0], 0.36, 0.40, 0.44, BELT_X[1]):
        for y in (PICK_ZONE[0], -0.075, 0.0, 0.075, PICK_ZONE[1]):
            ik(x, y, PICK_HEIGHT)
            ik(x, y, TIP_TOP)


def test_shoulder_stays_inside_its_range_with_margin():
    worst = 0.0
    for centre in BINS.values():
        for x, y in corners_and_centre(*centre, BIN_INSIDE / 2):
            worst = max(worst, abs(ik(x, y, PLACE_HEIGHT)[0]))
    # The reject bin's far corner (-0.44, 0.00) is the hardest: 147°, inside the ±150° range
    assert degrees(worst) == pytest.approx(146.9, abs=0.1)
    assert worst <= J1_MAX


def test_carried_item_clears_items_on_the_belt_and_the_bin_walls():
    carried_bottom = BELT_TOP + TIP_TOP - ITEM_SIZE    # cup fully up, item hanging under it
    assert carried_bottom - (BELT_TOP + ITEM_SIZE) == pytest.approx(0.05)  # 50 mm over item tops
    assert carried_bottom - BIN_WALL_TOP == pytest.approx(0.06)            # 60 mm over the walls

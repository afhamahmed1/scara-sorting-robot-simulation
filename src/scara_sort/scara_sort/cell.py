"""Where everything in the sort cell is: the numbers from PLAN.md, in one place.

The same numbers are in scara_sort_sim/worlds/sort_cell.sdf; change both together.

Units: metres. The robot base is at (0, 0), x points forward and y to the robot's left.
Heights (z) are world heights, except where a name says "above the belt".
"""

BELT_TOP = 0.20                 # z of the belt surface (the driller's table height)
BELT_X = (0.32, 0.48)           # the belt's width
BELT_Y = (-0.70, 0.70)          # the belt's length; items move towards +y
BELT_CENTRE_X = 0.40

ITEM_SIZE = 0.04                # every item is 40 mm tall (and about 40 mm across)
ITEM_TOP = BELT_TOP + ITEM_SIZE  # the plane of all item tops, z 0.24

SPAWN = (0.40, -0.65)           # where new items appear (± a random sideways offset)

CAMERA = (0.40, -0.45, 0.85)    # above the belt, looking straight down
CAMERA_VIEW_Y = (-0.67, -0.23)  # the part of the belt it sees, on the item-top plane

PICK_LINE = 0.0                 # y where items are picked
PICK_ZONE = (-0.15, 0.15)       # y range in which the arm may pick from the belt

# Bins: centre (x, y) of each. Inside 0.12 x 0.12 m, floor at z 0.15, walls up to z 0.23.
BINS = {
    '1': (0.12, 0.40),
    '2': (-0.12, 0.42),
    '3': (-0.34, 0.24),
    'reject': (-0.38, 0.06),
}
BIN_INSIDE = 0.12
BIN_FLOOR = 0.15
BIN_WALL_TOP = 0.23

CATCH_BOX_X = (0.30, 0.50)      # on the floor past the end of the belt
CATCH_BOX_Y = (0.72, 0.92)

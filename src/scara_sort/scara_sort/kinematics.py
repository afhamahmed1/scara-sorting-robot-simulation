"""Kinematics of the SCARA arm: joint values <-> cup tip position.

The same arm as the driller: the suction cup's tip sits exactly where the drill tip was.

Units: metres and radians.
x, y: cup tip position seen from the robot's base (x forward, y to the left).
z:    cup tip height above the belt top (z 0.20 in the world, the driller's table height).

FK (forward kinematics): joints -> tip. Easy: add up the two links.
IK (inverse kinematics): tip -> joints. Solve the triangle base-elbow-tip.
"""

from math import acos, atan2, cos, degrees, hypot, radians, sin, sqrt

# Sizes, the same as urdf/scara.urdf.xacro in scara_sort_sim
L1 = 0.30  # shoulder (j1) to elbow (j2)
L2 = 0.25  # elbow (j2) to cup axis
TIP_TOP = 0.13  # tip height above the belt with the quill fully up (j3 = 0)

# Working range of each joint
J1_MAX = radians(150)
J2_MAX = radians(145)
J3_MAX = 0.15

# How close and how far from the base the tip can get (j2 = 145° and j2 = 0)
REACH_MIN = sqrt(L1**2 + L2**2 + 2 * L1 * L2 * cos(J2_MAX))
REACH_MAX = L1 + L2

EPS = 1e-9  # allow for rounding errors, so a point exactly on the edge of the range is accepted


def fk(j1, j2, j3):
    """Joint values -> cup tip (x, y, z)."""
    x = L1 * cos(j1) + L2 * cos(j1 + j2)
    y = L1 * sin(j1) + L2 * sin(j1 + j2)
    z = TIP_TOP - j3
    return x, y, z


def ik(x, y, z):
    """Cup tip (x, y, z) -> joint values (j1, j2, j3).

    Raises ValueError with the reason if the arm can't get there.
    """
    # Distance from the base axis to the tip, in the x-y plane
    r = hypot(x, y)
    if not REACH_MIN - EPS <= r <= REACH_MAX + EPS:
        raise ValueError(f'{r * 1000:.0f} mm from the base, but the arm reaches '
                         f'{REACH_MIN * 1000:.0f} to {REACH_MAX * 1000:.0f} mm')

    # Elbow angle from the law of cosines: r² = L1² + L2² + 2·L1·L2·cos(j2).
    # Two answers (+j2 and -j2); always taking +j2 means the arm never flips between points.
    c = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    j2 = acos(max(-1.0, min(1.0, c)))  # clamp: rounding can give 1.0000001 at full reach

    # Shoulder angle: direction to the tip, minus the angle the elbow bend adds
    j1 = atan2(y, x) - atan2(L2 * sin(j2), L1 + L2 * cos(j2))
    j1 = atan2(sin(j1), cos(j1))  # wrap into -180°..180°
    if abs(j1) > J1_MAX + EPS:
        raise ValueError(f'shoulder j1 would be {degrees(j1):.0f}°, '
                         f'outside ±{degrees(J1_MAX):.0f}°')

    # Height: the quill goes down by j3 from its top position
    j3 = TIP_TOP - z
    if not -EPS <= j3 <= J3_MAX + EPS:
        raise ValueError(f'height {z * 1000:.0f} mm, but the cup reaches '
                         f'{(TIP_TOP - J3_MAX) * 1000:.0f} to {TIP_TOP * 1000:.0f} mm '
                         'above the belt')

    return j1, j2, j3

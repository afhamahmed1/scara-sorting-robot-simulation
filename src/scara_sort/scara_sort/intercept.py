"""Where can the arm meet an item that is moving along the belt?

Plain Python, no ROS: tested in test/test_intercept.py.

Each joint motor turns at its top speed until it is there (j1 and j2 at 2 rad/s, j3 at
0.2 m/s), all at the same time, so a move takes as long as its slowest joint:

    move time = max(|Δj1| / 2, |Δj2| / 2, |Δj3| / 0.2)

The item moves along the belt at the belt's speed: y(t) = y_now + speed · t. The meeting
point is the first place on that path where the arm, setting off now, gets there no later
than the item (with a margin). After that the cup follows the item while it goes down,
grips and lifts, which takes follow_time, and all of that must happen inside the pick zone.
No such place = the arm can't catch the item.
"""

from scara_sort.cell import PICK_ZONE
from scara_sort.kinematics import ik

J1_SPEED = 2.0   # rad/s, as the motors in scara.urdf.xacro
J2_SPEED = 2.0   # rad/s
J3_SPEED = 0.2   # m/s
MARGIN = 0.25    # s added to every move: messages, settling, the moment of deciding
STEP = 0.01      # s between the meeting times tried


def move_time(start, end):
    """Seconds the arm needs from joint values start to end, (j1, j2, j3) each."""
    return max(abs(end[0] - start[0]) / J1_SPEED,
               abs(end[1] - start[1]) / J2_SPEED,
               abs(end[2] - start[2]) / J3_SPEED)


def intercept(joints, x, y, speed, height, follow_time, zone=PICK_ZONE, margin=MARGIN):
    """Where and when the arm can meet an item now at (x, y) moving +y at speed (m/s).

    joints: the arm's joint values now. height: cup height above the belt to meet it at.
    follow_time: seconds the cup then rides along with the item (lower, grip, lift).
    Returns (t, y_meet): the item reaches y_meet t seconds from now and the arm is there
    by then. None if the arm can't do it all inside the pick zone.
    """
    t = 0.0
    while True:
        y_t = y + speed * t
        if y_t + speed * follow_time > zone[1]:
            return None   # too late: it would leave the pick zone before the arm is done
        if y_t >= zone[0]:
            try:
                target = ik(x, y_t, height)
            except ValueError:
                target = None   # out of reach here (can't happen inside the pick zone)
            if target is not None and move_time(joints, target) + margin <= t:
                return t, y_t
        if speed <= 0 and y_t < zone[0]:
            return None   # not moving and not in the pick zone: it never comes
        t += STEP
        if speed <= 0 and t > 10.0:
            return None

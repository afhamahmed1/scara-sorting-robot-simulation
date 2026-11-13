"""Sort the items coming down the belt into the bins.

    ros2 run scara_sort sorter --ros-args -p rules:=config/sort_by_colour.yaml
    ros2 run scara_sort sorter --ros-args -p mode:=move -p speed:=0.10

Stop-and-go (mode:=stop, the default):

1. wait at the ready pose, above the pick line, 60 mm over the item tops
2. when the next item's predicted position reaches y = -0.02, stop the belt
3. move above the item, lower to 2 mm over its top, suction on; if nothing was gripped,
   lift, log the miss and let the item ride on into the catch box
4. lift fully, which also restarts the belt (the arm is now clear of it), move over the
   item's bin, lower to 10 mm above the walls, suction off, back to the ready pose
5. log, e.g. '#7 red cube → bin 1 (3.4 s)'

On the move (mode:=move), the belt never stops:

- intercept: from the item's path and how long the arm needs to get there (each joint at its
  top speed, plus a margin), pick the first point in the pick zone where the arm can be
  before the item (intercept.py). Move there, 20 mm over the item tops, and wait for it.
- follow: every 20 ms send the arm to where the item is now (aimed a little ahead, for the
  arm's lag), while lowering the cup to 2 mm over the item, gripping (once the cup is over
  it), and lifting until the item is 30 mm over the belt. Then swing over to the bin, the cup
  rising fully on the way, place it as before, and go straight on to the next item.
- skip: if the arm can't catch an item before it leaves the pick zone, it logs
  'missed #9 (arm busy)' and lets it ride into the catch box. A skipped item never goes
  into a wrong bin.

Which bin: the first rule in the rules file that matches the item's colour and shape.
The sorter works only from the camera (/items) and the belt's encoder, as on a real belt.
Ctrl-C stops the belt and releases the suction.

Publishes:   /conveyor/cmd_vel  (std_msgs/Float64), /scara/j1_cmd ... j3_cmd  (std_msgs/Float64)
             /sort_markers  (visualization_msgs/MarkerArray) for RViz
Subscribes:  /items  (scara_sort_msgs/ItemArray), /conveyor/odom  (nav_msgs/Odometry),
             /joint_states  (sensor_msgs/JointState)
Uses:        /suction  (std_srvs/SetBool)
"""

import sys
from math import hypot

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray

from scara_sort.cell import (BELT_CENTRE_X, BELT_TOP, BELT_X, BIN_INSIDE, BIN_WALL_TOP, BINS,
                             ITEM_SIZE, PICK_LINE, PICK_ZONE)
from scara_sort.config import load_yaml
from scara_sort.intercept import intercept
from scara_sort.kinematics import TIP_TOP, fk, ik
from scara_sort.move_to import JOINTS
from scara_sort.pick_place import PICK_HEIGHT, READY, READY_HEIGHT, Arm
from scara_sort.rules import bin_for, check_rules
from scara_sort_msgs.msg import ItemArray

READY_POINT = (*READY, READY_HEIGHT)
STOP_AT = PICK_LINE - 0.02   # m: stop the belt when an item's predicted y reaches this

# On the move
HOVER_HEIGHT = ITEM_SIZE + 0.020   # m above the belt: meet items 20 mm over their tops
FOLLOW_TIME = 0.6    # s the cup rides along with an item: lower, grip, lift clear (measured ~0.4)
CLEAR_HEIGHT = ITEM_SIZE + 0.002 + 0.030   # m: lifted this far, the item is 30 mm over the belt
LEAD = 0.03          # s: aim this far ahead of the item, for the arm's lag (measured, see README)
FOLLOW_PERIOD = 0.02  # s between follow commands
OVER_ITEM = 0.006    # m: grip only once the cup is this close over the item (it may have to catch up)

# RViz colours of the item colours
RGB = {'red': (0.9, 0.1, 0.1), 'green': (0.1, 0.7, 0.2), 'blue': (0.1, 0.3, 0.9),
       'yellow': (0.95, 0.85, 0.1), 'unknown': (0.6, 0.6, 0.6)}


def marker(ns, marker_id, marker_type, xyz, scale, rgba):
    """An RViz marker in the world frame."""
    m = Marker(ns=ns, id=marker_id, type=marker_type, action=Marker.ADD)
    m.header.frame_id = 'world'
    m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = map(float, scale)
    m.color.r, m.color.g, m.color.b, m.color.a = map(float, rgba)
    return m


class Sorter(Arm):

    def __init__(self):
        super().__init__('sorter')
        self.rules_file = self.declare_parameter('rules', 'config/sort_by_colour.yaml').value
        self.rules = load_yaml(self.rules_file)['rules']
        self.speed = self.declare_parameter('speed', 0.05).value   # m/s
        self.mode = self.declare_parameter('mode', 'stop').value     # stop or move
        self.following = None      # item the cup is riding along with (on the move)
        self.follow_height = CLEAR_HEIGHT
        self.items = []         # latest /items: list of Item
        self.items_belt = 0.0   # encoder reading when they arrived
        self.belt = None        # latest encoder reading (m)
        self.belt_speed = 0.0   # m/s, measured
        self.belt_cmd = None    # m/s, last commanded
        self.handled = set()    # ids of items picked, missed or skipped
        self.sorted = 0

        self.belt_pub = self.create_publisher(Float64, '/conveyor/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/sort_markers', 10)
        self.create_subscription(ItemArray, '/items', self.on_items, 10)
        self.create_subscription(Odometry, '/conveyor/odom', self.on_odom, 10)
        self.create_timer(0.02, self.watch_belt)
        self.create_timer(FOLLOW_PERIOD, self.follow)
        self.create_timer(0.2, self.publish_markers)

    # ----- what comes in -----

    def on_odom(self, msg):
        self.belt = msg.pose.pose.position.x
        self.belt_speed = msg.twist.twist.linear.x

    def on_items(self, msg):
        if self.belt is not None:
            self.items, self.items_belt = list(msg.items), self.belt

    def where(self, item):
        """(x, y) of an item now: its reported position moved on by the belt since then."""
        return item.x, item.y + (self.belt - self.items_belt)

    def waiting(self):
        """Items not handled yet that are still before the end of the pick zone."""
        return [i for i in self.items
                if i.id not in self.handled and self.where(i)[1] <= PICK_ZONE[1]]

    # ----- the belt -----

    def set_belt(self, speed):
        if speed != self.belt_cmd:
            self.belt_cmd = speed
            self.belt_pub.publish(Float64(data=speed))

    def belt_stopped(self):
        return self.belt_cmd == 0.0 and abs(self.belt_speed) < 1e-6

    def watch_belt(self):
        """Stop the belt the moment an item reaches the stop point (runs 50 times a second)."""
        if self.belt is None or self.belt_cmd == 0.0 or self.mode != 'stop':
            return
        if any(self.where(i)[1] >= STOP_AT for i in self.waiting()):
            self.set_belt(0.0)

    # ----- the sort cycle -----

    def run(self):
        problems = check_rules(self.rules)
        if problems:
            self.get_logger().error(f'Rules refused: {"; ".join(problems)}')
            return
        if not self.suction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the suction node...')
            while not self.suction_client.wait_for_service(timeout_sec=1.0):
                pass
        if self.mode not in ('stop', 'move'):
            self.get_logger().error(f'mode must be stop or move, not {self.mode!r}')
            return
        how = 'stop-and-go' if self.mode == 'stop' else 'on the move'
        self.get_logger().info(f'Sorting by {self.rules_file}, {how}, belt at '
                               f'{self.speed * 1000:.0f} mm/s')
        self.ready()
        while self.belt is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.set_belt(self.speed)
        while rclpy.ok():
            if self.mode == 'stop':
                self.sort_item(self.next_item())
            else:
                self.catch_next()

    def next_item(self):
        """Wait until the belt has stopped for an item at the stop point, and return it."""
        while True:
            rclpy.spin_once(self, timeout_sec=0.02)
            if not self.belt_stopped():
                continue
            ready = [i for i in self.waiting() if self.where(i)[1] >= STOP_AT]
            if ready:
                return max(ready, key=lambda i: self.where(i)[1])
            self.set_belt(self.speed)   # stopped, but the item is gone: go on

    def sort_item(self, item):
        start = self.get_clock().now()
        x, y = self.where(item)
        target = bin_for(self.rules, item.colour, item.shape)
        self.handled.add(item.id)
        gripped, message = self.pick(x, y)   # ends with the cup fully up
        self.set_belt(self.speed)            # the arm is clear of the belt
        if not gripped:
            self.get_logger().warn(f'#{item.id} {item.colour} {item.shape} at '
                                   f'({x * 1000:.0f}, {y * 1000:.0f}) mm: missed ({message}), '
                                   'it rides on into the catch box')
        else:
            self.place(target)
            seconds = (self.get_clock().now() - start).nanoseconds * 1e-9
            self.sorted += 1
            self.get_logger().info(f'#{item.id} {item.colour} {item.shape} → bin {target} '
                                   f'({seconds:.1f} s)')
        self.ready()

    # ----- on the move -----

    def find(self, item_id):
        """The item with this id in the latest /items, or None."""
        return next((i for i in self.items if i.id == item_id), None)

    def y_now(self, item_id, last):
        """Where item item_id is along the belt now (from the latest /items), else last."""
        item = self.find(item_id)
        return self.where(item)[1] if item is not None else last

    def cup_over(self, item_id):
        """Is the cup over item item_id now (by the joint angles and the item's prediction)?"""
        item = self.find(item_id)
        if item is None:
            return False
        x, y = self.where(item)
        cx, cy, _ = fk(*(self.joints[name] for name in JOINTS))
        return hypot(x - cx, y - cy) <= OVER_ITEM

    def follow(self):
        """Every 20 ms while following: send the arm to where the item is now (a bit ahead)."""
        item = self.find(self.following) if self.following is not None else None
        if item is None or self.belt is None:
            return
        x, y = self.where(item)
        try:
            self.send(ik(x, y + self.belt_speed * LEAD, self.follow_height))
        except ValueError:
            pass  # out of reach: keep the last command

    def wait_until(self, condition, timeout=5.0):
        """Spin until condition() is true. False on timeout (simulated seconds)."""
        end = self.seconds() + timeout
        while not condition():
            if self.seconds() > end:
                return False
            rclpy.spin_once(self, timeout_sec=0.005)
        return True

    def catch_next(self):
        """Choose the next item the arm can catch, catch it and place it."""
        joints = [self.joints.get(name, 0.0) for name in JOINTS]
        for item in sorted(self.waiting(), key=lambda i: -self.where(i)[1]):
            x, y = self.where(item)
            plan = intercept(joints, x, y, self.speed, HOVER_HEIGHT, FOLLOW_TIME)
            if plan is None:
                self.handled.add(item.id)
                self.get_logger().warn(f'missed #{item.id} (arm busy): at y {y * 1000:.0f} mm '
                                       'the arm can no longer catch it, it rides on into the '
                                       'catch box')
                continue
            self.catch(item, plan[1])
            return
        # Nothing to catch now: wait at the ready pose
        if not self.arrived(ik(*READY_POINT)):
            self.ready()
        rclpy.spin_once(self, timeout_sec=0.05)

    def catch(self, item, y_meet):
        start = self.get_clock().now()
        x = item.x
        target = bin_for(self.rules, item.colour, item.shape)
        self.handled.add(item.id)
        self.tip_to(x, y_meet, HOVER_HEIGHT)                  # get there first, and wait
        self.wait_until(lambda: self.y_now(item.id, y_meet) >= y_meet - 0.003, timeout=30.0)

        self.following, self.follow_height = item.id, PICK_HEIGHT  # ride along, going down
        down = self.wait_until(lambda: abs(self.joints['j3'] - (TIP_TOP - PICK_HEIGHT)) < 0.0005
                               and self.cup_over(item.id))
        gripped, message = self.suction(True) if down else (False, 'the cup did not get down')
        # Lift, still riding along, until the item is clear of the belt and its rails; the cup
        # goes on up to fully up while the arm swings over to the bin
        self.follow_height = CLEAR_HEIGHT
        self.wait_until(lambda: abs(self.joints['j3'] - (TIP_TOP - CLEAR_HEIGHT)) < 0.0005)
        self.following = None
        if not gripped:
            self.get_logger().warn(f'#{item.id} {item.colour} {item.shape}: missed ({message}), '
                                   'it rides on into the catch box')
            return
        self.place(target)
        seconds = (self.get_clock().now() - start).nanoseconds * 1e-9
        self.sorted += 1
        off = message.split(', ', 1)[1] if ', ' in message else ''  # how well the cup kept up
        self.get_logger().info(f'#{item.id} {item.colour} {item.shape} → bin {target} '
                               f'({seconds:.1f} s; met at y {y_meet * 1000:.0f} mm, {off})')

    def stop(self):
        """Ctrl-C: belt off, suction off."""
        self.belt_pub.publish(Float64(data=0.0))
        try:
            self.suction(False, timeout=2.0)
        except TimeoutError:
            pass

    # ----- RViz -----

    def publish_markers(self):
        if self.belt is None:
            return
        out = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        zone = marker('pick zone', 0, Marker.CUBE,
                      (BELT_CENTRE_X, (PICK_ZONE[0] + PICK_ZONE[1]) / 2, BELT_TOP + 0.001),
                      (BELT_X[1] - BELT_X[0], PICK_ZONE[1] - PICK_ZONE[0], 0.002),
                      (0.3, 0.6, 1.0, 0.3))
        line = marker('pick zone', 1, Marker.LINE_LIST, (0.0, 0.0, 0.0), (0.003, 0, 0),
                      (0.3, 0.6, 1.0, 1.0))
        line.points = [Point(x=BELT_X[0], y=PICK_LINE, z=BELT_TOP + 0.002),
                       Point(x=BELT_X[1], y=PICK_LINE, z=BELT_TOP + 0.002)]
        out.markers += [zone, line]
        for k, (name, (bx, by)) in enumerate(BINS.items()):
            box = marker('bins', k, Marker.CUBE, (bx, by, BIN_WALL_TOP - 0.04),
                         (BIN_INSIDE, BIN_INSIDE, 0.08), (0.3, 0.5, 0.8, 0.25))
            label = marker('bin labels', k, Marker.TEXT_VIEW_FACING,
                           (bx, by, BIN_WALL_TOP + 0.05), (0, 0, 0.03), (1, 1, 1, 1))
            label.text = f'bin {name}'
            out.markers += [box, label]
        for item in self.items:
            x, y = self.where(item)
            if y > PICK_ZONE[1] + 0.05 and item.id in self.handled:
                continue
            shape = {'cube': Marker.CUBE, 'cylinder': Marker.CYLINDER}.get(item.shape,
                                                                            Marker.SPHERE)
            body = marker('items', item.id, shape, (x, y, BELT_TOP + ITEM_SIZE / 2),
                          (ITEM_SIZE, ITEM_SIZE, ITEM_SIZE), (*RGB.get(item.colour, RGB['unknown']), 0.8))
            label = marker('item labels', item.id, Marker.TEXT_VIEW_FACING,
                           (x, y, BELT_TOP + ITEM_SIZE + 0.03), (0, 0, 0.02), (1, 1, 1, 1))
            label.text = f'#{item.id} → {bin_for(self.rules, item.colour, item.shape)}'
            out.markers += [body, label]
        self.marker_pub.publish(out)


def main():
    # Ctrl-C raises KeyboardInterrupt but leaves ROS running, so the belt and the suction can
    # still be switched off
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Sorter()
    ok = True
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info(f'Stopped with Ctrl-C after {node.sorted} items: belt and '
                               'suction off')
        node.stop()
    except TimeoutError as error:
        node.get_logger().error(f'Stopped: {error}')
        node.stop()
        ok = False
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

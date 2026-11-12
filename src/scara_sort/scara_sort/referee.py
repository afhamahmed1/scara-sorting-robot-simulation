"""Referee: score where every item really ended up, using the truth from Gazebo.

    ros2 run scara_sort referee --ros-args -p rules:=config/sort_by_colour.yaml

Gets each item's true colour and shape from the spawner (/items/spawned), works out its bin
from the same rules file the sorter uses, and watches its true position (/gz/poses). Once an
item has lain still for 1 s off the belt, its outcome is recorded:

    correct    in the bin its kind belongs to
    wrong bin  in another bin
    missed     in the catch box (nobody picked it)
    dropped    anywhere else

When all items of the run have an outcome (count from items.yaml), or on Ctrl-C, it prints:

    30 items: 30 correct, 0 wrong bin, 0 missed, 0 dropped — 9.6 items/min

items/min counts the correct items per minute of simulated time, from the first item
appearing to the last outcome.

A checker: it may use the truth. The detector and the sorter may not.

Subscribes:  /items/spawned  (scara_sort_msgs/SpawnedItem), /gz/poses  (tf2_msgs/TFMessage)
"""

from dataclasses import dataclass
from math import dist

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from scara_sort.cell import (BELT_TOP, BELT_X, BELT_Y, BIN_INSIDE, BINS, CATCH_BOX_X,
                             CATCH_BOX_Y, ITEM_SIZE)
from scara_sort.config import load_yaml
from scara_sort.rules import bin_for
from scara_sort_msgs.msg import SpawnedItem

STILL_FOR = 1.0    # s of simulated time
STILL = 0.001      # m: moving less than this counts as lying still
OUTCOMES = ['correct', 'wrong bin', 'missed', 'dropped']


@dataclass
class Item:
    number: int
    name: str
    kind: str
    target: str                  # the bin it belongs in
    position: tuple = None
    still_at: tuple = None       # where it was when it last stopped moving
    still_since: float = None    # sim time (s)
    outcome: str = None
    place: str = ''              # where it ended up, in words


def where(x, y, z):
    """Where a position is: ('bin', name), ('catch box',), ('belt',), ('air',) or ('floor',)."""
    for name, (bx, by) in BINS.items():
        if abs(x - bx) <= BIN_INSIDE / 2 + 0.005 and abs(y - by) <= BIN_INSIDE / 2 + 0.005:
            return ('bin', name)
    if CATCH_BOX_X[0] <= x <= CATCH_BOX_X[1] and CATCH_BOX_Y[0] <= y <= CATCH_BOX_Y[1] \
            and z < BELT_TOP:
        return ('catch box',)
    if BELT_X[0] - 0.01 <= x <= BELT_X[1] + 0.01 and BELT_Y[0] <= y <= BELT_Y[1] \
            and z >= BELT_TOP:
        return ('belt',) if z < BELT_TOP + ITEM_SIZE else ('air',)
    if z > BELT_TOP + ITEM_SIZE:
        return ('air',)   # carried by the arm
    return ('floor',)


class Referee(Node):

    def __init__(self):
        super().__init__('referee', parameter_overrides=[Parameter('use_sim_time', value=True)])
        rules_file = self.declare_parameter('rules', 'config/sort_by_colour.yaml').value
        self.rules = load_yaml(rules_file)['rules']
        items_file = self.declare_parameter('items', 'config/items.yaml').value
        self.count = self.declare_parameter('count', load_yaml(items_file)['count']).value
        self.items = {}      # number -> Item
        self.by_name = {}    # model name -> number of the item that has it now
        self.start = None    # sim time of the first item
        self.end = None      # sim time of the last outcome
        self.finished = False
        self.create_subscription(
            SpawnedItem, '/items/spawned', self.on_spawned,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(TFMessage, '/gz/poses', self.on_poses, 10)
        self.get_logger().info(f"Refereeing {self.count or 'endless'} items with {rules_file}")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_spawned(self, msg):
        if self.start is None:
            self.start = self.now()
        kind = f'{msg.colour} {msg.shape}'
        self.items[msg.number] = Item(msg.number, msg.name, kind,
                                      bin_for(self.rules, msg.colour, msg.shape))
        self.by_name[msg.name] = msg.number

    def on_poses(self, msg):
        now = self.now()
        for t in msg.transforms:
            number = self.by_name.get(t.child_frame_id)
            if number is None:
                continue
            item = self.items[number]
            p = t.transform.translation
            item.position = (p.x, p.y, p.z)
            if item.outcome:
                continue
            if item.still_at is None or dist(item.position, item.still_at) > STILL:
                item.still_at, item.still_since = item.position, now   # it moved
            elif now - item.still_since >= STILL_FOR:
                self.judge(item, now)

    def judge(self, item, now):
        place = where(*item.position)
        if place[0] in ('belt', 'air'):
            return  # still on its way (the belt may just be stopped)
        if place[0] == 'bin':
            item.outcome = 'correct' if place[1] == item.target else 'wrong bin'
            item.place = f'bin {place[1]}'
        elif place[0] == 'catch box':
            item.outcome, item.place = 'missed', 'the catch box'
        else:
            x, y, _ = item.position
            item.outcome, item.place = 'dropped', f'({x * 1000:.0f}, {y * 1000:.0f}) mm'
        self.end = now
        mark = {'correct': '✓', 'wrong bin': f'✗ wrong bin, belongs in bin {item.target}',
                'missed': '✗ missed', 'dropped': '✗ dropped'}[item.outcome]
        self.get_logger().info(f'#{item.number} {item.name} {item.kind}: {item.place} {mark}')
        done = [i for i in self.items.values() if i.outcome]
        if self.count and len(done) >= self.count and not self.finished:
            self.finished = True
            self.print_table()

    def print_table(self):
        done = [i for i in self.items.values() if i.outcome]
        counts = {o: sum(i.outcome == o for i in done) for o in OUTCOMES}
        minutes = (self.end - self.start) / 60 if self.end and self.start is not None else 0
        rate = counts['correct'] / minutes if minutes > 0 else 0.0
        open_items = len(self.items) - len(done)
        extra = f' ({open_items} still on their way)' if open_items else ''
        self.get_logger().info(
            f"{len(done)} items: {counts['correct']} correct, {counts['wrong bin']} wrong bin, "
            f"{counts['missed']} missed, {counts['dropped']} dropped — {rate:.1f} items/min"
            f'{extra}')


def main():
    rclpy.init()
    node = Referee()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        if not node.finished:
            node.print_table()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

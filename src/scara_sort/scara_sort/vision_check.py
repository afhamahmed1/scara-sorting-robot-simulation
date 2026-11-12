"""Check the item detector against the truth from Gazebo.

    ros2 run scara_sort vision_check

For every item on /items, finds the real item at that place on the belt (Gazebo's true
positions, /gz/poses) and its real colour and shape (/items/spawned). Once the item has passed
the pick line (or been picked up), it reports what the detector called it by then, the
largest position error while it was under the camera, and the error at the pick line, where
the arm picks it (predicted from the encoder, long after the camera last saw it):

    #4 red cube = item_03 (#4 red cube): 1.1 mm under the camera, 1.9 mm at the pick line
    4 items: 4 named right, 0 wrong; worst error 1.3 mm under the camera, 2.0 mm at the pick line

A checker: it may use the truth. The detector and the sorter may not.

Subscribes:  /items  (scara_sort_msgs/ItemArray), what the detector says
             /items/spawned  (scara_sort_msgs/SpawnedItem), /gz/poses  (tf2_msgs/TFMessage): the truth
"""

from collections import deque
from math import hypot

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from scara_sort.cell import BELT_TOP, CAMERA_VIEW_Y, ITEM_SIZE, PICK_LINE
from scara_sort_msgs.msg import ItemArray, SpawnedItem

MATCH = 0.025       # m: the real item must be this close to the reported position
AT_PICK_LINE = 0.025  # m: errors this close to the pick line count as "at the pick line"


def nanoseconds(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


class Truth:
    """Recent true positions of one Gazebo model, to look up its position at any moment."""

    def __init__(self):
        self.history = deque(maxlen=120)  # (ns, x, y, z), about 2 s

    def add(self, t, x, y, z):
        self.history.append((t, x, y, z))

    def at(self, t):
        """Position at time t (ns), interpolated between the two nearest readings."""
        h = self.history
        if not h:
            return None
        if t <= h[0][0]:
            return h[0][1:]
        for (t0, *p0), (t1, *p1) in zip(h, list(h)[1:]):
            if t0 <= t <= t1 and t1 > t0:
                f = (t - t0) / (t1 - t0)
                return tuple(a + (b - a) * f for a, b in zip(p0, p1))
        return h[-1][1:]


class VisionCheck(Node):

    def __init__(self):
        super().__init__('vision_check',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.spawned = {}  # model name -> (number, colour, shape) of the item that has it now
        self.truth = {}    # model name -> Truth
        self.matches = {}  # detector id -> model name
        self.results = {}  # detector id -> dict
        self.create_subscription(
            SpawnedItem, '/items/spawned', self.on_spawned,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(TFMessage, '/gz/poses', self.on_poses, 10)
        self.create_subscription(ItemArray, '/items', self.on_items, 10)
        self.get_logger().info('Comparing /items with the truth...')

    def on_spawned(self, msg):
        self.spawned[msg.name] = (msg.number, msg.colour, msg.shape)
        self.truth[msg.name] = Truth()  # a new item with this name: forget the old one's path

    def on_poses(self, msg):
        for t in msg.transforms:
            if t.child_frame_id in self.truth:
                p = t.transform.translation
                self.truth[t.child_frame_id].add(nanoseconds(t.header.stamp), p.x, p.y, p.z)

    def on_belt(self, position):
        return position is not None and abs(position[2] - (BELT_TOP + ITEM_SIZE / 2)) < 0.005

    def on_items(self, msg):
        t = nanoseconds(msg.header.stamp)
        for item in msg.items:
            name = self.matches.get(item.id)
            if name is None:
                name = self.find(item, t)
                if name is None:
                    continue
                self.matches[item.id] = name
                self.first_seen(item, name)
            position = self.truth[name].at(t)
            result = self.results[item.id]
            if result['done']:
                continue
            if not self.on_belt(position) or position[1] > PICK_LINE + AT_PICK_LINE:
                result['done'] = True   # past the pick line, or picked up by the arm
                self.report(item.id)
                continue
            result['seen'] = f'{item.colour} {item.shape}'   # its latest answer
            error = hypot(item.x - position[0], item.y - position[1]) * 1000  # mm
            if CAMERA_VIEW_Y[0] <= position[1] <= CAMERA_VIEW_Y[1]:
                result['camera'] = max(result['camera'], error)
            if abs(position[1] - PICK_LINE) <= AT_PICK_LINE:
                result['pick'] = max(result['pick'], error)

    def find(self, item, t):
        """The real item at the reported position, or None."""
        best, best_distance = None, MATCH
        for name, truth in self.truth.items():
            position = truth.at(t)
            if not self.on_belt(position):
                continue
            distance = hypot(item.x - position[0], item.y - position[1])
            if distance < best_distance:
                best, best_distance = name, distance
        return best

    def first_seen(self, item, name):
        number, colour, shape = self.spawned[name]
        self.results[item.id] = dict(name=name, number=number, kind=f'{colour} {shape}',
                                     seen=f'{item.colour} {item.shape}', right=False,
                                     camera=0.0, pick=-1.0, done=False)

    def report(self, item_id):
        r = self.results[item_id]
        r['right'] = r['seen'] == r['kind']
        mark = '=' if r['right'] else '≠ WRONG:'
        pick = f"{r['pick']:.1f} mm" if r['pick'] >= 0 else 'not seen'
        self.get_logger().info(
            f"#{item_id} {r['seen']} {mark} {r['name']} (#{r['number']} {r['kind']}): "
            f"{r['camera']:.1f} mm under the camera, {pick} at the pick line")
        done = [r for r in self.results.values() if r['done']]
        right = sum(r['right'] for r in done)
        worst_pick = max([r['pick'] for r in done if r['pick'] >= 0], default=0.0)
        self.get_logger().info(
            f'{len(done)} items: {right} named right, {len(done) - right} wrong; worst error '
            f"{max(r['camera'] for r in done):.1f} mm under the camera, "
            f'{worst_pick:.1f} mm at the pick line')


def main():
    rclpy.init()
    node = VisionCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

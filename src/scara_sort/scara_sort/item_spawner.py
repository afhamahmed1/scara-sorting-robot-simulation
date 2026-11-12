"""Put items on the start of the belt, one every 250 mm (± 50 mm) of belt travel.

    ros2 run scara_sort item_spawner
    ros2 run scara_sort item_spawner --ros-args -p count:=10 -p seed:=2

Items are spawned by belt travel, read from the belt's encoder, not by time: when the belt
stops, new items stop arriving too. Each item appears at (0.40 ± 0.03, -0.65) m, turned by a
random angle. The settings are in config/items.yaml; count, seed, spacing (mm) and lateral (mm)
can also be given with -p, and items:=<file> reads another settings file.

Item names come from a pool, item_00 ... item_23 (the suction cup needs every name in advance).
Before a name is used again, the oldest item, which had it, is removed from Gazebo: right
after an item is spawned, so that it is surely gone by the time the next one appears.

Subscribes:  /conveyor/odom  (nav_msgs/Odometry), the belt's encoder
Publishes:   /items/spawned  (scara_sort_msgs/SpawnedItem), the truth about each item, for the
             referee and vision_check only
"""

import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from scara_sort.cell import BELT_TOP, SPAWN
from scara_sort.config import load_yaml
from scara_sort.gz import GzWorld
from scara_sort.items import SIZE, item_sdf, item_sequence
from scara_sort_msgs.msg import SpawnedItem

POOL = 24      # item names item_00 ... item_23, used in turn
DROP = 0.0005  # m, items appear this far above the belt and settle onto it


def pool_name(number):
    """Model name of item number 1, 2, 3, ...: item_00, item_01, ... item_23, item_00, ..."""
    return f'item_{(number - 1) % POOL:02d}'


class ItemSpawner(Node):

    def __init__(self):
        super().__init__('item_spawner')
        settings_file = self.declare_parameter('items', 'config/items.yaml').value
        self.settings = load_yaml(settings_file)
        for key in ('count', 'seed', 'spacing', 'lateral'):  # -p overrides the file
            self.settings[key] = self.declare_parameter(key, self.settings[key]).value
        self.items = item_sequence(self.settings)
        self.belt = None     # latest encoder reading (m)
        self.next_at = None  # encoder reading at which the next item appears

        # Latched (transient local), so the referee gets every item even if it starts late
        self.spawned_pub = self.create_publisher(
            SpawnedItem, '/items/spawned',
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/conveyor/odom', self.on_odom, 10)
        self.gz = GzWorld(self)

    def on_odom(self, msg):
        self.belt = msg.pose.pose.position.x  # metres the belt has moved

    def run(self):
        """Spawn the items as the belt moves on. Returns when all are spawned."""
        if not self.gz.wait():
            self.get_logger().error('No /world/sort_cell/create service: is the simulation running?')
            return
        self.remove_old_items()
        count = self.settings['count'] or 'endless'
        self.get_logger().info(
            f"{count} items, seed {self.settings['seed']}, one every "
            f"{self.settings['spacing']} ± 50 mm of belt travel")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.belt is None or (self.next_at is not None and self.belt < self.next_at):
                continue
            item = next(self.items, None)
            if item is None:
                self.get_logger().info('All items spawned')
                return
            self.spawn(item)
            # Measured from where this item should have appeared, so the spacing doesn't drift
            self.next_at = (self.belt if self.next_at is None else self.next_at) + item['gap']

    def remove_old_items(self):
        """Remove the items of an earlier run from Gazebo, if there are any."""
        for k in range(POOL):
            self.gz.remove(pool_name(k + 1))
        end = time.time() + 1.0   # let Gazebo take them away before the names are used again
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def spawn(self, item):
        number, name = item['number'], pool_name(item['number'])
        x, y = SPAWN[0] + item['lateral'], SPAWN[1]
        sdf = item_sdf(name, item['colour'], item['shape'])
        if not self.gz.create(sdf, x, y, BELT_TOP + SIZE / 2 + DROP, item['yaw']):
            self.get_logger().error(f'Could not spawn {name}: is the simulation running?')
            return
        self.spawned_pub.publish(SpawnedItem(number=number, name=name, colour=item['colour'],
                                             shape=item['shape']))
        self.get_logger().info(f"#{number} {name}: {item['colour']} {item['shape']} "
                               f'at x {x * 1000:.0f} mm')
        # The next item takes the name of the oldest one: remove that one now
        if number + 1 > POOL:
            self.gz.remove(pool_name(number + 1))


def main():
    rclpy.init()
    node = ItemSpawner()
    try:
        node.run()
        rclpy.spin(node)  # keep /items/spawned up for latecomers
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

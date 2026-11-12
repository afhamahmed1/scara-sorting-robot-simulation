"""Put one item on the belt, for trying things out by hand.

    ros2 run scara_sort put_item COLOUR SHAPE X Y     (X, Y in mm)
    ros2 run scara_sort put_item red cube 400 0

COLOUR: red, green, blue or yellow. SHAPE: cube, cylinder or prism. It takes the first item
name (item_00 ... item_23) that isn't in use. Start the suction node first, or the suction cup
grabs the item the moment it appears (see suction.py).
"""

import sys

import rclpy
from rclpy.utilities import remove_ros_args

from scara_sort.cell import BELT_TOP
from scara_sort.gz import GzWorld
from scara_sort.item_spawner import DROP, POOL, pool_name
from scara_sort.items import COLOURS, SHAPES, SIZE, item_sdf


def main():
    args = remove_ros_args(sys.argv)[1:]
    if len(args) != 4 or args[0] not in COLOURS or args[1] not in SHAPES:
        print(f"Usage: ros2 run scara_sort put_item COLOUR SHAPE X Y   (mm; COLOUR: "
              f"{', '.join(COLOURS)}; SHAPE: {', '.join(SHAPES)})")
        sys.exit(1)
    colour, shape = args[0], args[1]
    x, y = float(args[2]) / 1000, float(args[3]) / 1000

    rclpy.init()
    node = rclpy.create_node('put_item')
    world = GzWorld(node)
    ok = False
    if not world.wait():
        print('No /world/sort_cell/create service: is the simulation running?')
    else:
        present = world.items_present() or []
        free = [pool_name(k + 1) for k in range(POOL) if pool_name(k + 1) not in present]
        if not free:
            print('All 24 item names are in use')
        else:
            ok = world.create(item_sdf(free[0], colour, shape), x, y, BELT_TOP + SIZE / 2 + DROP)
            print(f'{free[0]}: {colour} {shape} at ({x * 1000:.0f}, {y * 1000:.0f}) mm'
                  if ok else 'Gazebo did not take it')
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

"""The suction cup: simulated hardware that grips the item right under the cup.

    ros2 run scara_sort suction
    ros2 service call /suction std_srvs/srv/SetBool "{data: true}"    # on
    ros2 service call /suction std_srvs/srv/SetBool "{data: false}"   # off

Gazebo has no suction, so this fakes it in a way that behaves like the real thing as far as
the sorter can tell:

  on   looks up which item is right under the cup (within 15 mm sideways, its top 0 to 5 mm
       below the cup), using the true positions from Gazebo, and welds it to the quill with
       that item's DetachableJoint plugin. With nothing there it answers
       success: false, "nothing under the cup", like a vacuum sensor that never builds up
       pressure.
  off  releases the item.

The URDF has one DetachableJoint plugin per item name (item_00 ... item_23), each with its own
attach, detach and state topics: /scara/suction/item_07/attach etc. This node uses them through
Gazebo's own transport (no bridge needed).

Warm-up: each plugin welds its item as soon as that name first appears in the world. So at
startup, with the arm standing still, this node spawns all 24 names as invisible dummies
off to the side, waits until they are welded, releases them and removes them. After that a
name is only welded when this node asks. The /suction service appears once that is done; the
item spawner waits for it.

Subscribes:  /joint_states  (sensor_msgs/JointState), where the cup is
             /gz/poses  (tf2_msgs/TFMessage), the items' true positions (simulated hardware
             may use the truth)
Service:     /suction  (std_srvs/SetBool)
"""

import threading
import time

import rclpy
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node as GzNode
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

from scara_sort.cell import BELT_TOP
from scara_sort.grip import item_under_cup, nearest_item
from scara_sort.gz import GzWorld
from scara_sort.item_spawner import POOL, pool_name
from scara_sort.kinematics import fk

WELD_TIMEOUT = 2.0     # s to wait for a plugin to say "attached" or "detached"


def dummy_sdf(name):
    """An invisible, weightless stand-in with an item's name, for the warm-up."""
    return (f"<sdf version='1.10'><model name='{name}'>"
            "<link name='link'><gravity>false</gravity><inertial><mass>0.001</mass>"
            "<inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz></inertia></inertial>"
            "</link></model></sdf>")


class Suction(Node):

    def __init__(self):
        super().__init__('suction')
        self.joints = {}
        self.joint_updates = 0  # how many /joint_states have come in
        self.items = {}        # item name -> (x, y, z) true centre
        self.held = None       # name of the item in the cup
        self.gripping = None   # name of the item being gripped or held
        self.warming_up = True
        self.states = {}       # item name -> 'attached' / 'detached', as the plugin last said
        self.state_changed = threading.Condition()

        self.gz = GzNode()
        self.attach_pubs, self.detach_pubs = {}, {}
        for k in range(POOL):
            name = pool_name(k + 1)
            self.attach_pubs[name] = self.gz.advertise(f'/scara/suction/{name}/attach', Empty)
            self.detach_pubs[name] = self.gz.advertise(f'/scara/suction/{name}/detach', Empty)
            self.gz.subscribe(StringMsg, f'/scara/suction/{name}/state',
                              lambda msg, name=name: self.on_state(name, msg.data))

        # Queues of 1: always work from the latest message, never from a backlog
        group = ReentrantCallbackGroup()  # the service waits while the topics keep coming in
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 1,
                                 callback_group=group)
        self.create_subscription(TFMessage, '/gz/poses', self.on_poses, 1, callback_group=group)

        self.warm_up()
        self.warming_up = False
        self.create_service(SetBool, '/suction', self.on_request, callback_group=group)
        self.get_logger().info('Suction ready: /suction')

    # ----- what the plugins and Gazebo say -----

    def on_state(self, name, state):
        with self.state_changed:
            self.states[name] = state
            self.state_changed.notify_all()
        if state == 'attached' and not self.warming_up and name != self.gripping:
            # A weld nobody asked for now (e.g. an earlier request for an item that had just
            # gone, which the plugin keeps until that name appears again): let go at once
            self.detach_pubs[name].publish(Empty())

    def wait_for_state(self, names, state, timeout=WELD_TIMEOUT):
        """Wait until every one of names is in state. Returns those that are."""
        with self.state_changed:
            self.state_changed.wait_for(
                lambda: all(self.states.get(name) == state for name in names), timeout)
            return [name for name in names if self.states.get(name) == state]

    def on_joint_states(self, msg):
        self.joints.update(zip(msg.name, msg.position))
        self.joint_updates += 1

    def cup_standing_still(self, timeout=1.0):
        """Wait for fresh joint states showing the cup still (the arm may have only just
        arrived, and this node may not have seen that yet). False on timeout."""
        end = time.time() + timeout
        last, seen = None, self.joint_updates
        while time.time() < end:
            if self.joint_updates >= seen + 2:   # two new messages since the last look
                cup = self.cup()
                if last is not None and max(abs(a - b) for a, b in zip(cup, last)) < 0.0001:
                    return True
                last, seen = cup, self.joint_updates
            time.sleep(0.005)
        return False

    def on_poses(self, msg):
        # Each message lists every item in the world: removed ones must not linger
        self.items = {t.child_frame_id: (t.transform.translation.x, t.transform.translation.y,
                                         t.transform.translation.z) for t in msg.transforms}

    # ----- warm-up -----

    def warm_up(self):
        """Let every plugin weld (and release) its name once, so it never does so by itself."""
        world = GzWorld(self)
        if not world.wait():
            raise RuntimeError('no /world/sort_cell/create service: is the simulation running?')
        names = [pool_name(k + 1) for k in range(POOL)]
        old = world.items_present() or []   # items of an earlier run
        for name in old:
            world.remove(name)
        if old:
            time.sleep(1.0)   # let Gazebo take them away before the names are used again
        self.get_logger().info(f'Warm-up: spawning {POOL} dummy items so each suction slot '
                               'welds once...')
        for k, name in enumerate(names):  # off to the side, behind the robot on its right
            world.create(dummy_sdf(name), -0.9, -0.9 + 0.05 * k, 0.05)
        welded = self.wait_for_state(names, 'attached')
        for name in welded:
            self.detach_pubs[name].publish(Empty())
        released = self.wait_for_state(welded, 'detached')
        for name in names:
            world.remove(name)
        if len(released) < len(welded):
            self.get_logger().warn(f'Warm-up: {len(welded) - len(released)} slots did not let go')
        elif len(welded) == POOL:
            self.get_logger().info(f'Warm-up done: all {POOL} slots welded and released once')
        else:  # the simulation was already running: the other slots were warmed up before
            self.get_logger().info(f'Warm-up done: {POOL - len(welded)} slots were warmed up '
                                   f'before, {len(welded)} welded and released now')

    # ----- the service -----

    def cup(self):
        """True position of the cup tip in the world, from the joint angles."""
        x, y, z = fk(self.joints['j1'], self.joints['j2'], self.joints['j3'])
        return x, y, BELT_TOP + z

    def under_cup(self):
        """Name of the item right under the cup, or None."""
        return item_under_cup(self.cup(), self.items)

    def on_request(self, request, response):
        if 'j1' not in self.joints:
            response.success, response.message = False, 'no /joint_states'
        elif request.data:
            response.success, response.message = self.grip()
        else:
            response.success, response.message = self.release()
        return response

    def grip(self):
        if self.held:
            return True, f'already holding {self.held}'
        if not self.cup_standing_still():
            return False, 'the arm is still moving'
        name = self.under_cup()
        if name is None:
            nearest = nearest_item(self.cup(), self.items)
            if nearest and nearest[1] < 100:
                self.get_logger().info('Nothing under the cup: nearest %s, %.1f mm sideways, '
                                       'top %.1f mm below the cup' % nearest)
            return False, 'nothing under the cup'
        self.gripping = name
        self.attach_pubs[name].publish(Empty())
        if not self.wait_for_state([name], 'attached'):
            self.gripping = None
            return False, f'{name} did not weld'
        self.held = name
        return True, f'gripped {name}'

    def release(self):
        if not self.held:
            return True, 'nothing to release'
        name, self.held, self.gripping = self.held, None, None
        self.detach_pubs[name].publish(Empty())
        if not self.wait_for_state([name], 'detached'):
            return False, f'{name} did not let go'
        return True, f'released {name}'


def main():
    rclpy.init()
    node = Suction()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

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

Reads, straight from Gazebo (it is part of the simulated hardware, so it may use the truth):
             /joint_states, where the cup is, and /world/sort_cell/dynamic_pose/info, where
             the items really are. Gazebo delivers these on its own threads, so they keep
             coming in while a /suction call waits, and ROS 2 only has to serve the call.
Service:     /suction  (std_srvs/SetBool)
"""

import threading
import time
from math import hypot

import rclpy
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node as GzNode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool

from scara_sort.cell import BELT_TOP
from scara_sort.grip import item_under_cup, nearest_item
from scara_sort.gz import WORLD, GzWorld
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

        self.gz.subscribe(Model, '/joint_states', self.on_joint_states)
        self.gz.subscribe(Pose_V, f'/world/{WORLD}/dynamic_pose/info', self.on_poses)

        self.warm_up()
        self.warming_up = False
        self.create_service(SetBool, '/suction', self.on_request)
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
        self.joints.update({joint.name: joint.axis1.position for joint in msg.joint})
        self.joint_updates += 1

    def fresh_joint_states(self, timeout=1.0):
        """Wait for two new /joint_states. The arm may have only just arrived (or be riding
        along with an item on the moving belt), and this node may not have seen that yet."""
        end = time.time() + timeout
        seen = self.joint_updates
        while self.joint_updates < seen + 2:
            if time.time() > end:
                return False
            time.sleep(0.002)
        return True

    def on_poses(self, msg):
        # Each message lists every item in the world: removed ones must not linger
        self.items = {p.name: (p.position.x, p.position.y, p.position.z)
                      for p in msg.pose if p.name.startswith('item_')}

    # ----- warm-up -----

    def slot_connected(self, name):
        """True once Gazebo's plugin for this name is up and connected to this node."""
        state_pubs, _ = self.gz.topic_info(f'/scara/suction/{name}/state')
        _, attach_subs = self.gz.topic_info(f'/scara/suction/{name}/attach')
        _, detach_subs = self.gz.topic_info(f'/scara/suction/{name}/detach')
        return bool(state_pubs and attach_subs and detach_subs)

    def warm_up(self):
        """Let every plugin weld (and release) its name once, so it never does so by itself."""
        world = GzWorld(self)
        if not world.wait(timeout=60.0):
            raise RuntimeError('no /world/sort_cell/create service: is the simulation running?')
        names = [pool_name(k + 1) for k in range(POOL)]
        # The arm (with its plugins) may not be in Gazebo yet, and the topics take a moment to
        # connect: until then a weld would go unnoticed, or a release would get lost
        if not all(self.slot_connected(name) for name in names):
            self.get_logger().info('Waiting for the arm and its suction slots in Gazebo...')
            while not all(self.slot_connected(name) for name in names):
                time.sleep(0.2)
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
        for name in names:   # every slot, in case a weld happened but its message was missed
            self.detach_pubs[name].publish(Empty())
        released = self.wait_for_state(welded, 'detached')
        for name in names:
            world.remove(name)
        end = time.time() + 5.0   # gone before anyone may use the names
        while time.time() < end and set(world.items_present() or []) & set(names):
            time.sleep(0.1)
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
        if not self.fresh_joint_states():
            return False, 'no /joint_states'
        name = self.under_cup()
        if name is None:
            nearest = nearest_item(self.cup(), self.items)
            if nearest and nearest[1] < 100:
                self.get_logger().info('Nothing under the cup: nearest %s, %.1f mm sideways '
                                       '(x %+.1f, y %+.1f), top %.1f mm below the cup' % nearest)
            return False, 'nothing under the cup'
        self.gripping = name
        self.attach_pubs[name].publish(Empty())
        if not self.wait_for_state([name], 'attached'):
            self.gripping = None
            return False, f'{name} did not weld'
        self.held = name
        x, y, _ = self.items[name]
        cx, cy, _ = self.cup()
        return True, f'gripped {name}, {hypot(x - cx, y - cy) * 1000:.1f} mm off the cup\'s axis'

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
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

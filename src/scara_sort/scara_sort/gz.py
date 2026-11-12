"""Add and remove Gazebo models from a ROS 2 node.

Gazebo's services /world/sort_cell/create and /world/sort_cell/remove are bridged to ROS 2 by
ros_gz_bridge (scara_sort_sim/config/bridge.yaml), so this is plain ROS service calls.

(Not Gazebo's Python bindings: their service calls time out when the same process also
listens to a Gazebo topic while ROS 2 is running, which the suction node has to do.)

The calls wait for the answer, so make them from a node's main loop, not from inside one of
its callbacks.
"""

import time
from math import cos, sin

import rclpy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from tf2_msgs.msg import TFMessage

WORLD = 'sort_cell'  # world name in scara_sort_sim/worlds/sort_cell.sdf


class GzWorld:

    def __init__(self, node, world=WORLD):
        self.node = node
        self.create_client = node.create_client(SpawnEntity, f'/world/{world}/create')
        self.remove_client = node.create_client(DeleteEntity, f'/world/{world}/remove')

    def wait(self, timeout=10.0):
        """Wait until the bridge offers the services. False if it doesn't (no simulation)."""
        return (self.create_client.wait_for_service(timeout_sec=timeout)
                and self.remove_client.wait_for_service(timeout_sec=timeout))

    def call(self, client, request, timeout=5.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        return future.done() and future.result() is not None and future.result().success

    def create(self, sdf, x, y, z, yaw=0.0):
        """Add a model, given as an SDF string, at (x, y, z) m turned by yaw. True if Gazebo took it.

        The position must be given here: the service always sends one (the origin if not set),
        and Gazebo then ignores the <pose> in the SDF.
        Gazebo adds the model at the start of its next step; a model with the same name must be
        gone by then, or Gazebo refuses it (which only shows in Gazebo's own log).
        """
        request = SpawnEntity.Request()
        request.entity_factory.sdf = sdf
        pose = request.entity_factory.pose
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.z, pose.orientation.w = sin(yaw / 2), cos(yaw / 2)
        return self.call(self.create_client, request)

    def items_present(self, timeout=3.0):
        """Names of the items (item_..) in the world now, from /gz/poses. None if it is silent.

        A simulation-side helper: the spawner and the suction node clear an earlier run's
        items with it. (The detector and the sorter must not use /gz/poses.)
        """
        names = []
        sub = self.node.create_subscription(
            TFMessage, '/gz/poses', lambda msg: names.append([t.child_frame_id for t in msg.transforms]), 10)
        end = time.time() + timeout
        while not names and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return names[0] if names else None

    def remove(self, name):
        """Remove a model by name. It is gone after Gazebo's next step.

        True if Gazebo took the request, which it also does when there is no such model.
        """
        request = DeleteEntity.Request()
        request.entity = Entity(name=name, type=Entity.MODEL)
        return self.call(self.remove_client, request)

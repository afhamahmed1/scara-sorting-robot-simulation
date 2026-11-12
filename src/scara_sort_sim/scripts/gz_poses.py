#!/usr/bin/env python3
"""Copy the true positions of the items from Gazebo to ROS 2, with their names.

Publishes:  /gz/poses  (tf2_msgs/TFMessage) one transform per item, child_frame_id = model name
            (e.g. item_07), in the world frame, about 60 times a second.

ros_gz_bridge can bridge Gazebo's /world/sort_cell/dynamic_pose/info, but it drops the names,
so this node reads that topic itself, with Gazebo's Python bindings.

Ground-truth rule: only the "simulated hardware" (the suction node) and the checkers (referee,
vision_check) may use /gz/poses. The detector and the sorter must not.
"""

import rclpy
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class GzPoses(Node):

    def __init__(self):
        super().__init__('gz_poses')
        world = self.declare_parameter('world', 'sort_cell').value
        self.prefix = self.declare_parameter('prefix', 'item_').value  # which models to copy
        self.pub = self.create_publisher(TFMessage, '/gz/poses', 10)
        self.gz = GzNode()
        # Model poses of top-level models are world poses (link poses would be relative)
        self.gz.subscribe(Pose_V, f'/world/{world}/dynamic_pose/info', self.on_poses)

    def on_poses(self, msg):
        out = TFMessage()
        for pose in msg.pose:
            if not pose.name.startswith(self.prefix):
                continue
            t = TransformStamped()
            t.header.stamp.sec, t.header.stamp.nanosec = msg.header.stamp.sec, msg.header.stamp.nsec
            t.header.frame_id = 'world'
            t.child_frame_id = pose.name
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation.x = pose.orientation.x
            t.transform.rotation.y = pose.orientation.y
            t.transform.rotation.z = pose.orientation.z
            t.transform.rotation.w = pose.orientation.w
            out.transforms.append(t)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = GzPoses()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

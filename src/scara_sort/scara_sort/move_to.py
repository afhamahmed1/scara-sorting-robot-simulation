"""Move the cup tip to a point, then exit.

    ros2 run scara_sort move_to X Y Z      (millimetres)

X, Y: position seen from the robot's base (x forward, y to the left).
Z:    height of the cup tip above the belt top.

Publishes:   /scara/j1_cmd, /scara/j2_cmd, /scara/j3_cmd  (std_msgs/Float64) to the joint motors
Subscribes:  /joint_states  (sensor_msgs/JointState) to see when the arm has arrived
"""

import sys
import time
from math import degrees, dist

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from scara_sort.kinematics import fk, ik

JOINTS = ['j1', 'j2', 'j3']
TOLERANCE = 0.0001  # arrived when every joint is this close to its target (rad or m)


class MoveTo(Node):
    """A node that moves the arm. pick_place and the sorter build on it too."""

    def __init__(self, name='move_to', **kwargs):
        super().__init__(name, **kwargs)
        self.joints = {}  # latest joint positions, e.g. {'j1': 0.5, ...}
        self.cmd_pubs = [self.create_publisher(Float64, f'/scara/{joint}_cmd', 10) for joint in JOINTS]
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)

    def on_joint_states(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def arrived(self, target):
        return all(abs(self.joints.get(name, float('inf')) - value) < TOLERANCE
                   for name, value in zip(JOINTS, target))

    def send(self, target):
        """Send each joint motor its target (j1, j2, j3), without waiting."""
        for pub, value in zip(self.cmd_pubs, target):
            pub.publish(Float64(data=value))

    def move(self, target, timeout=20.0, stall=None):
        """Send joint targets (j1, j2, j3) and wait until the arm is there. False on timeout.

        stall: if given (seconds), also give up (False) once the arm has stopped moving for
        that long without getting there, e.g. because what it carries sits on something.
        """
        end = time.time() + timeout

        # 1. Wait until the bridge listens to our topics, otherwise the commands are lost
        while any(pub.get_subscription_count() == 0 for pub in self.cmd_pubs):
            if time.time() > end:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        # 2. Send each joint motor its target
        self.send(target)

        # 3. Wait until /joint_states shows every joint at its target
        last, last_change = None, self.get_clock().now()
        while not self.arrived(target):
            if time.time() > end:
                return False
            rclpy.spin_once(self, timeout_sec=0.02)
            if stall is not None and self.joints:
                now = [self.joints.get(name, 0.0) for name in JOINTS]
                if last is None or max(abs(a - b) for a, b in zip(now, last)) > 1e-5:
                    last, last_change = now, self.get_clock().now()
                elif (self.get_clock().now() - last_change).nanoseconds * 1e-9 > stall:
                    return False
        return True


def main():
    args = remove_ros_args(sys.argv)[1:]
    if len(args) != 3:
        print('Usage: ros2 run scara_sort move_to X Y Z   (millimetres, Z = height above the belt)')
        sys.exit(1)
    point = [float(a) / 1000 for a in args]  # mm -> m

    try:
        target = ik(*point)
    except ValueError as error:
        print(f'Refused: {error}')
        sys.exit(1)

    j1, j2, j3 = target
    print(f'Moving: j1 {degrees(j1):.1f}°, j2 {degrees(j2):.1f}°, j3 {j3 * 1000:.1f} mm')

    rclpy.init()
    node = MoveTo()
    if node.move(target):
        tip = fk(*(node.joints[name] for name in JOINTS))
        print('Arrived: tip at (%.1f, %.1f, %.1f) mm, %.2f mm from the target'
              % (tip[0] * 1000, tip[1] * 1000, tip[2] * 1000, dist(tip, point) * 1000))
    else:
        print('Timed out: is the simulation running?')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

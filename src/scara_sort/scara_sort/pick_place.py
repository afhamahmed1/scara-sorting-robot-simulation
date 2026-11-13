"""Pick one item off the stopped belt and drop it into a bin.

    ros2 run scara_sort pick_place X Y BIN      (X, Y in mm; BIN = 1, 2, 3 or reject)

1. go to the ready pose, above the pick line, 60 mm over the item tops
2. move above the item, lower the cup to 2 mm over its top, suction on
   (nothing gripped: lift, report it and stop)
3. lift fully, move over the bin, lower until the item is 10 mm above the bin walls (or
   until it sits on the heap in the bin, if that is higher), suction off: it drops in
4. back to the ready pose

The sorter uses the same moves (the Arm class).

Publishes:   /scara/j1_cmd, /scara/j2_cmd, /scara/j3_cmd  (std_msgs/Float64) to the joint motors
Subscribes:  /joint_states  (sensor_msgs/JointState)
Uses:        /suction  (std_srvs/SetBool)
"""

import sys

import rclpy
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from std_srvs.srv import SetBool

from scara_sort.cell import BELT_TOP, BIN_WALL_TOP, BINS, ITEM_SIZE, PICK_LINE
from scara_sort.intercept import J3_SPEED
from scara_sort.kinematics import TIP_TOP, ik
from scara_sort.move_to import MoveTo

# Cup tip heights above the belt (m)
GRIP_GAP = 0.002                                   # cup to item top when gripping
READY_HEIGHT = ITEM_SIZE + 0.060                   # 60 mm over the item tops
PICK_HEIGHT = ITEM_SIZE + GRIP_GAP                 # 2 mm over an item's top
TRAVEL_HEIGHT = TIP_TOP                            # cup fully up: a carried item clears everything
PLACE_HEIGHT = (BIN_WALL_TOP - BELT_TOP) + 0.010 + ITEM_SIZE + GRIP_GAP  # item 10 mm over the walls
READY = (0.40, PICK_LINE)                          # above the middle of the belt at the pick line


class Arm(MoveTo):
    """MoveTo plus the cup: tip_to() in metres, suction(), and the pick and place moves."""

    def __init__(self, name='pick_place', **kwargs):
        # Simulated time, so that durations don't depend on how fast the computer is
        kwargs.setdefault('parameter_overrides', [Parameter('use_sim_time', value=True)])
        super().__init__(name, **kwargs)
        self.suction_client = self.create_client(SetBool, '/suction')

    def tip_to(self, x, y, z):
        """Move the cup tip to (x, y, z) m, z above the belt, and wait until it is there."""
        if not self.move(ik(x, y, z)):
            raise TimeoutError('the arm did not get there (is the simulation running?)')

    def suction(self, on, timeout=5.0):
        """Switch the suction on or off. Returns (success, message) from the suction node."""
        if not self.suction_client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError('no /suction service (is the suction node running?)')
        future = self.suction_client.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError('/suction did not answer')
        return future.result().success, future.result().message

    def seconds(self):
        """Simulated time now, in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def ready(self):
        self.tip_to(*READY, READY_HEIGHT)

    def pick(self, x, y):
        """Grip the item at (x, y) on the stopped belt. Returns (gripped, message)."""
        self.tip_to(x, y, READY_HEIGHT)
        self.tip_to(x, y, PICK_HEIGHT)
        gripped, message = self.suction(True)
        self.tip_to(x, y, TRAVEL_HEIGHT)  # lift fully, with or without the item
        return gripped, message

    def place(self, bin_name):
        """Carry the gripped item over a bin and drop it in. Returns the suction's message."""
        x, y = BINS[bin_name]
        self.tip_to(x, y, TRAVEL_HEIGHT)
        # Going down, the item may come to sit on the heap in the bin before it gets there
        # (the bins are small). So the quill gets the time the way down takes, plus a little,
        # and the item is let go where it is then.
        target = ik(x, y, PLACE_HEIGHT)
        self.send(target)
        end = self.seconds() + abs(target[2] - self.joints['j3']) / J3_SPEED + 0.15
        while not self.arrived(target) and self.seconds() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
        _, message = self.suction(False)
        return message


def main():
    args = remove_ros_args(sys.argv)[1:]
    if len(args) != 3 or args[2] not in BINS:
        print('Usage: ros2 run scara_sort pick_place X Y BIN   (mm; BIN = 1, 2, 3 or reject)')
        sys.exit(1)
    x, y, bin_name = float(args[0]) / 1000, float(args[1]) / 1000, args[2]
    try:
        ik(x, y, PICK_HEIGHT)
    except ValueError as error:
        print(f'Refused: {error}')
        sys.exit(1)

    # Ctrl-C raises KeyboardInterrupt but leaves ROS running, so the suction can still be released
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    arm = Arm()
    ok = False
    try:
        arm.ready()
        start = arm.seconds()
        print(f'Picking at ({x * 1000:.0f}, {y * 1000:.0f}) mm → bin {bin_name}')
        gripped, message = arm.pick(x, y)
        if not gripped:
            print(f'Missed: {message}')
        else:
            print(message.capitalize())
            arm.place(bin_name)
            print(f'Placed in bin {bin_name} ({arm.seconds() - start:.1f} s)')
            ok = True
        arm.ready()
    except TimeoutError as error:
        print(f'Stopped: {error}')
    except KeyboardInterrupt:
        print('Stopped with Ctrl-C: suction off')
        arm.suction(False)
    arm.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

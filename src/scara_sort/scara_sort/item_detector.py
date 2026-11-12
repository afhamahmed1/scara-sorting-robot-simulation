"""Find the items in the camera image, follow them, and publish where they are now.

    ros2 run scara_sort item_detector

Every camera frame: find the items (vision.py), read the belt's encoder at the moment the
frame was taken, and add them to the tracker (tracker.py). Ten times a second: publish every
confirmed item, at the place the encoder says it has reached by now, also long after it has
left the camera's view.

Uses only the camera and the encoder, as on a real belt: never Gazebo's true positions.

Subscribes:  /camera/image, /camera/camera_info  (sensor_msgs)
             /conveyor/odom  (nav_msgs/Odometry), the belt's encoder
Publishes:   /items  (scara_sort_msgs/ItemArray) confirmed items, 10 Hz
             /camera/detections  (sensor_msgs/Image) the camera image with outlines and labels
"""

from bisect import bisect_left
from collections import deque

import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image

from scara_sort.config import load_yaml
from scara_sort.tracker import Tracker
from scara_sort.vision import Camera, detect, draw
from scara_sort_msgs.msg import Item, ItemArray


def nanoseconds(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


class Encoder:
    """The belt's recent encoder readings, to look up the reading at any moment."""

    def __init__(self):
        self.times = deque(maxlen=500)      # ns, sim time (10 s at 50 Hz)
        self.positions = deque(maxlen=500)  # m of belt travel
        self.speed = 0.0                    # m/s, latest

    def add(self, msg):
        self.times.append(nanoseconds(msg.header.stamp))
        self.positions.append(msg.pose.pose.position.x)
        self.speed = msg.twist.twist.linear.x

    def at(self, t):
        """Encoder reading at time t (ns): interpolated, or extrapolated past the latest."""
        if not self.times:
            return None
        if t >= self.times[-1]:
            return self.positions[-1] + self.speed * (t - self.times[-1]) * 1e-9
        k = bisect_left(self.times, t)
        if k == 0:
            return self.positions[0]
        t0, t1 = self.times[k - 1], self.times[k]
        p0, p1 = self.positions[k - 1], self.positions[k]
        return p0 + (p1 - p0) * (t - t0) / (t1 - t0)


class ItemDetector(Node):

    def __init__(self):
        super().__init__('item_detector',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.ranges = load_yaml(self.declare_parameter('colours', 'config/colours.yaml').value)
        self.camera = None
        self.encoder = Encoder()
        self.tracker = Tracker()
        self.announced = set()  # ids of the items already logged as confirmed
        self.bridge = CvBridge()

        self.items_pub = self.create_publisher(ItemArray, '/items', 10)
        self.image_pub = self.create_publisher(Image, '/camera/detections', 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_camera_info, 10)
        self.create_subscription(Odometry, '/conveyor/odom', self.encoder.add, 50)
        self.create_subscription(Image, '/camera/image', self.on_image, 10)
        self.create_timer(0.1, self.publish_items)
        self.get_logger().info('Waiting for the camera and the belt encoder...')

    def on_camera_info(self, msg):
        if self.camera is None:
            k = msg.k  # 3x3 camera matrix, row by row
            self.camera = Camera(fx=k[0], fy=k[4], cx=k[2], cy=k[5])
            self.get_logger().info(f'Camera: fx {k[0]:.1f}, fy {k[4]:.1f} px, '
                                   f'centre ({k[2]:.1f}, {k[5]:.1f})')

    def on_image(self, msg):
        belt = self.encoder.at(nanoseconds(msg.header.stamp))
        if self.camera is None or belt is None:
            return
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        detections = detect(image, self.ranges, self.camera)
        ids = self.tracker.update([(d.colour, d.shape, d.x, d.y) for d in detections], belt)

        labels = []
        tracks = {t.id: t for t in self.tracker.tracks}
        for item_id in ids:
            track = tracks[item_id]
            labels.append(f'#{item_id} {track.colour} {track.shape}' if track.confirmed
                          else f'#{item_id} ?')
            if track.confirmed and item_id not in self.announced:
                self.announced.add(item_id)
                self.get_logger().info(f'#{item_id} {track.colour} {track.shape} '
                                       f'at x {track.x * 1000:.0f} mm')
        out = self.bridge.cv2_to_imgmsg(draw(image, detections, labels), 'bgr8')
        out.header = msg.header
        self.image_pub.publish(out)

    def publish_items(self):
        now = self.get_clock().now()
        belt = self.encoder.at(now.nanoseconds)
        if belt is None:
            return
        msg = ItemArray()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'world'
        for track in self.tracker.items(belt):
            msg.items.append(Item(id=track.id, colour=track.colour, shape=track.shape,
                                  x=track.x, y=track.y(belt), times_seen=track.times_seen))
        self.items_pub.publish(msg)


def main():
    rclpy.init()
    node = ItemDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

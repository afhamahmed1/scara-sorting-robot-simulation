"""Find the items in a camera image: colour, shape and position on the belt.

Plain functions, no ROS: tested on synthetic images drawn with OpenCV (test/test_vision.py).

1. colour    one HSV mask per colour (ranges from config/colours.yaml), cleaned, then blobs.
             The ranges only take the brightly lit top faces: the side faces, which the camera
             sees for items off its centre line, are much darker. Blobs touching the image edge
             are left until the item is fully in view.
2. shape     round outline (circularity > 0.85) = cylinder; else simplify the outline
             (approxPolyDP, 4 % of its length): 3 corners = prism, 4 = cube. A blob whose area
             doesn't fit that shape (two touching items, a bit of an item) is unknown.
3. position  every item is 40 mm tall, so all the top faces lie in one plane, 0.61 m under
             the camera: a pixel (u, v) maps to a point on that plane.
"""

from dataclasses import dataclass, field
from math import pi

import cv2
import numpy as np

from scara_sort.cell import BELT_X, CAMERA, ITEM_TOP

CAMERA_HEIGHT = CAMERA[2] - ITEM_TOP   # 0.61 m from the camera down to the item tops

# Top-face areas of the shapes, m²
SHAPE_AREAS = {
    'cube': 0.04 * 0.04,
    'cylinder': pi * 0.02 ** 2,
    'prism': 0.04 * 0.04 * 3 ** 0.5 / 4,
}
AREA_TOLERANCE = 0.30      # a blob within ±30 % of its shape's area is that shape
ROUND = 0.85               # circularity 4π·area/perimeter²: circle 1.0, square 0.79
EDGE = 2                   # px: a blob this close to the image edge is not fully in view


@dataclass
class Camera:
    """Pinhole camera looking straight down: focal lengths and centre in pixels."""
    fx: float
    fy: float
    cx: float
    cy: float

    def to_belt(self, u, v):
        """Pixel (u, v) -> point (x, y) in robot coordinates on the plane of the item tops.

        The camera is turned so that image right (+u) is +y, the way the belt runs, and image
        down (+v) is +x, away from the robot (see the camera's pose in sort_cell.sdf).
        """
        x = CAMERA[0] + (v - self.cy) * CAMERA_HEIGHT / self.fy
        y = CAMERA[1] + (u - self.cx) * CAMERA_HEIGHT / self.fx
        return x, y

    def to_pixel(self, x, y):
        """Point (x, y) on the plane of the item tops -> pixel (u, v). The inverse of to_belt."""
        u = self.cx + (y - CAMERA[1]) * self.fx / CAMERA_HEIGHT
        v = self.cy + (x - CAMERA[0]) * self.fy / CAMERA_HEIGHT
        return u, v

    def area_in_pixels(self, area):
        """How many pixels an area (m²) on the item-top plane covers."""
        return area * self.fx * self.fy / CAMERA_HEIGHT ** 2


@dataclass
class Detection:
    colour: str
    shape: str
    x: float            # m, robot frame, on the item-top plane
    y: float
    u: float            # px, centre of the blob
    v: float
    contour: np.ndarray = field(repr=False)


def colour_masks(bgr, ranges):
    """One cleaned black/white mask per colour.

    ranges: {colour: [[h, s, v], [h, s, v]], ...} lower and upper bounds (OpenCV: hue 0-180),
    or a list of such pairs for a colour whose hue wraps around 0 (red).
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    kernel = np.ones((3, 3), np.uint8)
    masks = {}
    for colour, bounds in ranges.items():
        pairs = bounds if isinstance(bounds[0][0], (list, tuple)) else [bounds]
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lower, upper in pairs:
            mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
        masks[colour] = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # remove specks
    return masks


def classify(contour, camera):
    """Shape of a blob from its outline: cube, cylinder, prism or unknown."""
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return 'unknown'
    circularity = 4 * pi * area / perimeter ** 2
    if circularity > ROUND:
        shape = 'cylinder'
    else:
        corners = len(cv2.approxPolyDP(contour, 0.04 * perimeter, True))
        shape = {3: 'prism', 4: 'cube'}.get(corners, 'unknown')
    if shape != 'unknown':
        expected = camera.area_in_pixels(SHAPE_AREAS[shape])
        if abs(area - expected) > AREA_TOLERANCE * expected:
            shape = 'unknown'   # e.g. two items touching, or a piece of one
    return shape


def detect(bgr, ranges, camera):
    """All items fully in view on the belt: a list of Detection."""
    height, width = bgr.shape[:2]
    smallest = camera.area_in_pixels(min(SHAPE_AREAS.values())) * 0.3
    found = []
    for colour, mask in colour_masks(bgr, ranges).items():
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            if cv2.contourArea(contour) < smallest:
                continue  # a speck, not an item
            bx, by, bw, bh = cv2.boundingRect(contour)
            if bx < EDGE or by < EDGE or bx + bw > width - EDGE or by + bh > height - EDGE:
                continue  # not fully in view yet
            m = cv2.moments(contour)
            u, v = m['m10'] / m['m00'], m['m01'] / m['m00']
            x, y = camera.to_belt(u, v)
            if not BELT_X[0] <= x <= BELT_X[1]:
                continue  # not on the belt
            found.append(Detection(colour, classify(contour, camera), x, y, u, v, contour))
    return found


def draw(bgr, detections, labels):
    """A copy of the image with each detection's outline and its label (e.g. '#4 red cube')."""
    out = bgr.copy()
    for detection, label in zip(detections, labels):
        cv2.drawContours(out, [detection.contour], -1, (255, 255, 255), 2)
        org = (int(detection.u) - 40, int(detection.v) - 30)
        cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                    cv2.LINE_AA)
    return out

"""Follow each item from frame to frame, and predict where it is from the belt's encoder.

Plain Python, no ROS: tested in test/test_tracker.py.

An item lying on the belt moves exactly as far as the belt does. So for every sighting the
tracker stores where the item sits on the belt, s = y_seen - belt_seen, which stays the same
while it rides along. Wherever the belt is now, the item is at

    y = s + belt_now        (= y_seen + (belt_now - belt_seen))

whether the belt runs, stops or changes speed, and also long after the item has left the
camera's view. s and x are averaged over all sightings, which smooths out the pixel noise.
"""

from collections import Counter
from dataclasses import dataclass, field
from math import hypot

MATCH_DISTANCE = 0.015   # m: a detection this close to an item's predicted position is that item
CONFIRM_FRAMES = 3       # an item is confirmed once it has been seen in this many frames
LOST_AFTER = 0.03        # m of belt travel: an unconfirmed item not seen again is dropped
BELT_END = 0.75          # m: items predicted past here have left the belt


@dataclass
class Track:
    id: int
    x: float                  # m, mean of the x seen
    s: float                  # m, mean of (y seen - belt reading): its place on the belt
    times_seen: int = 1
    last_seen: float = 0.0    # belt reading of the last sighting
    colours: Counter = field(default_factory=Counter)
    shapes: Counter = field(default_factory=Counter)

    @property
    def colour(self):
        return self.colours.most_common(1)[0][0]

    @property
    def shape(self):
        return self.shapes.most_common(1)[0][0]

    @property
    def confirmed(self):
        return self.times_seen >= CONFIRM_FRAMES

    def y(self, belt):
        """Where the item is along the belt when the encoder reads `belt`."""
        return self.s + belt

    def add_sighting(self, x, y, belt, colour, shape):
        n = self.times_seen
        self.x = (self.x * n + x) / (n + 1)
        self.s = (self.s * n + (y - belt)) / (n + 1)
        self.times_seen = n + 1
        self.last_seen = belt
        self.colours[colour] += 1
        self.shapes[shape] += 1


class Tracker:

    def __init__(self):
        self.tracks = []
        self.next_id = 1

    def update(self, detections, belt):
        """Add one camera frame. detections: [(colour, shape, x, y)] seen at encoder reading belt.

        Returns the id given to each detection, in the same order.
        """
        ids = []
        taken = set()
        for colour, shape, x, y in detections:
            # The nearest item of the same colour, if it is close enough to where we expect it
            best, best_distance = None, MATCH_DISTANCE
            for track in self.tracks:
                if track.id in taken or track.colour != colour:
                    continue
                distance = hypot(x - track.x, y - track.y(belt))
                if distance <= best_distance:
                    best, best_distance = track, distance
            if best is None:
                best = Track(self.next_id, x, y - belt, last_seen=belt)
                best.colours[colour] += 1
                best.shapes[shape] += 1
                self.tracks.append(best)
                self.next_id += 1
            else:
                best.add_sighting(x, y, belt, colour, shape)
            taken.add(best.id)
            ids.append(best.id)
        self.prune(belt)
        return ids

    def prune(self, belt):
        """Drop items that have left the belt, and unconfirmed ones that were never seen again."""
        self.tracks = [t for t in self.tracks
                       if t.y(belt) <= BELT_END
                       and (t.confirmed or belt - t.last_seen <= LOST_AFTER)]

    def items(self, belt):
        """The confirmed items, with where they are when the encoder reads belt."""
        self.prune(belt)
        return [t for t in self.tracks if t.confirmed]

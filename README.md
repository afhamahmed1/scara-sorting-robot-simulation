# SCARA Sorter

A SCARA robot arm with a suction cup sorts items coming down a conveyor belt, simulated in
Gazebo. The full design is in [PLAN.md](PLAN.md). It builds on the SCARA Driller: same arm,
same kinematics, same joint motors.

**Stages 1 to 3 of 6 are done:** the cell (the arm with its suction cup, no suction yet,
the conveyor belt with its encoder, four bins, the catch box, the camera on its pole) with
`move_to` to put the cup anywhere; the item spawner, which sends a seeded stream of coloured
cubes, cylinders and prisms down the belt; and the item detector, which names each item from
the camera image and tracks it along the belt with the encoder.

## 1. Build

The repo is the workspace:

```bash
git clone https://github.com/afhamahmed1/scara-sorting-robot-simulation.git ~/Robotics/Projects/scara_sort_ws
cd ~/Robotics/Projects/scara_sort_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

Run `colcon build` again after every change.

## 2. Run

```bash
source ~/Robotics/Projects/scara_sort_ws/install/setup.bash
ros2 launch scara_sort_sim sim.launch.py
```

Gazebo shows the orange arm on its grey column, a dark belt running along x = 0.40 m with
low rails on both sides, four blue bins on the robot's left and behind it, a brown catch box
at the end of the belt, and the camera on its pole above the start of the belt. RViz shows
the arm.

`gui:=false` runs Gazebo without its window and `rviz:=false` leaves out RViz.

## 3. Run the belt

In a second terminal (run `source ~/Robotics/Projects/scara_sort_ws/install/setup.bash` first):

```bash
ros2 topic pub --once /conveyor/cmd_vel std_msgs/msg/Float64 "{data: 0.05}"   # 50 mm/s
```

Whatever lies on the belt now rides along it (+y) and drops off the end into the catch box.
Stop the belt with `"{data: 0.0}"`.

The belt's encoder counts how far the belt has moved, in metres:

```bash
ros2 topic echo /conveyor/odom --field pose.pose.position.x   # counts up 0.05 per second
ros2 topic echo /conveyor/odom --field twist.twist.linear.x   # 0.05 while it runs
```

The belt is Gazebo's `TrackController` on a static box: it drags whatever lies on it at the
commanded speed, starting and stopping at 0.5 m/s² so items don't slide.

## 4. Move the cup tip to a point

```bash
ros2 run scara_sort move_to 400 0 100
```

X and Y are measured from the robot's base (x forward, y to the left), Z is the height of the
cup tip above the **belt**, all in millimetres. This is the ready pose, 60 mm above the tops
of the items (they are 40 mm tall):

```
Moving: j1 -38.6°, j2 87.1°, j3 30.0 mm
Arrived: tip at (400.0, -0.0, 100.0) mm, 0.00 mm from the target
```

Check it with TF (z is the belt top at 0.20 m plus 0.10 m):

```bash
ros2 run tf2_ros tf2_echo world cup_tip   # Translation: [0.400, -0.000, 0.300]
```

Every bin centre (80 mm above the belt is 10 mm over the bin walls with an item hanging under
the cup) and both ends of the pick zone (42 mm is 2 mm over an item's top):

```bash
ros2 run scara_sort move_to 120 400 80     # bin 1:      j1 37.0°,  j2 81.6°
ros2 run scara_sort move_to -120 420 80    # bin 2:      j1 72.3°,  j2 75.2°
ros2 run scara_sort move_to -340 240 80    # bin 3:      j1 108.3°, j2 82.1°
ros2 run scara_sort move_to -380 60 80     # reject bin: j1 130.5°, j2 91.7°
ros2 run scara_sort move_to 400 -150 42    # start of the pick zone
ros2 run scara_sort move_to 400 150 42     # end of the pick zone
```

Each one ends with `0.00 mm from the target`.

The kinematics are the driller's (see its README): the arm reaches 172 to 550 mm from the
base, and the cup tip goes from 20 mm below to 130 mm above the belt. The motors are twice
as fast: j1 and j2 at 2 rad/s, j3 at 0.2 m/s. Points out of reach are refused:

```bash
ros2 run scara_sort move_to 600 0 50   # Refused: 600 mm from the base, but the arm reaches 172 to 550 mm
```

## 5. Items on the belt

With the simulation running, start the spawner and the belt (two more terminals):

```bash
ros2 run scara_sort item_spawner
ros2 topic pub --once /conveyor/cmd_vel std_msgs/msg/Float64 "{data: 0.05}"
```

```
[item_spawner]: 30 items, seed 1, one every 250 ± 50 mm of belt travel
[item_spawner]: #1 item_00: green prism at x 421 mm
[item_spawner]: #2 item_01: yellow cylinder at x 398 mm
[item_spawner]: #3 item_02: yellow cube at x 424 mm
...
[item_spawner]: #30 item_05: blue cylinder at x 391 mm
[item_spawner]: All items spawned
```

Items appear at the start of the belt, every 250 ± 50 mm of **belt travel** (read from the
encoder, not the clock: stop the belt and no new items come), up to 30 mm left or right of
the centre and turned by a random angle. They ride the length of the belt and drop into the
catch box. The same seed always gives the same items in the same order. The settings are in
`src/scara_sort/config/items.yaml`; `count`, `seed`, `spacing` and `lateral` can also be
given on the command line:

```bash
ros2 run scara_sort item_spawner --ros-args -p count:=10 -p seed:=2
```

Every item is 40 mm tall and weighs 50 g: a 40 mm cube, a Ø40 mm cylinder standing up, or a
triangular prism with 40 mm sides, in red, green, blue or yellow. The prism's collision shape
is three boxes that fill its triangle (it still looks exact): Gazebo would make a triangle
mesh of it, and a pile of meshes in a bin slows the simulation down several times.

Item names come from a pool, `item_00` … `item_23`, because the suction cup will need every
name in advance (stage 4). Before a name is used again, the oldest item, which had it, is
removed, so each bin keeps its most recent items.

The truth about each item, for the referee and checks only, comes on two topics:

```bash
ros2 topic echo /items/spawned   # number: 1, name: item_00, colour: green, shape: prism, ...
ros2 topic echo /gz/poses        # child_frame_id: item_00, transform: translation: ...
```

`/gz/poses` comes from `gz_poses`, a small node started by `sim.launch.py`: `ros_gz_bridge`
can bridge Gazebo's poses, but it drops the model names.

The simulation slows down as items pile up in the catch box (on a 4-core machine without a
GPU: from real time to about half speed with 20 items in a heap). Everything runs on simulated time, so the results
don't change, only the waiting.

## 6. Identifying the items

With the simulation running and items riding along (section 5), start the detector:

```bash
ros2 run scara_sort item_detector
```

```
[item_detector]: Camera: fx 879.2, fy 879.2 px, centre (320.0, 240.0)
[item_detector]: #1 green prism at x 421 mm
[item_detector]: #2 yellow cylinder at x 398 mm
...
```

In RViz, the **Detections** image shows what the camera sees, with each item outlined and
labelled, e.g. `#4 red cube` (`#4 ?` until it has been seen 3 times). Every confirmed item,
with where it is now, is on `/items` ten times a second:

```bash
ros2 topic echo /items   # each item: id, colour, shape, x and y (m, where it is now), times_seen
```

How it works (`vision.py` and `tracker.py`, plain Python):

1. **Colour:** one HSV mask per colour (`config/colours.yaml`), cleaned, then the blobs. Only
   the brightly lit top faces match: the camera also sees the darker side faces of items off
   its centre line, and they would pull the blob off centre. Blobs touching the image edge
   wait until the item is fully in view, and blobs off the belt don't count.
2. **Shape:** a round outline (circularity 4π·area/perimeter² > 0.85) is a cylinder; otherwise
   the outline is simplified (`approxPolyDP`, 4 % of its length): 3 corners = prism, 4 = cube.
   A blob much bigger or smaller than its shape (two touching items) is `unknown`.
3. **Position:** all items are 40 mm tall, so all top faces lie in one plane, 0.61 m under the
   camera: pixel (u, v) → (0.40 + (v − cy)·0.61/fy, −0.45 + (u − cx)·0.61/fx) m.
4. **Tracking:** each sighting is stored as where the item sits on the belt,
   s = y − encoder reading (read at the moment the frame was taken). Wherever the belt is now,
   the item is at y = s + encoder now: whether the belt runs, stops or changes speed, and long
   after it has left the camera's view. A sighting within 15 mm of an item's predicted place,
   in the same colour, is that item; otherwise it is a new one. An item is confirmed after 3
   sightings, and its colour and shape are the most common answers.

The detector uses only the camera and the encoder, never Gazebo's true positions.
`vision_check` compares it with the truth:

```bash
ros2 run scara_sort vision_check
```

For each item, once it has passed the pick line: what the detector called it, its largest
position error while under the camera, and the error at the pick line, predicted from the
encoder about 5 s after the camera last saw it. For the seeded 30-item run (seed 1):

```
[vision_check]: #1 green prism = item_00 (#1 green prism): 0.8 mm under the camera, 0.6 mm at the pick line
[vision_check]: #2 yellow cylinder = item_01 (#2 yellow cylinder): 0.7 mm under the camera, 0.4 mm at the pick line
...
[vision_check]: #30 blue cylinder = item_05 (#30 blue cylinder): 0.6 mm under the camera, 0.5 mm at the pick line
[vision_check]: 30 items: 30 named right, 0 wrong; worst error 0.9 mm under the camera, 0.7 mm at the pick line
```

## 7. Unit tests

The kinematics, the cell layout, the items, the vision (on synthetic images drawn with
OpenCV) and the tracker are tested without Gazebo:

```bash
colcon test --packages-select scara_sort --event-handlers console_direct+
```

It should end with `82 passed`. `test_cell.py` checks that every bin centre and corner and the
whole pick zone are reachable. The hardest point is the reject bin's far corner, at
(−0.44, 0.00) m: the shoulder turns 147° there, inside its ±150° range.

## Files

| File | What it does |
|---|---|
| `src/scara_sort_sim/urdf/scara.urdf.xacro` | The arm: column, two links, quill with vacuum tube and suction cup, `cup_tip` frame, joint motors |
| `src/scara_sort_sim/worlds/sort_cell.sdf` | The cell: belt (with `TrackController`), frame and rails, bins, catch box, camera on its pole |
| `src/scara_sort_sim/models/sort_bin/` | One bin, used four times by the world |
| `src/scara_sort_sim/config/bridge.yaml` | Topics and services copied between Gazebo and ROS 2 |
| `src/scara_sort_sim/scripts/gz_poses.py` | `gz_poses` node: the items' true positions, with their names, on `/gz/poses` |
| `src/scara_sort_sim/launch/sim.launch.py` | Starts Gazebo, the arm, the bridge, `gz_poses` and RViz |
| `src/scara_sort_sim/rviz/sort.rviz` | RViz layout |
| `src/scara_sort_msgs/msg/` | `Item`, `ItemArray`, `SpawnedItem` messages |
| `src/scara_sort/scara_sort/kinematics.py` | IK and FK: joint angles ↔ cup tip position (the driller's) |
| `src/scara_sort/scara_sort/cell.py` | Where everything in the cell is: belt, camera, pick zone, bins, catch box |
| `src/scara_sort/scara_sort/move_to.py` | `move_to` node: point in mm → IK → joint motors, waits until the arm arrives |
| `src/scara_sort/scara_sort/items.py` | The items: shapes, colours, their Gazebo models, the seeded sequence |
| `src/scara_sort/scara_sort/item_spawner.py` | `item_spawner` node: puts the items on the belt by belt travel |
| `src/scara_sort/scara_sort/gz.py` | Adds and removes Gazebo models through the bridged services |
| `src/scara_sort/scara_sort/vision.py` | Finds the items in an image: colour masks, shape from the outline, pixel → belt position |
| `src/scara_sort/scara_sort/tracker.py` | Follows the items from frame to frame and predicts where they are from the encoder |
| `src/scara_sort/scara_sort/item_detector.py` | `item_detector` node: camera + encoder → `/items` and `/camera/detections` |
| `src/scara_sort/scara_sort/vision_check.py` | `vision_check` node: compares `/items` with the truth |
| `src/scara_sort/config/items.yaml` | Item settings: colours, shapes, spacing, count, seed |
| `src/scara_sort/config/colours.yaml` | HSV ranges of the colours |
| `src/scara_sort/test/` | Unit tests |
| `PLAN.md` | The design and the six stages |

## Stages

| Stage | What it adds | Status |
|---|---|---|
| 1 cell | The arm with the cup, belt, bins, catch box, camera, `move_to` | done |
| 2 items | `item_spawner`: 30 mixed items ride down the belt | done |
| 3 vision | Camera → item colour, shape and position; tracking with the belt encoder | done |
| 4 suction | Suction cup, `pick_place` | |
| 5 sort | Stop-and-go sorter, referee | |
| 6 on the move | Picking from the running belt | |

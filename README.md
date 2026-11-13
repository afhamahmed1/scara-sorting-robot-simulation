# SCARA Sorter

A SCARA robot arm with a suction cup sorts items coming down a conveyor belt, simulated in
Gazebo. The full design is in [PLAN.md](PLAN.md). It builds on the SCARA Driller: same arm,
same kinematics, same joint motors.

**All six stages are done.** The cell (the arm with its suction cup, the conveyor belt with
its encoder, four bins, the catch box, the camera on its pole); the item spawner, which sends
a seeded stream of coloured cubes, cylinders and prisms down the belt; the item detector,
which names each item from the camera image and tracks it along the belt with the encoder;
the suction cup; the sorter, which puts each item into its bin, either stopping the belt for
each pick or picking from the running belt; and a referee that scores every item from
Gazebo's true positions.

Quick start, everything at once (after building, section 1):

```bash
ros2 launch scara_sort sort.launch.py
```

```
[referee]: 30 items: 30 correct, 0 wrong bin, 0 missed, 0 dropped — 8.4 items/min
```

and picking from the running belt:

```bash
ros2 launch scara_sort sort.launch.py mode:=move
```

```
[referee]: 30 items: 30 correct, 0 wrong bin, 0 missed, 0 dropped — 11.3 items/min
```

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

With the simulation running, start the suction node (it must warm up the suction cup before
any item appears, see section 7), the spawner and the belt (three more terminals):

```bash
ros2 run scara_sort suction
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

Item names come from a pool, `item_00` … `item_23`, because the suction cup needs every name
in advance (section 7). Before a name is used again, the oldest item, which had it, is
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

## 7. The suction cup

Gazebo has no suction, so it is faked in a way that behaves like the real thing as far as
the sorter can tell. Start the suction node right after the simulation:

```bash
ros2 run scara_sort suction
```

```
[suction]: Warm-up: spawning 24 dummy items so each suction slot welds once...
[suction]: Warm-up done: all 24 slots welded and released once
[suction]: Suction ready: /suction
```

The URDF has one Gazebo `DetachableJoint` plugin per item name (`item_00` … `item_23`, made by
an xacro loop). Told to, a plugin welds its item to the quill where it is, and lets go again.
The suction node offers the `/suction` service (std_srvs/SetBool):

- **on:** it looks up which item is right under the cup (its centre within 15 mm of the cup's
  axis, its top 0 to 5 mm below the cup), using the true positions from Gazebo, and welds it.
  With nothing there it answers `success: false, "nothing under the cup"`, like a vacuum
  sensor that never builds up pressure.
- **off:** it releases the item, which drops.

Each plugin welds its item the moment that name *first* appears in the world. So at startup
the suction node spawns all 24 names as invisible, weightless dummies off to the side, lets
each plugin weld and release its dummy once, and removes them. Only then does `/suction`
appear, and the spawner waits for it. (Restarted while the simulation keeps running, it finds
the slots already warmed up.)

Try it over the empty belt, with the cup 2 mm above where an item's top would be:

```bash
ros2 run scara_sort move_to 400 0 42
ros2 service call /suction std_srvs/srv/SetBool "{data: true}"
# std_srvs.srv.SetBool_Response(success=False, message='nothing under the cup')
```

## 8. Pick and place one item

Put an item on the stopped belt at the pick line, then move it into a bin:

```bash
ros2 run scara_sort put_item green prism 400 0     # item_00: green prism at (400, 0) mm
ros2 run scara_sort pick_place 400 0 1
```

```
Picking at (400, 0) mm → bin 1
Gripped item_00
Placed in bin 1 (4.1 s)
```

X and Y in mm, the bin is `1`, `2`, `3` or `reject`. The arm goes to the ready pose (above
the pick line, 60 mm over the item tops), moves above the item, lowers the cup to 2 mm over
its top, switches the suction on, lifts fully (a carried item then hangs 48 mm above the items
on the belt and 58 mm above the bin walls), moves over the bin, lowers until the item is 10 mm
above the walls, switches the suction off and goes back to the ready pose. The time is in
simulated seconds. With nothing there it says `Missed: nothing under the cup`. Ctrl-C
switches the suction off.

Tested 10 times for each shape, each item put down at a random angle: all 30 landed in bin 1,
and while carried the item stayed exactly where it was welded under the cup.

## 9. Sorting (stop-and-go)

One command starts the simulation, the suction node, the spawner, the detector, the sorter
and the referee:

```bash
ros2 launch scara_sort sort.launch.py
```

The suction node warms up first; the spawner and the sorter wait for it. Then:

```
[sorter]: Sorting by config/sort_by_colour.yaml, stop-and-go, belt at 50 mm/s
[sorter]: #1 green prism → bin 2 (4.4 s)
[referee]: #1 item_00 green prism: bin 2 ✓
[sorter]: #2 yellow cylinder → bin reject (3.1 s)
[referee]: #2 item_01 yellow cylinder: bin reject ✓
[sorter]: #3 yellow cube → bin reject (3.1 s)
[referee]: #3 item_02 yellow cube: bin reject ✓
...
[sorter]: #30 blue cylinder → bin 3 (3.5 s)
[referee]: #30 item_05 blue cylinder: bin 3 ✓
[referee]: 30 items: 30 correct, 0 wrong bin, 0 missed, 0 dropped — 8.4 items/min
```

The sort cycle, for each item:

1. Wait at the ready pose: above the pick line, 60 mm over the item tops.
2. When the item's predicted position reaches y = −20 mm, stop the belt (it stops within
   3 mm).
3. Move above the item, lower to 2 mm over its top, suction on. If nothing was gripped: lift,
   log the miss, and let the item ride on into the catch box.
4. Lift fully, which restarts the belt (the arm is now clear of it), move over the item's
   bin, lower until the item is 10 mm over the walls, suction off, back to the ready pose.
   (The bins are small: once a few items lie in one, the heap can stand higher than the
   walls, and the item comes to rest on it on the way down. So the quill gets the time the
   way down takes, plus 0.15 s, and the item is let go wherever it is then.)
5. Log `#7 red cube → bin 1 (3.4 s)`: simulated seconds from the belt stopping to letting go.

The sorter works only from `/items` (the camera) and the belt's encoder. The referee alone
looks at Gazebo's true positions: once an item has lain still for 1 s off the belt, it records
**correct** (in its bin), **wrong bin**, **missed** (in the catch box) or **dropped**
(anywhere else). items/min counts the correct items per minute of simulated time, from the
first item appearing to the last outcome. Ctrl-C stops the belt and switches the suction off.

Which item goes where is in a rules file: the first rule that matches wins, `'*'` matches
anything.

```yaml
# config/sort_by_colour.yaml
rules:
  - {colour: red,   shape: '*', bin: 1}
  - {colour: green, shape: '*', bin: 2}
  - {colour: blue,  shape: '*', bin: 3}
  - {colour: '*',   shape: '*', bin: reject}   # yellow, unknown, ...
```

Sorting by shape needs no code change, just the other file (cubes to bin 1, cylinders to 2,
prisms to 3):

```bash
ros2 launch scara_sort sort.launch.py rules:=config/sort_by_shape.yaml
```

Rules can mix colour and shape too, e.g. `{colour: red, shape: cube, bin: 1}`. Other launch
arguments: `mode:=move` (section 10), `count:=10`, `seed:=2`, `speed:=0.05` (m/s),
`gui:=false`, `rviz:=false`.

In RViz, **Sorting** shows the pick zone on the belt (blue, with the pick line), the bins, and
every item the camera has seen, in its colour, labelled with the bin it goes to (`#7 → 1`).

The nodes can also be started one by one, each in its own terminal, after `sim.launch.py`:
`suction`, `item_spawner`, `item_detector`, `sorter`, `referee` (the last two with
`--ros-args -p rules:=config/sort_by_shape.yaml` for shapes).

## 10. Sorting on the move

```bash
ros2 launch scara_sort sort.launch.py mode:=move
```

The belt never stops. For each item:

- **Intercept** (`intercept.py`): each joint motor runs at its top speed until it is there, so
  a move takes as long as its slowest joint (|Δj1|/2, |Δj2|/2 rad/s, |Δj3|/0.2 m/s). Along the
  item's path, the sorter takes the first point in the pick zone where the arm, setting off
  now, gets there first (with a 0.25 s margin), leaving room in the zone for the 0.6 s it
  then rides along. The arm goes there, 20 mm over the item tops, and waits.
- **Follow**: every 20 ms it sends the arm to where the item is now (aimed 30 ms ahead, for
  the arm's lag), while lowering the cup to 2 mm over the item, switching the suction on once
  the cup is over it, and lifting until the item is 30 mm over the belt. Then the arm swings
  over to the bin, the cup rising fully on the way, places the item as before and goes
  straight on to the next one.
- **Skip**: if the arm can't catch an item before it leaves the pick zone, it logs
  `missed #9 (arm busy)` and lets it ride into the catch box. A skipped item never goes into a
  wrong bin.

```
[sorter]: Sorting by config/sort_by_colour.yaml, on the move, belt at 50 mm/s
[sorter]: #1 green prism → bin 2 (13.2 s; met at y -150 mm, 1.4 mm off the cup's axis)
[referee]: #1 item_00 green prism: bin 2 ✓
[sorter]: #2 yellow cylinder → bin reject (4.1 s; met at y -150 mm, 0.3 mm off the cup's axis)
[referee]: #2 item_01 yellow cylinder: bin reject ✓
...
[sorter]: #30 blue cylinder → bin 3 (4.6 s; met at y -132 mm, 0.5 mm off the cup's axis)
[referee]: #30 item_05 blue cylinder: bin 3 ✓
[referee]: 30 items: 30 correct, 0 wrong bin, 0 missed, 0 dropped — 11.3 items/min
```

11.3 items/min against 8.4 stop-and-go: the belt keeps going, so items arrive at its full
rate (one per 250 mm, every 5 s at 50 mm/s), and the arm keeps up. The time is now from
choosing the item to letting go, including the wait for it to arrive (13.2 s for the first
item, which the arm meets as soon as it enters the pick zone, at y = −150 mm). "Off the cup's
axis" is how far the item's true centre was from the cup when it was gripped, measured by the
suction node: how well the cup keeps up with a moving item. At 50 mm/s it stays within
1.5 mm.

At 100 mm/s, items come every 2.5 s, faster than the arm can sort them:

```bash
ros2 launch scara_sort sort.launch.py mode:=move speed:=0.10
```

```
[sorter]: #2 yellow cylinder → bin reject (3.6 s; met at y 56 mm, 0.7 mm off the cup's axis)
[sorter]: missed #3 (arm busy): at y 78 mm the arm can no longer catch it, it rides on into the catch box
[referee]: #3 item_02 yellow cube: the catch box ✗ missed
...
[referee]: 30 items: 20 correct, 0 wrong bin, 10 missed, 0 dropped — 13.2 items/min
```

Every item the arm can't reach in time is skipped and lands in the catch box; none goes into a
wrong bin. At this speed grips were mostly within 5 mm of the cup's axis, but one was 13.3 mm
off (the cup accepts 15 mm), even though the detector's prediction is just as good at
100 mm/s (worst error 1.7 mm, checked with `vision_check`). So the cup itself fell behind
for a moment there; that is not explained yet.

## 11. Unit tests

The kinematics, the cell layout, the items, the vision (on synthetic images drawn with
OpenCV), the tracker, the suction's grip check, the sort rules and the intercept are tested
without Gazebo:

```bash
colcon test --packages-select scara_sort --event-handlers console_direct+
```

It should end with `112 passed`. `test_cell.py` checks that every bin centre and corner and the
whole pick zone are reachable. The hardest point is the reject bin's far corner, at
(−0.44, 0.00) m: the shoulder turns 147° there, inside its ±150° range.

## Files

| File | What it does |
|---|---|
| `src/scara_sort_sim/urdf/scara.urdf.xacro` | The arm: column, two links, quill with vacuum tube and suction cup, `cup_tip` frame, joint motors, 24 suction slots |
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
| `src/scara_sort/scara_sort/grip.py` | Which item is right under the cup (the suction's stand-in for a vacuum sensor) |
| `src/scara_sort/scara_sort/suction.py` | `suction` node: warm-up, then `/suction` on/off with the DetachableJoint plugins |
| `src/scara_sort/scara_sort/pick_place.py` | `pick_place` node, and the `Arm` class with the pick and place moves |
| `src/scara_sort/scara_sort/put_item.py` | `put_item` node: puts one item on the belt, for trying things by hand |
| `src/scara_sort/scara_sort/rules.py` | Sort rules: which bin each kind of item goes into |
| `src/scara_sort/scara_sort/sorter.py` | `sorter` node: stop-and-go or on the move, belt control, RViz markers |
| `src/scara_sort/scara_sort/intercept.py` | Where the arm can meet a moving item: move times, meeting point |
| `src/scara_sort/scara_sort/referee.py` | `referee` node: scores where each item really ended up |
| `src/scara_sort/launch/sort.launch.py` | Starts everything: simulation, suction, spawner, detector, sorter, referee |
| `src/scara_sort/config/sort_by_colour.yaml`, `sort_by_shape.yaml` | Rules files |
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
| 4 suction | Suction cup, `pick_place` | done |
| 5 sort | Stop-and-go sorter, referee | done |
| 6 on the move | Picking from the running belt | done |

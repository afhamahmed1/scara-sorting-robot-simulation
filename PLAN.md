# SCARA Sorter — project plan

A SCARA robot arm with a suction cup sorts items coming down a conveyor belt, simulated in
Gazebo. A camera above the belt identifies each item by its colour and shape. The arm picks
the item off the belt and drops it into the bin its kind belongs to. A referee node checks
every result against the true positions in Gazebo, so the scores are measured, not guessed.

Stack: ROS 2 Jazzy + Gazebo Harmonic, Python nodes, OpenCV. Builds on the SCARA Driller
project: same arm, same kinematics, same joint motors.

## How it works

```
item_spawner ──► Gazebo: an item appears at the start of the belt ──► the belt carries it (+y)
                                         │
                            camera (looks straight down, upstream of the arm)
                                         │ /camera/image
                                         ▼
item_detector ── 1. find    colour masks (HSV) → blobs
              ── 2. name    shape from the outline: square = cube, circle = cylinder,
                            triangle = prism
              ── 3. locate  pixel → position on the belt (mm)
              ── 4. track   the same item across frames; once it has left the camera's view,
                            predict where it is from the belt encoder
                                         │ /items  (id, colour, shape, x, y)
                                         ▼
sorter ── rules: which bin? ── item reaches the pick line ── stop the belt (stage 5)
                                                             or chase the item (stage 6)
       ── pick: above → down → suction on → up ── place: over the bin → suction off ── belt on
              │ joint angles (inverse kinematics)               │ /suction
              ▼                                                 ▼
   /scara/j1_cmd … j3_cmd ──► Gazebo joint motors      suction node ──► Gazebo DetachableJoint

referee ── watches the true item positions in Gazebo and scores where each item ended up
```

## What carries over from the driller

- **The arm**: the same URDF with the drill swapped for a suction cup. The cup's tip sits
  exactly where the drill tip was, so `kinematics.py` and its tests copy over unchanged.
  Only the names change (`drill_tip` → `cup_tip`).
- **The joint motors** and the `/scara/j1_cmd` … `/scara/j3_cmd` topics. They are faster here
  (j1 and j2 at 2 rad/s, j3 at 0.2 m/s) because nothing is being cut.
- **`move_to`** (give it a point in mm and the arm goes there), for testing.
- **Spawning and removing models from Python with `gz service`**, as `drill_job` does.
- **The way of working**: one stage per git branch, a README with exact commands and the
  output to expect, and unit tests that run without Gazebo.

## The cell

Top view. The robot base is at (0, 0), x points forward and y to the robot's left, all in
metres. The belt top is at z = 0.20 m, the same height as the driller's table, so "height"
in `move_to` now means the height above the belt.

```
             y ▲ (robot's left)
               │                                   ┌────────┐ catch box (on the floor)
    bin 2      │   bin 1                           │        │ y 0.64 – 0.92, reaching
 (-0.12,0.42)  │ (0.12,0.40)                       ╞════════╡ y +0.70  under the belt end
               │                                   ║        ║
 bin 3         │                                   ║   ▲    ║ items move +y, 50 mm/s
 (-0.34,0.24)  │                                   ║   │    ║
               │                                   ║        ║
 reject        │                                   ╟────────╢ y +0.15 ┐
 (-0.38,0.06)  ◉ robot ────────────────────────────╫─ ─ ─ ─ ╫─► x     │ pick zone, pick line at y 0
               │                                   ╟────────╢ y −0.15 ┘
               │                                   ║        ║
               │                                   ╟────────╢ y −0.23 ┐
               │                                   ║ [cam]  ║         │ camera view
               │                                   ║   ●    ║ spawn   ┘ y −0.67
                                                   ╞════════╡ y −0.70   belt start
                                                 x 0.32   x 0.48
```

| Thing | Where | Size |
|---|---|---|
| Robot | base at (0, 0) | the driller's arm: reach 0.17 – 0.55 m |
| Belt | x 0.32 – 0.48, y −0.70 – +0.70, top at z 0.20 | 0.16 × 1.40 m, dark grey, low side rails |
| Spawn point | (0.40 ± 0.03, −0.65) | |
| Camera | above (0.40, −0.45), z 0.85, looking straight down; on a pole at (0.70, −0.45), clear of the arm | 640 × 480, 40° field of view → sees y −0.67 – −0.23, about 0.7 mm per pixel |
| Pick zone | y −0.15 – +0.15 across the belt; the pick line is at y = 0 | |
| Bins 1, 2, 3, reject | centres in the drawing | 0.12 × 0.12 m inside, floor at z 0.15, walls up to z 0.23 |
| Catch box | x 0.30 – 0.50, y 0.64 – 0.92, on the floor, its near wall under the end of the belt | catches items nobody picked; items falling off the end can't land or roll in front of it |

These numbers have been checked against the arm's kinematics. Every bin centre and corner and
the whole pick zone are reachable, and the largest shoulder angle needed is 147°, at the
reject bin's far corner (−0.44, 0.00) (the limit is ±150°; its centre needs 130°). The bins are on the robot's left and behind it, so the camera never sees
the arm.

Heights: with the cup fully up (j3 = 0), a carried item hangs with its bottom at z 0.29. That
clears items on the belt (tops at z 0.24) by 50 mm and the bin walls by 60 mm. So the arm
always travels with the cup fully up.

## The items

| | |
|---|---|
| Shapes | cube 40 mm · cylinder Ø40 × 40 mm (standing up) · triangular prism (40 mm sides, 40 mm tall) |
| Colours | red, green, blue, yellow: strong matt colours that are easy to tell apart on a dark belt |
| Mass | 50 g, high friction so they don't slide on the belt |
| Collision | the prism's is three boxes filling its triangle (it still looks exact): Gazebo makes a triangle mesh of a prism, and piles of meshes in a bin slow the simulation down several times |

All items are 40 mm tall. Their top faces therefore all lie in one plane, which makes
turning a pixel into a position simple (see Vision).

`config/items.yaml`:

```yaml
colours: [red, green, blue, yellow]
shapes: [cube, cylinder, prism]
spacing: 250      # mm of belt travel between items (± 50 mm random)
lateral: 30       # mm, random offset from the belt centre
count: 30         # items per run, 0 = keep going
seed: 1           # same seed → same items in the same order
```

Items are spawned by **belt travel, read from the belt encoder, not by time**. When the belt
stops for a pick, new items stop arriving too and never pile up at the start.

The spawner adds and removes models through Gazebo's `create` and `remove` services, bridged
to ROS 2 by `ros_gz_bridge` (`ros_gz_interfaces/srv/SpawnEntity`, `DeleteEntity`). The
position goes in the request, not the SDF: the bridged request always carries a pose, and
Gazebo then ignores the SDF's.

## The belt

Gazebo's `TrackController` plugin turns a static box into a conveyor belt. It makes the
belt's surface drag whatever sits on it at the commanded speed (the `examples/worlds/conveyor.sdf`
world in Gazebo works the same way).

- `/conveyor/cmd_vel` (std_msgs/Float64, m/s): set the belt speed; 0 stops it.
- `/conveyor/odom` (nav_msgs/Odometry, 50 Hz): how far the belt has moved. This is the belt's
  **encoder**, the key to tracking items once they are out of the camera's view.
- Acceleration is limited (`max_acceleration` 0.5 m/s²), so items don't slide when the belt
  starts or stops. From 50 mm/s it stops within 3 mm.

## Identifying items (vision)

`vision.py` holds plain functions with no ROS in them, so they can be tested on synthetic
images drawn with OpenCV.

1. **Colour**: convert the image to HSV and make one mask per colour, using the ranges in
   `config/colours.yaml` (red is two ranges, because its hue wraps around 0). Clean each
   mask (morphological open) and find the blobs (contours). Only blobs on the belt count.
   Blobs touching the image edge are ignored until the item is fully in view.
   For items away from the image centre the camera also sees a side face; the side facing
   the light reaches about 0.77 of the top face's brightness. Each colour's lowest value
   (V) is set between its top and its brightest side, so a blob is exactly the top face.
2. **Shape**: simplify the outline (`approxPolyDP`, tolerance 4 % of its length). 3 corners
   = prism, 4 = cube. If the outline is round (circularity 4π·area/perimeter² > 0.85; a
   square scores 0.79, a circle 1.0), it is a cylinder. Any blob much smaller or larger than
   one item, such as two touching items, is `unknown`.
3. **Position**: the blob's centre pixel (u, v) → a point on the plane of the item tops.
   The camera looks straight down from 0.61 m above that plane, so the offset from the point
   under the camera is `((u − cx)·0.61/fx, (v − cy)·0.61/fy)`, turned into robot
   coordinates. fx, fy, cx and cy come from `/camera/camera_info`.

## Tracking

`tracker.py`, also plain Python and unit-tested.

- Each item is stored with where it was seen and the encoder reading at that moment. Where it
  is now: `y = y_seen + (belt_now − belt_seen)`. This works whether the belt runs, stops or
  changes speed.
- A new detection updates an existing item if it lies within 15 mm of that item's predicted
  position and has the same colour; otherwise it becomes a new item with the next id.
- An item is **confirmed** once it has been seen in 3 frames. Its colour and shape are the most
  common answers over all the frames it was seen in.
- `item_detector` publishes every confirmed item on `/items` at 10 Hz, with positions
  predicted to the current time. It also publishes `/camera/detections`, the camera image
  with outlines and labels such as `#4 red cube` drawn on, for RViz.

## The suction cup

Gazebo has no suction, so the plan fakes it in a way that behaves like the real thing as far
as the sorter can tell.

- The cup is a black disc under the quill. Like the drill bit, it is visual only, with no
  collision shape.
- Gazebo's `DetachableJoint` plugin can weld a model to the cup and release it on command.
  Each plugin needs its model's name in advance, so items take their names from a fixed pool
  (`item_00` … `item_23`). The URDF holds one plugin per name, generated by an xacro loop,
  each with its own attach and detach topics.
- The **`suction` node** offers the `/suction` service (std_srvs/SetBool).
  - **On**: it looks up which item is right under the cup (within 15 mm sideways and 5 mm
    below), using the true positions from Gazebo, and welds that item. If there is none, it
    answers `success: false, "nothing under the cup"`, just as a real vacuum sensor never
    builds up pressure.
  - **Off**: it releases the item.
- **Quirk** (checked in the Harmonic source): each plugin welds its item as soon as that
  name *first* appears in the world (`attachRequested` starts as true). So at startup the
  suction node **warms up** every slot. With the arm standing still, it spawns all 24 names
  off to the side (invisible, weightless dummies), releases them, and removes them. After
  that, a reused name is never welded unless the node asks for it. The `/suction` service only
  appears once the warm-up is done, and the spawner waits for it.
- A request to weld a name that isn't in the world stays pending in its plugin until the name
  appears. So the suction node releases at once any weld it didn't just ask for.
- The suction node talks to the plugins' topics with Gazebo's own Python bindings
  (`/scara/suction/item_07/attach`, `.../detach`, `.../state`), so they need no bridge entries.
- `put_item COLOUR SHAPE X Y` puts one item on the belt by hand, for trying `pick_place`.
- The spawner removes the oldest item before reusing its name, so each bin keeps its most
  recent items. It does so right after spawning the item before, so the old model is surely
  gone when the name is used again (Gazebo removes models at the end of a step).

## Sort rules

Each rules file maps item kinds to bins. The first matching rule wins and `'*'` matches
anything:

```yaml
# config/sort_by_colour.yaml
rules:
  - {colour: red,   shape: '*', bin: 1}
  - {colour: green, shape: '*', bin: 2}
  - {colour: blue,  shape: '*', bin: 3}
  - {colour: '*',   shape: '*', bin: reject}   # yellow, unknown, ...
```

`sort_by_shape.yaml` sends cubes to bin 1, cylinders to bin 2 and prisms to bin 3. Rules can
also mix colour and shape (`{colour: red, shape: cube, bin: 1}`). Changing the sorting means
changing a file, not code.

## The sort cycle

**Stop-and-go (stage 5):**

1. Wait at the ready pose: above the pick line, 60 mm over the item tops.
2. When the next confirmed item's predicted position reaches y = −0.02, stop the belt.
3. Move above the item → lower to 2 mm over its top → suction on. If nothing was gripped,
   lift, log the miss and let the item ride on into the catch box.
4. Lift fully, which also restarts the belt, since the arm is now clear of it. Move over the
   item's bin → lower to 10 mm above the walls → suction off → back to the ready pose.
5. Log, e.g. `#7 red cube → bin 1 (3.4 s)`.

**On the move (stage 6):** the belt never stops.

- **Intercept**: from the item's predicted path and how long the arm needs to get there
  (joint distance ÷ joint speed, plus a margin), pick the point in the pick zone where arm
  and item meet. Move there.
- **Follow**: while lowering, send the IK solution for the item's current position every
  20 ms, so the cup rides along with it. Grip, lift while still following, then place as
  before.
- **Skip**: if the arm can't reach an item before it leaves the pick zone, skip it and log
  `missed #9 (arm busy)`. It rides into the catch box. A skipped item is never put in a
  wrong bin.

Ctrl-C stops the belt and releases the suction.

## Measuring the result

**Ground-truth rule:** the true positions from Gazebo (`/world/sort_cell/dynamic_pose/info`,
copied to `/gz/poses` by the `gz_poses` node, because `ros_gz_bridge` drops the model names)
may be used only by the "simulated hardware" and the checkers. That
means the suction node, the referee and `vision_check`. The detector and the sorter work
only from the camera and the encoder, exactly as they would on a real belt.

- **`referee`** gets each item's true kind from the spawner (`/items/spawned`) and watches the
  true positions. Once an item has been still for 1 s, it records the outcome: **correct
  bin**, **wrong bin**, **missed** (in the catch box) or **dropped** (anywhere else). At the
  end of a run it prints a table:

  ```
  30 items: 30 correct, 0 wrong bin, 0 missed, 0 dropped — 9.6 items/min
  ```

- **`vision_check`** compares every item on `/items` with its true position and kind, and
  prints the misidentifications and position errors in mm.

## Topics and services

| Name | Type | From → to |
|---|---|---|
| `/conveyor/cmd_vel` | std_msgs/Float64 (m/s) | sorter → belt |
| `/conveyor/odom` | nav_msgs/Odometry | belt → detector, spawner (the encoder) |
| `/camera/image`, `/camera/camera_info` | sensor_msgs | camera → detector |
| `/camera/detections` | sensor_msgs/Image | detector → RViz |
| `/items` | scara_sort_msgs/ItemArray | detector → sorter |
| `/scara/j1_cmd` … `j3_cmd`, `/joint_states` | as in the driller | sorter ↔ motors |
| `/suction` | std_srvs/SetBool (service) | sorter → suction node |
| `/items/spawned` | scara_sort_msgs/SpawnedItem | spawner → referee (the truth) |
| `/gz/poses` | tf2_msgs/TFMessage | `gz_poses` → suction node, referee, vision_check |
| `/world/sort_cell/create`, `/remove` | ros_gz_interfaces SpawnEntity, DeleteEntity (services, bridged) | spawner, suction node → Gazebo |
| `/sort_markers` | visualization_msgs/MarkerArray | sorter → RViz: items labelled with their bin, pick zone, bins |

```
# Item.msg
uint32 id           # given by the tracker, counts up
string colour       # red, green, blue, yellow or unknown
string shape        # cube, cylinder, prism or unknown
float64 x           # m, robot frame, where the item is now (predicted from the encoder)
float64 y
uint32 times_seen   # frames the camera saw it in; confirmed from 3

# ItemArray.msg
std_msgs/Header header
Item[] items

# SpawnedItem.msg
uint32 number       # 1 for the first item of the run, then 2, 3, ...
string name         # Gazebo model name, e.g. item_07
string colour
string shape
```

## Packages

The workspace is `~/Robotics/Projects/scara_sort_ws`, and the repo is the workspace: README
and PLAN at the top, packages in `src/`.

| Package | Contents |
|---|---|
| `scara_sort_msgs` | `Item`, `ItemArray`, `SpawnedItem` messages |
| `scara_sort_sim` | URDF (arm, cup, suction slots), world `sort_cell.sdf` (belt, bins, catch box, camera), bridge config, `gz_poses` node, `sim.launch.py`, RViz layout |
| `scara_sort` | plain Python: `kinematics.py`, `vision.py`, `tracker.py`, `rules.py`, `intercept.py` (all unit-tested). Nodes: `move_to`, `item_spawner`, `item_detector`, `suction`, `pick_place`, `sorter`, `referee`, `vision_check`. Plus `config/*.yaml`, `launch/sort.launch.py` and tests |

The sim package is named `scara_sort_sim`, not `scara_sim`, so it can't clash with the driller's
package if both workspaces are ever sourced.

Extra dependencies on top of the driller's: `sudo apt install ros-jazzy-cv-bridge python3-opencv`.

## Stages (one git branch each, `main` = latest finished stage)

| Branch | Adds | Done when |
|---|---|---|
| `stage-1-cell` | The arm with the cup (no suction yet), faster motors. World: belt, rails, bins, catch box, camera on its pole, one grey test box on the belt. Bridge for the belt. `kinematics.py` + `move_to` copied, with tests | `ros2 topic pub --once /conveyor/cmd_vel std_msgs/msg/Float64 "{data: 0.05}"` carries the test box from one end into the catch box. `/conveyor/odom` counts up at 0.05 m/s. `move_to` reaches every bin centre and both ends of the pick zone. A test checks all of them are reachable |
| `stage-2-items` | `item_spawner` + `items.yaml`; truth on `/items/spawned`; test box removed | 30 mixed items ride end to end into the catch box, none tipped over or stuck. Same seed → same items |
| `stage-3-vision` | camera bridged; `vision.py`, `tracker.py`, `item_detector`, `/camera/detections` in RViz, `vision_check` | seeded 30-item run: every item named right. Position error ≤ 3 mm under the camera and ≤ 5 mm when predicted at the pick line. Synthetic-image tests pass |
| `stage-4-suction` | suction slots (xacro loop), warm-up, `suction` node, `pick_place X Y BIN` for an item on the stopped belt | `ros2 run scara_sort pick_place 400 0 1` lands the item in bin 1, 10 out of 10, for each shape. Suction on over an empty belt answers "nothing under the cup". Carried items don't wobble or fall |
| `stage-5-sort` | `rules.py`, rules files, `sorter` (stop-and-go), `referee`, markers, `sort.launch.py` | seeded 30-item run with `sort_by_colour`: referee says 30/30 correct. Switching to `sort_by_shape` needs no code change. Items/min printed |
| `stage-6-on-the-move` | `intercept.py` (tested), follow-while-picking, skip logic | same run at 50 mm/s without stopping the belt: 30/30 correct and more items/min than stage 5. At 100 mm/s any item the arm can't catch ends in the catch box as "missed", never in a wrong bin |

Throughput to expect: one pick-and-place takes about 3–5 s with the faster motors, which is
roughly 8 items/min stop-and-go. Picking on the move should beat that. The referee measures
the real numbers.

## Risks and what to do about them

| Risk | What to do |
|---|---|
| Suction welds an item the moment it first appears | warm-up at startup (stage 4). If welding still misbehaves, fall back to carrying the item by setting its pose every step (`/world/sort_cell/set_pose`) |
| Colours look different under Gazebo's light | strong matt colours, dark belt, no shadows; HSV ranges in `colours.yaml`, tuned once using the annotated image |
| Items slide when the belt starts or stops | high friction on belt and items; belt acceleration limited |
| The camera needs a GPU renderer (ogre2) | it's the renderer the Gazebo window already uses; if the image stays black, try `--render-engine ogre` |
| The arm lags behind a moving item | measured in stage 6 (true cup position vs. true item position); aim ahead by belt speed × lag |

## Later ideas

- A 4th joint (wrist rotation) to set items down neatly aligned, using each item's angle
  from the camera.
- A learned detector (e.g. YOLO) trained on simulator images that label themselves from the
  true poses.
- Items of different heights, measured with a depth camera instead of assuming 40 mm.
- Two arms on one belt, splitting the work.
- `ros2_control` + MoveIt, as in the driller's later ideas.

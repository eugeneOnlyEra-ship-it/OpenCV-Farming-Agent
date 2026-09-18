# OpenCV-Farming-Agent — PyBullet Simulation (Step 4)

Standalone PyBullet loop for the pod-farm inspection robot, built
against a **stub** of the `classify_pod()` interface so this doesn't
have to wait on the trained perception models. This implements items
1–7 from the handoff doc's "What This Chat Should Help With" section,
laid out around the pole/trough station design in
`Opencv_agent_pods.svg`: each station is a vertical pole with
elongated rectangular trough pods stacked at several heights, mirrored
on the pole's front and back face — arranged in a grid of stations
with realistic walkable spacing, serviced by an overhead bridge-crane
robot rather than a free-floating stage.

## Farm layout

- **3 rows x 3 columns of stations** (poles) = 9 poles, **4 trough
  layers** per pole, **2 sides** per layer (front/back) → **72 pods**
  total.
- Each pod is an elongated box (`TROUGH_HALF_EXTENTS_M` in
  `pod_registry.py`) matching the rectangular trough shape in the SVG,
  not a plain cube.
- **Column spacing** (pole-to-pole within a row): 0.75m — clearance
  for the trough width (0.44m) plus a bit of margin for manual access
  to one pole.
- **Row spacing** (aisle between rows): derived from a **1.0m clear
  walking gap** (standard single-person working-aisle range is
  roughly 0.9–1.2m in warehouse/greenhouse settings) plus 0.32m of
  trough overhang on both sides → **1.32m** pole-to-pole.
- Change `ROWS` / `COLUMNS` / `AISLE_CLEAR_WIDTH_M` / spacing constants
  in `pod_registry.py` to resize the farm.

## The robot: an overhead bridge crane, not a floating stage

The first version of this robot anchored its base at ground level and
moved the whole assembly up with a plain vertical joint — which looked
like it was hovering in open air with nothing holding it up. This
version is built the way a real gantry/bridge crane actually works:

- **Poles are taller than the pod stack** — tall enough that the
  **fully retracted** hoist (see movement sequencing below) clears
  every trough top with real margin (`RAIL_CLEARANCE_M` = 0.75m above
  the top trough layer). This was measured directly, not assumed: at
  an earlier, shorter value the retracted camera was actually 1.5cm
  *below* the top trough's surface, which would have let it clip pods
  during horizontal travel.
- **A full grid of horizontal beams connects every pole top** — beams
  running along each row (X) *and* along each column (Y), forming a
  lattice the trolley notionally rides on. This is the "horizontal
  pole interconnecting the vertical poles" from your description.
- **The robot's base is anchored UP at that rail height**, not at the
  ground. `carriage_joint` (X) and `row_joint` (Y) move the trolley
  across the rail lattice — the trolley never leaves that height.
- **A hoist arm telescopes DOWN** from the trolley to reach each
  layer: a short fixed-length mast (`MAST_LENGTH_M` = 0.35m, always
  visible, hanging from the trolley), a small mounting stub, and a
  dynamically-sized rod (see below) that closes the gap down to the
  camera at the tip.
- **Movement is a five-phase sequence, not simultaneous joint
  commands** — the way a real automated storage/retrieval shuttle
  operates, not just "travel then descend":
  1. **retreat**: slide sideways back to the *current* pole's
     centerline, still at the current height, before lifting at all
  2. **retract**: pull the hoist fully up, now climbing a clear
     centerline column instead of through the trough just serviced
  3. **travel**: move to the new column and the *new* pole's
     centerline, at that safe retracted height
  4. **descend**: straight down the new pole's centerline to the
     target layer
  5. **approach**: slide sideways the last short distance into the
     pod's actual front/back position, now that the hoist is already
     at the right height

  This exists because of a real geometric fact about the farm: all 4
  layers on a given pole+side share the *same* x/y, only z differs. A
  simpler "travel then descend straight down" sequence works fine for
  the topmost layer, but for any lower layer it drives the hoist
  directly through the solid trough boxes of every layer above it —
  and symmetrically, retracting straight up out of a trough it was
  just sitting in clips that same trough on the way out. Routing the
  vertical motion through the pole's own centerline (where nothing is
  mounted — troughs sit offset to the front/back, not on the
  centerline) and only reaching sideways into a target once already at
  the correct height avoids both. This isn't a "looks right in a
  render" claim: verified by checking the camera's exact position
  against every trough's bounding box at **every single simulation
  step** across a full 72-pod run — zero intersections with any pod
  other than the one actually being serviced at rest, out of 70,000+
  steps checked.
- **The connecting rod is redrawn continuously, not just at rest.**
  `GantryRobot._sync_hoist_visual()` recreates a rod spanning exactly
  from the mast's bottom to the camera's actual current position, and
  is called on every simulation step during the retreat/retract/
  descend/approach phases (not only once settled) — otherwise the rod
  stays at its old length for the whole motion and then snaps into
  place at the end, which looks like it's appearing/disappearing
  rather than actually extending.

So at every point in the sim, the robot is either sitting on the rail
or hanging from something that is, the connecting rod never shows a
gap, and the hoist's path never runs through a pod other than the one
it's actually visiting — checked directly, not assumed.

## Files

| File | What it does |
|---|---|
| `pod_registry.py` | Farm layout: rows × columns × layers × sides, each with a 3D position, crop type, and a ground-truth (growth_stage, diseased) condition. `pod_visit_order()` returns a full boustrophedon traversal (serpentine across columns within a row, alternating bottom-to-top per pole, front-then-back per layer). |
| `generate_placeholder_images.py` | Generates labeled placeholder PNGs into `sample_images/`. **Not real data** — see "Swapping in real data" below. |
| `perception_stub.py` | `classify_pod(pod_id, crop_type, image)` — the locked-in interface contract. Two modes: pure-random stub, and a "noisy oracle" that mostly echoes ground truth for cleaner demo runs. |
| `decision_agent.py` | Rule-based `decide()`: low confidence → `schedule_monitoring`; disease detected → `flag_for_treatment`; healthy + near-harvest stage → `schedule_monitoring`; otherwise → `log_and_continue`. Also defines per-action robot dwell times. |
| `gantry_robot.py` | Loads `urdf/gantry.urdf`, builds the pole grid + overhead beam lattice + trough markers (visual-only — see "Known gotchas" below), and drives the trolley/hoist through each pod's retract → travel → descend sequence. Recolors each trough by decision (green/yellow/red) after every visit. |
| `urdf/gantry.urdf` | Bridge-crane kinematic chain: `carriage_joint` (X, trolley along a row's rail), `row_joint` (Y, between-aisle travel *and* the front/back face switch — one axis covers both), a **fixed** mast stub hanging from the trolley, `hoist_joint` (Z, telescopes down from the mast to the target layer), and a camera marker at the tip. |
| `run_logger.py` | Writes one CSV + JSON row per pod visit: timestamp, pod, row, col, layer, side, crop, growth_stage, disease_flag, confidence, action, reason, ground truth. This is the artifact for the Agentic Vision Award and later feeds DynamoDB. |
| `main.py` | Orchestrates the full loop and CLI. |

## Running it

```bash
pip install pybullet pillow numpy

# headless (no display needed) — full 72-pod farm, pure-random stub
python3 main.py

# GUI window (needs a local display), noisy-oracle stub for a cleaner
# demo trace, slowed down so it's watchable
python3 main.py --gui --oracle --sleep 0.004

# quick smoke test on just the first 8 pods
python3 main.py --pods 8 --oracle --seed 42
```

Each run prints a live `[pod] saw X -> decided Y` trace and writes
`logs/run_<timestamp>.csv` / `.json`.

Once the window is open, PyBullet's default mouse controls work for
inspecting the farm from any angle: left-click + drag to orbit, scroll
to zoom, ctrl/middle-click + drag to pan.

**Note on this sandbox:** this container has no display, so everything
was tested with `--gui` **off** (`p.DIRECT` mode) — physics, all three
joints' position control, classification, decision logic, marker
recoloring, and logging all verified working on the full 72-pod farm
(no phase of the retract/travel/descend sequence ever hit its
1400-step cap). Two
static camera renders were also used during development to visually
confirm the trolley/hoist is actually connected to the rail structure
(not included in this delivery). Run with `--gui` on your machine to
see the actual window; nothing else changes.

## What the GUI shows

- A 3×3 grid of grey poles, taller than the pod stack, connected at
  the top by a full lattice of horizontal beams — the overhead rail
  structure.
- Elongated trough-shaped pods at 4 heights on each pole, on both the
  front and back face — colored by crop until visited, then recolored
  by the decision made there: 🟢 `log_and_continue` · 🟡
  `schedule_monitoring` · 🔴 `flag_for_treatment`.
- An orange **carriage** riding the rail along X, a teal **row**
  block riding along Y, a grey **mast** stub hanging fixed below them,
  a dynamically-sized blue **rod** that exactly closes the gap down to
  the camera at every layer (see below), a small blue hoist mount, and
  a dark cylinder camera marker at the tip.
- A short dark **bracket** physically connecting every trough to its
  pole, so troughs don't appear to float next to the structure with
  nothing holding them there.
- The robot visibly dwells longer at flagged pods than at healthy ones
  (`decision_agent.ACTION_DWELL_SECONDS`) — the decision changes the
  robot's physical behavior, not just a printed label.

## Known gotchas (already fixed, worth knowing about)

**The carriage was landing 0.35m off in X from every single pod, in
every column, the whole time.** The robot's base sits at a fixed world
position, and `move_to()` sets `carriage_joint`'s target directly to
the pod's world x-coordinate — but an earlier version anchored the
base at x=-0.35 (purely a cosmetic choice, so the opening move would
be visible instead of starting inside the first pole) and never
corrected the joint targets for that offset. Since a joint's
POSITION_CONTROL target is relative to the base, not an absolute world
coordinate, the carriage's real world position was always `base_x +
joint_value` = `-0.35 + target_x` — every pod, every column, off by a
constant 0.35m. This was never visually obvious in any render; it only
surfaced once exact camera-to-target alignment was checked
numerically (see the fix below), not from "does this look roughly
right" screenshots. Fixed by anchoring the base at world (0, 0, rail
height) so joint values and world coordinates coincide — verified
directly afterward: camera x/y now matches every checked pod's target
x/y with **zero** error, across multiple rows and columns, not just
"close enough."

**The hoist arm used to show a visible gap between the mast and the
camera when reaching lower pods, and the rod would appear/disappear
rather than extend.** Two related problems, both fixed: (1) the
original hoist was a single fixed-length rigid rod (0.22m) sliding
down a variable distance — the math only works out gap-free for
extensions up to *half* the rod's own length (0.11m), and every layer
in this farm needs 4–15x that much (0.40–1.66m, measured directly). A
true multi-stage telescoping mechanism that's both gap-free and fully
retracts when not needed would need 5+ nested segments and was judged
more complexity than the payoff justified here, so instead the URDF's
`hoist_link` is now just a small mounting stub, and a *separate*,
freestanding rod (`GantryRobot._sync_hoist_visual`) is redrawn to
exactly span from the mast's bottom to the camera's actual current
position. (2) That rod was initially only redrawn at the *end* of each
phase, once settled — meaning during the actual motion it stayed at
its old length the whole time and then snapped to the correct length
right at the end, which looked like it was appearing out of nowhere
rather than extending. Fixed by redrawing it on **every simulation
step** during motion (`sync_visual=True` in `_settle()`), not just
once at rest.

**The hoist's straight vertical descent used to drive it directly
through the trough boxes of every layer above the target, and
retracting afterward clipped the trough it had just left.** This is a
real geometric fact about the farm, not a control-timing issue: all 4
layers on a given pole+side share the exact same x/y, only z differs.
Descending straight down to a low layer necessarily passes through the
z-range of every layer above it at that same x/y. Fixed by routing all
vertical motion through the pole's own centerline instead of the
pod's actual front/back offset — troughs are mounted offset to the
front/back, not on the centerline, so nothing sits on that line at any
height. The robot now **retreats** to centerline before lifting,
**retracts**, **travels** to the new pole's centerline, **descends**
there, then **approaches** sideways into the target only once already
at the correct height (five phases total — see "The robot" section
above for the full breakdown). Verified exhaustively, not just
visually: the camera's position was checked against every trough's
bounding box at every single simulation step across a full 72-pod run
— **zero** intersections with any pod other than the one actually
being serviced at rest, out of 70,000+ steps checked. If you resize
the farm (different layer spacing, trough size, etc.), that
verification script is worth re-running rather than assuming the
geometry still clears — it isn't in this delivery, but the check is
straightforward: step through every phase of every `move_to()` call
and compare the camera link's world position against every trough's
half-extents.

**The carriage was landing 0.35m off in X from every single pod, in
every column, the whole time.** The robot's base sits at a fixed world
position, and `move_to()` sets `carriage_joint`'s target directly to
the pod's world x-coordinate — but an earlier version anchored the
base at x=-0.35 (purely a cosmetic choice, so the opening move would
be visible instead of starting inside the first pole) and never
corrected the joint targets for that offset. Since a joint's
POSITION_CONTROL target is relative to the base, not an absolute world
coordinate, the carriage's real world position was always `base_x +
joint_value` = `-0.35 + target_x` — every pod, every column, off by a
constant 0.35m. This was never visually obvious in any render; it only
surfaced once exact camera-to-target alignment was checked
numerically, not from "does this look roughly right" screenshots.
Fixed by anchoring the base at world (0, 0, rail height) so joint
values and world coordinates coincide — verified directly afterward:
camera x/y now matches every checked pod's target x/y with **zero**
error, across multiple rows and columns, not just "close enough."

**The camera used to be able to clip through pods while traveling
between stations.** The original `move_to()` drove the trolley (X/Y)
and the hoist (Z) all at once, so partway through a move the arm could
be partially extended *and* still sliding sideways — sweeping through
any pod at that intermediate height and position. Fixed by sequencing
the move into phases (see "The robot" section above) and by giving the
retracted position real clearance above the tallest trough (measured
directly rather than assumed — see `RAIL_CLEARANCE_M`). If you resize
the farm (more layers, taller troughs, etc.), re-check that the
retracted camera height still clears the new top trough surface with
margin; the numbers don't auto-recompute a safety margin, only the
joint targets.

**The poles, overhead beams, trough markers, and trough brackets are
all built without collision shapes** (`baseCollisionShapeIndex=-1`,
visual only). Early on, the poles had real collision geometry, and the
carriage physically jammed against them while trying to slide
laterally past — a pole sits exactly on the path the carriage has to
travel, so with collision enabled the "robot" was rigid-body-blocked
by its own farm structure and would sit stuck partway to the next
pole. Scene furniture is visual-only for that reason; if you later
want actual collision-aware path planning, that needs a real
collision-free path rather than just re-enabling collision on the
current geometry.

## Swapping in real data / real models

Nothing here needs to be restructured later — that's the point of the
interface contract:

1. **Real ground-truth images**: replace the files in `sample_images/`
   (or just repoint `pod["image_path"]` in `pod_registry.py`) with real
   Roboflow/Mendeley crops matched to each pod's assigned condition.
2. **Real perception**: write `real_perception.classify_pod(pod_id,
   crop_type, image)` returning the same
   `{growth_stage, disease_flag, confidence}` shape, and swap the
   import in `main.py` from `perception_stub` to `real_perception`.
   Everything downstream (decision agent, robot, logger) is unchanged.
3. **AWS layer**: `run_logger.py`'s per-row dict is already shaped to
   drop straight into a DynamoDB per-pod health history table; the CSV
   is ready for the report's evaluation section as-is.

## Known simplifications (documented, not hidden)

- Pod "images" are synthetic placeholders (flat color + mottling +
  text label), not real plant/mushroom photos — real dataset images
  don't exist for this sandbox run. This mirrors the documented
  substitution from the handoff doc (physical pods don't exist, so a
  real dataset image stands in for a live camera frame); the
  placeholders here stand in one level further up, for the real
  dataset images themselves.
- The pole/beam/trough geometry is a schematic visual, not a to-scale
  CAD model of a real pod farm. Aisle *width* and rail *clearance* are
  sized to real-world-plausible values as described above; other
  dimensions (pole/beam thickness, trough size) are reasonable but not
  sourced from a specific real farm.
- The connecting rod is now recreated every simulation step during
  motion (not just at rest), which is correct but means `--gui` runs
  do a lot of remove/recreate calls — a full 72-pod headless run takes
  about 40s. Fine for this demo's scale; if the farm grows much larger
  or needs to run faster, throttling the rod update to every few steps
  instead of every step would trade a little visual smoothness for
  speed.
- Decision thresholds in `decision_agent.py` are simple, defensible
  starting points (confidence floor 0.6, reschedule floor 0.7,
  crop-specific "near harvest" stage sets) — tune these once real
  model confidence calibration is known.

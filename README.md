# OpenCV-Farming-Agent — PyBullet Simulation (Step 4)

Standalone PyBullet loop for the pod-farm inspection robot, built
against a **stub** of the `classify_pod()` interface so this doesn't
have to wait on the trained perception models. This implements items
1–7 from the handoff doc's "What This Chat Should Help With" section,
laid out around the pole/trough station design in
`Opencv_agent_pods.svg`: each station is a vertical pole with
elongated rectangular trough pods stacked at several heights, mirrored
on the pole's front and back face.

## Farm layout

- **3 stations** (poles) along the rail, **4 trough layers** per pole,
  **2 sides** per layer (front/back) → **24 pods** total.
- Each pod is an elongated box (`TROUGH_HALF_EXTENTS_M` in
  `pod_registry.py`) matching the rectangular trough shape in the SVG,
  not a plain cube.
- Change `STATIONS` / `LAYERS_PER_STATION` / spacing constants in
  `pod_registry.py` to resize the farm — nothing else needs to change.

## Files

| File | What it does |
|---|---|
| `pod_registry.py` | Farm layout: stations × layers × sides, each with a 3D position, crop type, and a ground-truth (growth_stage, diseased) condition. `pod_visit_order()` returns a serpentine traversal (across poles, bottom-to-top per pole, front-then-back per layer). |
| `generate_placeholder_images.py` | Generates labeled placeholder PNGs into `sample_images/`. **Not real data** — see "Swapping in real data" below. |
| `perception_stub.py` | `classify_pod(pod_id, crop_type, image)` — the locked-in interface contract. Two modes: pure-random stub, and a "noisy oracle" that mostly echoes ground truth for cleaner demo runs. |
| `decision_agent.py` | Rule-based `decide()`: low confidence → `schedule_monitoring`; disease detected → `flag_for_treatment`; healthy + near-harvest stage → `schedule_monitoring`; otherwise → `log_and_continue`. Also defines per-action robot dwell times. |
| `gantry_robot.py` | Loads `urdf/gantry.urdf`, builds the poles + trough markers (visual-only — see "Known gotcha" below), and drives all three joints to each pod's (x, y, z) with position control until settled. Recolors each trough by decision (green/yellow/red) after every visit. |
| `urdf/gantry.urdf` | **Three**-prismatic-joint gantry: `lift_joint` (vertical, between layers), `carriage_joint` (lateral, between poles), `reach_joint` (short front/back throw to switch which face of the pole the camera is looking at), plus a camera marker link. |
| `run_logger.py` | Writes one CSV + JSON row per pod visit: timestamp, pod, station, layer, side, crop, growth_stage, disease_flag, confidence, action, reason, ground truth. This is the artifact for the Agentic Vision Award and later feeds DynamoDB. |
| `main.py` | Orchestrates the full loop and CLI. |

## Running it

```bash
pip install pybullet pillow numpy

# headless (no display needed) — full 24-pod farm, pure-random stub
python3 main.py

# GUI window (needs a local display), noisy-oracle stub for a cleaner
# demo trace, slowed down so it's watchable
python3 main.py --gui --oracle --sleep 0.004

# quick smoke test on just the first 6 pods
python3 main.py --pods 6 --oracle --seed 42
```

Each run prints a live `[pod] saw X -> decided Y` trace and writes
`logs/run_<timestamp>.csv` / `.json`.

**Note on this sandbox:** this container has no display, so everything
was tested with `--gui` **off** (`p.DIRECT` mode) — physics, all three
joints' position control, classification, decision logic, marker
recoloring, and logging all verified working on the full 24-pod farm.
Run with `--gui` on your machine to see the actual window; nothing
else changes. Two static camera renders (`scene_snapshot*.png`, not
included in this delivery) were also used during development to
visually confirm the pole/trough layout matches the SVG.

## What the GUI shows

- 3 grey poles (metal supports) with a top rail connecting them.
- Elongated trough-shaped pods at 4 heights on each pole, on both the
  front and back face — colored by crop until visited, then recolored
  by the decision made there: 🟢 `log_and_continue` · 🟡
  `schedule_monitoring` · 🔴 `flag_for_treatment`.
- A blue **lift** block sliding vertically, an orange **carriage**
  block sliding laterally between poles, a small teal **reach** block
  making the short front/back move to face a trough, and a dark
  cylinder camera marker.
- The robot visibly dwells longer at flagged pods than at healthy ones
  (`decision_agent.ACTION_DWELL_SECONDS`) — the decision changes the
  robot's physical behavior, not just a printed label.

## Known gotcha (already fixed, worth knowing about)

The pole and trough markers are built **without collision shapes**
(`baseCollisionShapeIndex=-1`, visual only). Early on these had real
collision geometry, and the carriage physically jammed against the
pole cylinders while trying to slide laterally past them — a station's
pole sits exactly on the path the carriage has to travel along X, so
with collision enabled the "robot" was rigid-body-blocked by its own
farm structure and would sit stuck at ~10% of the way to the next
pole, never reaching the target within the settle-step cap. Scene
furniture is visual-only for that reason; if you later want actual
collision-aware path planning, that needs a real collision-free path
(e.g. route the carriage at a Y offset clear of the poles, or model
the poles as offset from the carriage's travel line) rather than just
re-enabling collision on the current geometry.

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
- The pole/trough geometry is a schematic visual (cylinders + boxes),
  not a to-scale CAD model of a real pod farm.
- Decision thresholds in `decision_agent.py` are simple, defensible
  starting points (confidence floor 0.6, reschedule floor 0.7,
  crop-specific "near harvest" stage sets) — tune these once real
  model confidence calibration is known.

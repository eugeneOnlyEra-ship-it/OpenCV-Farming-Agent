# OpenCV-Farming-Agent — PyBullet Simulation (Step 4)

Standalone PyBullet loop for the pod-farm inspection robot, built
against a **stub** of the `classify_pod()` interface so this doesn't
have to wait on the trained perception models. This directly
implements items 1–7 from the handoff doc's "What This Chat Should
Help With" section.

## Files

| File | What it does |
|---|---|
| `pod_registry.py` | Farm layout: 3 layers × 4 slots = 12 pods, each with a 3D position, crop type, and a ground-truth (growth_stage, diseased) condition. `pod_visit_order()` returns a serpentine traversal. |
| `generate_placeholder_images.py` | Generates 12 labeled placeholder PNGs (one per unique crop/stage/disease combo) into `sample_images/`. **Not real data** — see "Swapping in real data" below. |
| `perception_stub.py` | `classify_pod(pod_id, crop_type, image)` — the locked-in interface contract. Two modes: pure-random stub, and a "noisy oracle" that mostly echoes ground truth for cleaner demo runs. |
| `decision_agent.py` | Rule-based `decide()`: low confidence → `schedule_monitoring`; disease detected → `flag_for_treatment`; healthy + near-harvest stage → `schedule_monitoring`; otherwise → `log_and_continue`. Also defines per-action robot dwell times. |
| `gantry_robot.py` | Loads `urdf/gantry.urdf`, builds the static rail + colored pod markers, and drives the two prismatic joints to each pod's (x, z) with position control until settled. Recolors each pod marker by decision (green/yellow/red) after every visit. |
| `urdf/gantry.urdf` | Two-prismatic-joint gantry: `lift_joint` (vertical, between layers) + `carriage_joint` (lateral, between slots), plus a small camera marker link. |
| `run_logger.py` | Writes one CSV + JSON row per pod visit: timestamp, pod, crop, growth_stage, disease_flag, confidence, action, reason, and ground truth for comparison. This is the artifact for the Agentic Vision Award and later feeds DynamoDB. |
| `main.py` | Orchestrates the full loop and CLI. |

## Running it

```bash
pip install pybullet pillow numpy

# headless (no display needed) — full 12-pod farm, pure-random stub
python3 main.py

# GUI window (needs a local display), noisy-oracle stub for a cleaner
# demo trace, slowed down so it's watchable
python3 main.py --gui --oracle --sleep 0.004

# quick smoke test on just the first 4 pods
python3 main.py --pods 4 --oracle --seed 42
```

Each run prints a live `[pod] saw X -> decided Y` trace and writes
`logs/run_<timestamp>.csv` / `.json`.

**Note on this sandbox:** this container has no display, so everything
above was tested with `--gui` **off** (`p.DIRECT` mode) — physics,
joint control, classification, decision logic, marker recoloring, and
logging all verified working. Run with `--gui` on your machine to see
the actual window; nothing else changes.

## What the GUI shows

- A grey rail column + top rail (static, visual only).
- A blue **lift** block sliding vertically, an orange **carriage**
  block sliding laterally on top of it, and a small dark cylinder
  camera marker — this is the gantry/lift mechanism from the handoff
  doc's Section 6.
- 12 small cubes at the registered pod positions, colored by crop
  until visited, then recolored by the decision made there:
  🟢 log_and_continue · 🟡 schedule_monitoring · 🔴 flag_for_treatment.
- The robot visibly dwells longer at flagged pods than at healthy ones
  (`decision_agent.ACTION_DWELL_SECONDS`) — the decision changes the
  robot's physical behavior, not just a printed label.

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
- The rail/pod-shelf geometry is a schematic visual (boxes), not a
  to-scale CAD model of a real pod farm.
- Decision thresholds in `decision_agent.py` are simple, defensible
  starting points (confidence floor 0.6, reschedule floor 0.7,
  crop-specific "near harvest" stage sets) — tune these once real
  model confidence calibration is known.

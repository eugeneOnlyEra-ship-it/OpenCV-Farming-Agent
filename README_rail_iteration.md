# Agent Rail & Station Structure — Arm Set Aside

This iteration is scoped down deliberately: just the station/pole
structure and the agent's rail-level movement, confirmed against your
sketch, with the hoist/arm mechanism set aside (not deleted) for now.

## What's new

- **`pod_registry.station_visit_order()`** — a station-level (not
  trough-level) visit order: park at the top dock, sweep each row's 3
  poles, park at the bottom dock. Separate from the existing
  `pod_visit_order()` (trough/layer level), which is untouched.
- **Dock rails** (`DOCK_SPACING_M`, `TOP_DOCK_Y`, `BOTTOM_DOCK_Y`,
  `DOCK_PARK_X` in `pod_registry.py`) — two rail-only extensions
  beyond row 0 and beyond the last row, where the agent parks before
  starting and after finishing. No stations, no poles of their own —
  cantilevered off the nearest row's two end-pole tops via short
  brackets, confirmed against your sketch (the unlabeled bottom edge
  is a mirrored extension, and both are pure rail — no arm work
  happens there).
- **`urdf/agent_trolley.urdf`** + **`agent_robot.py`** — a two-axis
  trolley (X across columns, Y across rows *and* out to the docks) with
  no hoist, mast, or camera. Simpler than the arm version's five-phase
  move: with nothing being reached into from above, there's no
  pod-clipping concern to route around, so it's a direct two-joint move
  to each station. The scene includes the actual crop-colored trough
  pods and their pole-attachment brackets (straight from
  `pod_registry.PODS`, unchanged data) — nothing services them yet
  without the arm, but the farm layout is visually complete rather than
  showing bare pole markers.
- **`agent_rail_demo.py`** — standalone runner: dock-top → all 9
  stations, row by row → dock-bottom. No perception, no decisions —
  just confirms the structure and travel order.

## What's unchanged

`pod_registry.py`'s existing trough/layer data (`PODS`,
`pod_visit_order()`), and every file from the arm-based build
(`gantry_robot.py`, `urdf/gantry.urdf`, `perception_stub.py`,
`decision_agent.py`, `run_logger.py`, `main.py`) are untouched. The
plan is to reattach that arm mechanism to this confirmed rail
structure once you're happy with it, not to throw the arm work away.

## Running it

```bash
pip install pybullet pillow numpy

python3 agent_rail_demo.py                 # headless, full run
python3 agent_rail_demo.py --gui --sleep 0.004   # GUI, slowed down
```

Tested headlessly in this sandbox: all 11 stops (2 docks + 9 stations)
settle cleanly, none hit the step cap.

## One assumption worth flagging

Your sketch's outer (left/right) columns have a continuous vertical
line running through all three rows; the middle column's dots don't
show their own vertical line. I kept the existing full 3×3 pole grid
(all 9 positions get their own vertical pole) rather than reading that
as "the middle column has no dedicated pole" — mainly because it
doesn't affect the movement logic you asked me to focus on this round,
and I didn't want to guess at a structural change you didn't
explicitly ask for. Flag it if you did intend the middle poles to go
away (e.g., relying only on the beam's own rigidity between the two
outer poles) and I'll rework the pole grid accordingly.

"""
pod_registry.py

Defines the pod farm layout as a ROWS x COLUMNS grid of *stations* --
each station is a single vertical metal pole with elongated
rectangular trough pods stacked at several heights, mirrored on the
pole's front (+ into its aisle) and back (- into its aisle) face. This
matches the layout sketched in Opencv_agent_pods.svg (one rectangular
trough shape, repeated at N layers, on 2 sides of a station), scaled
up to a full grid of stations.

Grid layout:
  - COLUMNS poles per row (X axis, meters) -- the direction the
    gantry's carriage travels along within a row.
  - ROWS of stations (Y axis, meters) -- each row sits in its own
    aisle; the gap between rows is sized to be an actual human-
    walkable aisle, not just clearance between pole centers.
  - LAYERS_PER_STATION trough heights per pole (Z axis, meters).
  - SIDES = ("front", "back") -> the two faces of each pole, offset a
    little either side of the row's centerline.

Spacing rationale (this is the part worth tuning for a real farm):
  - COLUMN_SPACING_M is pole-to-pole spacing *within* a row -- the
    gantry rides a rail along this direction, so it mainly needs
    clearance for the trough width (TROUGH_HALF_EXTENTS_M) plus a
    little margin for manual access to a single pole, not a full
    walking aisle.
  - AISLE_CLEAR_WIDTH_M is the actual open, walkable gap between one
    row's front-facing troughs and the next row's back-facing troughs
    -- i.e. the space a person (or the robot) would walk down between
    two rows. 1.0m sits in the standard range for a single-person
    working aisle (roughly 0.9-1.2m is typical guidance for
    warehouse/greenhouse pedestrian aisles); ROW_SPACING_M is then
    *derived* from that clear width plus the trough overhang on both
    sides, so the aisle stays exactly 1.0m clear regardless of how
    SIDE_OFFSET_M is tuned.

Design note (documented substitution, see handoff doc Section 6):
Physical pods/plants don't exist, so at each stop the simulated robot
loads a real dataset image pre-matched to a known ground-truth
condition instead of rendering organic matter. In this standalone
build that image pool is a small set of placeholder images generated
by generate_placeholder_images.py; swap PODS[i]["image_path"] to point
at real Roboflow/Mendeley crops once they're wired in -- no other code
needs to change, since the rest of the pipeline only cares about the
path.
"""

import itertools
import os

CROPS = ["cabbage", "lettuce", "mushroom"]

# Growth stages per crop (must match the fixed Roboflow class lists)
GROWTH_STAGES = {
    "cabbage": ["S2_3", "S4_5", "S6_7", "S8", "S9"],
    "lettuce": ["Early Growth", "Leafy growth", "Head formation", "Harvest stage"],
    "mushroom": ["Juvenile", "Intermediate", "Harvest"],
}

ROWS = 3                     # rows of stations, each in its own walkable aisle
COLUMNS = 3                  # poles per row
LAYERS_PER_STATION = 4       # stacked trough heights per pole (matches the SVG: 4 boxes)
SIDES = ("front", "back")    # both faces of each pole

COLUMN_SPACING_M = 0.75    # pole-to-pole along a row -- trough width (0.44m) + service clearance
LAYER_SPACING_M = 0.42     # vertical gap between trough heights
BASE_HEIGHT_M = 0.28       # height of the lowest trough off the ground
SIDE_OFFSET_M = 0.16       # how far a trough sticks out from the pole centerline (+/-)

AISLE_CLEAR_WIDTH_M = 1.0  # actual open walking gap between facing trough rows
ROW_SPACING_M = AISLE_CLEAR_WIDTH_M + 2 * SIDE_OFFSET_M  # pole-centerline-to-pole-centerline

TROUGH_HALF_EXTENTS_M = (0.22, 0.08, 0.045)  # (x, y, z) -- long + shallow + thin, per the SVG shape

# --- dock rails (added per the station/pole sketch) ---
# Two rail-only extensions, one beyond row 0 and one beyond the last row:
# where the agent is parked before work starts and where it parks again
# once finished. Pure rail -- no poles of their own, no stations mounted
# on them, cantilevered ("C" shaped) off the nearest row's end poles
# rather than needing new dedicated vertical supports. Kept deliberately
# closer than a full row-to-row aisle gap, since this is a parking
# extension the agent's own trolley footprint needs to clear, not a
# second human-walkable aisle.
DOCK_SPACING_M = 0.5
TOP_DOCK_Y = -DOCK_SPACING_M
BOTTOM_DOCK_Y = (ROWS - 1) * ROW_SPACING_M + DOCK_SPACING_M
DOCK_PARK_X = ((COLUMNS - 1) * COLUMN_SPACING_M) / 2  # centered on the rail

IMAGE_DIR = os.path.join(os.path.dirname(__file__), "sample_images")


def station_visit_order():
    """
    Station-level (not trough-level) visit order for the rail-only
    agent, per the confirmed sketch: park at the top dock, sweep each
    row's 3 poles left-to-right (alternating direction row to row so
    the agent doesn't waste travel snapping back to column 0 each
    time), then park at the bottom dock. No layers/sides/crops here --
    this is purely about which horizontal pole and which pole position
    on it, for reintroducing the arm/perception logic later without
    having to redo the rail-travel logic.

    Each stop is a dict: {label, x, y, is_dock, row, col}. row/col are
    None for dock stops.
    """
    stops = [{
        "label": "dock-top", "x": DOCK_PARK_X, "y": TOP_DOCK_Y,
        "is_dock": True, "row": None, "col": None,
    }]

    for row in range(ROWS):
        col_range = range(COLUMNS) if row % 2 == 0 else range(COLUMNS - 1, -1, -1)
        row_y = row * ROW_SPACING_M
        for col in col_range:
            stops.append({
                "label": f"R{row}C{col}", "x": col * COLUMN_SPACING_M, "y": row_y,
                "is_dock": False, "row": row, "col": col,
            })

    stops.append({
        "label": "dock-bottom", "x": DOCK_PARK_X, "y": BOTTOM_DOCK_Y,
        "is_dock": True, "row": None, "col": None,
    })
    return stops


def _ground_truth_cycle():
    """
    Deterministic, repeatable assignment of (growth_stage, disease) ground
    truth to pods, so a demo run is reproducible and includes a mix of
    healthy/diseased and early/late growth stages -- useful for showing
    the decision agent actually branching, not just logging one outcome
    over and over.
    """
    pattern = [
        (0, False),   # earliest stage, healthy
        (1, False),
        (2, True),    # mid stage, diseased -> should trigger "flag for treatment"
        (-1, False),  # last (harvest) stage, healthy -> should trigger tighter monitoring
    ]
    return itertools.cycle(pattern)


def build_pod_registry():
    """Returns a list of pod dicts describing the full farm layout:
    ROWS x COLUMNS poles x LAYERS_PER_STATION heights x 2 sides."""
    pods = []
    crop_cycle = itertools.cycle(CROPS)
    pod_index = 0

    for row in range(ROWS):
        row_y = row * ROW_SPACING_M

        for col in range(COLUMNS):
            station_x = col * COLUMN_SPACING_M
            gt_cycle = _ground_truth_cycle()

            for layer in range(LAYERS_PER_STATION):
                layer_z = BASE_HEIGHT_M + layer * LAYER_SPACING_M

                for side in SIDES:
                    crop = next(crop_cycle)
                    stages = GROWTH_STAGES[crop]
                    stage_offset, diseased = next(gt_cycle)
                    stage = stages[stage_offset if stage_offset >= 0 else len(stages) - 1]

                    side_y = row_y + (SIDE_OFFSET_M if side == "front" else -SIDE_OFFSET_M)
                    pod_id = f"R{row}C{col}-L{layer}-{side[0].upper()}"
                    image_name = (
                        f"{crop}_{stage}_{'diseased' if diseased else 'healthy'}.png"
                    ).replace(" ", "_")

                    pods.append({
                        "pod_id": pod_id,
                        "index": pod_index,
                        "row": row,
                        "col": col,
                        "layer": layer,
                        "side": side,
                        "crop_type": crop,
                        "position_m": {"x": station_x, "y": side_y, "z": layer_z},
                        "ground_truth": {"growth_stage": stage, "diseased": diseased},
                        "image_path": os.path.join(IMAGE_DIR, image_name),
                    })
                    pod_index += 1

    return pods


PODS = build_pod_registry()


def get_pod(pod_id):
    for p in PODS:
        if p["pod_id"] == pod_id:
            return p
    raise KeyError(f"Unknown pod_id: {pod_id}")


def station_key(pod):
    return (pod["row"], pod["col"])


def pod_visit_order():
    """
    Boustrophedon (serpentine) traversal across the whole grid:
      - within a row, sweep columns left-to-right, then right-to-left
        on the next row (minimizes X travel between consecutive poles)
      - within each pole, alternate bottom-to-top / top-to-bottom on
        consecutive poles (minimizes Z travel, same trick as before)
      - front-then-back at each layer (a Y-axis move rather than a
        full carriage/lift re-position) before climbing to the next layer
    """
    order = []
    station_counter = 0

    for row in range(ROWS):
        col_range = range(COLUMNS) if row % 2 == 0 else range(COLUMNS - 1, -1, -1)

        for col in col_range:
            station_pods = [p for p in PODS if p["row"] == row and p["col"] == col]
            station_pods.sort(key=lambda p: (p["layer"], 0 if p["side"] == "front" else 1))

            if station_counter % 2 == 1:
                layers_desc = sorted(set(p["layer"] for p in station_pods), reverse=True)
                station_pods.sort(key=lambda p: (
                    layers_desc.index(p["layer"]), 0 if p["side"] == "front" else 1
                ))

            order.extend(station_pods)
            station_counter += 1

    return order


if __name__ == "__main__":
    print(f"{ROWS} rows x {COLUMNS} columns x {LAYERS_PER_STATION} layers x 2 sides "
          f"= {len(PODS)} pods")
    print(f"column spacing: {COLUMN_SPACING_M}m | row spacing: {ROW_SPACING_M}m "
          f"(aisle clear width: {AISLE_CLEAR_WIDTH_M}m)")
    for p in pod_visit_order()[:16]:
        print(p["pod_id"], p["crop_type"], p["ground_truth"], p["position_m"])

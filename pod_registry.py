"""
pod_registry.py

Defines the pod farm layout as a row of *stations* -- each station is a
single vertical metal pole with elongated rectangular trough pods
stacked at several heights, mirrored on the pole's front (+Y) and back
(-Y) face. This matches the layout sketched in Opencv_agent_pods.svg:
one rectangular trough shape, repeated at N layers, on 2 sides of a
station.

Layout convention:
  - STATIONS poles along the rail (X axis, meters)
  - LAYERS_PER_STATION trough heights per pole (Z axis, meters)
  - SIDES = ("front", "back") -> +Y / -Y offset from the pole's centerline

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

STATIONS = 3                 # number of poles along the rail
LAYERS_PER_STATION = 4       # stacked trough heights per pole (matches the SVG: 4 boxes)
SIDES = ("front", "back")    # both faces of each pole

STATION_SPACING_M = 0.60         # lateral gap between poles
LAYER_SPACING_M = 0.42           # vertical gap between trough heights
BASE_HEIGHT_M = 0.28              # height of the lowest trough off the ground
SIDE_OFFSET_M = 0.16              # how far a trough sticks out from the pole centerline (+/-)

TROUGH_HALF_EXTENTS_M = (0.22, 0.08, 0.045)  # (x, y, z) -- long + shallow + thin, per the SVG shape

IMAGE_DIR = os.path.join(os.path.dirname(__file__), "sample_images")


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
    STATIONS poles x LAYERS_PER_STATION heights x 2 sides."""
    pods = []
    crop_cycle = itertools.cycle(CROPS)
    pod_index = 0

    for station in range(STATIONS):
        station_x = station * STATION_SPACING_M
        gt_cycle = _ground_truth_cycle()

        for layer in range(LAYERS_PER_STATION):
            layer_z = BASE_HEIGHT_M + layer * LAYER_SPACING_M

            for side in SIDES:
                crop = next(crop_cycle)
                stages = GROWTH_STAGES[crop]
                stage_offset, diseased = next(gt_cycle)
                stage = stages[stage_offset if stage_offset >= 0 else len(stages) - 1]

                side_y = SIDE_OFFSET_M if side == "front" else -SIDE_OFFSET_M
                pod_id = f"St{station}-L{layer}-{side[0].upper()}"
                image_name = (
                    f"{crop}_{stage}_{'diseased' if diseased else 'healthy'}.png"
                ).replace(" ", "_")

                pods.append({
                    "pod_id": pod_id,
                    "index": pod_index,
                    "station": station,
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


def pod_visit_order():
    """
    Serpentine traversal across stations (minimizes lateral travel
    between consecutive poles), bottom-to-top within each pole, and
    front-then-back at each layer (a short reach-axis move rather than
    a full carriage/lift re-position) before climbing to the next layer.
    """
    order = []
    for station in range(STATIONS):
        station_pods = [p for p in PODS if p["station"] == station]
        station_pods.sort(key=lambda p: (p["layer"], 0 if p["side"] == "front" else 1))
        if station % 2 == 1:
            # reverse layer order (not side order) so the lift still moves
            # top-to-bottom smoothly instead of jumping
            layers_desc = sorted(set(p["layer"] for p in station_pods), reverse=True)
            station_pods.sort(key=lambda p: (
                layers_desc.index(p["layer"]), 0 if p["side"] == "front" else 1
            ))
        order.extend(station_pods)
    return order


if __name__ == "__main__":
    for p in pod_visit_order():
        print(p["pod_id"], p["crop_type"], p["ground_truth"], p["position_m"])

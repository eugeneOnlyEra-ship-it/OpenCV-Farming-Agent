"""
gantry_robot.py

Loads the three-joint gantry URDF and provides:
  - move_to(pod): drives lift_joint (Z), carriage_joint (X), and
    row_joint (Y) toward a pod's target position using position
    control, stepping the simulation until all three settle within
    tolerance (i.e. "stop precisely at the pod", correct row/column/
    layer/face).
  - a static scene of a ROWS x COLUMNS grid of vertical poles, spaced
    with a realistic human-walkable aisle between rows, with a
    trough-shaped marker at every (row, col, layer, side) position,
    colored by crop until visited, then recolored by the decision made
    there (green=log_and_continue, yellow=schedule_monitoring,
    red=flag_for_treatment) so the GUI demo shows the decision loop's
    effect at a glance.

Coordinate mapping: pod_registry gives (x, y, z) in meters --
x = which column (pole along a row), y = which row's aisle, offset a
little further by +front/-back on the pole face, z = trough height.
The three joints map directly: carriage_joint -> x, row_joint -> y,
lift_joint -> z.
"""

import pybullet as p
import pybullet_data
import os
import time

from pod_registry import (
    ROWS, COLUMNS, LAYERS_PER_STATION, COLUMN_SPACING_M, ROW_SPACING_M,
    LAYER_SPACING_M, BASE_HEIGHT_M, TROUGH_HALF_EXTENTS_M,
)

LIFT_JOINT = 0
CARRIAGE_JOINT = 1
ROW_JOINT = 2

POSITION_TOLERANCE_M = 0.01
# ~2s at 240Hz was fine for a single row; the grid now has row-to-row
# jumps up to ROWS*ROW_SPACING_M (~5.3m) and column jumps up to
# COLUMNS*COLUMN_SPACING_M (~2.25m) in the worst case (e.g. the very
# first move, or a big serpentine turnaround), so give settling more
# headroom -- ~6s at 240Hz, comfortably more than the ~9s a full-span
# move would need at 0.6 m/s.
MAX_SETTLE_STEPS = 1400

POLE_RADIUS_M = 0.035

DECISION_COLOR = {
    "log_and_continue": (0.20, 0.75, 0.25, 1.0),     # green
    "schedule_monitoring": (0.95, 0.80, 0.10, 1.0),  # yellow
    "flag_for_treatment": (0.85, 0.15, 0.15, 1.0),   # red
    None: (0.55, 0.55, 0.55, 1.0),                   # unvisited = grey
}

CROP_MARKER_COLOR = {
    "cabbage": (0.34, 0.60, 0.35, 1.0),
    "lettuce": (0.55, 0.75, 0.35, 1.0),
    "mushroom": (0.77, 0.70, 0.58, 1.0),
}


class GantryRobot:
    def __init__(self, pods, gui=True):
        self.pods = pods
        self.gui = gui
        self.pod_marker_ids = {}

        connection_mode = p.GUI if gui else p.DIRECT
        self.client = p.connect(connection_mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(0)
        p.setTimeStep(1.0 / 240.0)

        p.loadURDF("plane.urdf")

        urdf_path = os.path.join(os.path.dirname(__file__), "urdf", "gantry.urdf")
        # start the carriage off to the side of the first pole so the
        # opening move into R0C0 is visible rather than starting inside it
        self.robot_id = p.loadURDF(
            urdf_path, basePosition=[-0.35, 0, 0], useFixedBase=True
        )

        self._build_poles_and_troughs()

        if gui:
            farm_span_x = (COLUMNS - 1) * COLUMN_SPACING_M
            farm_span_y = (ROWS - 1) * ROW_SPACING_M
            farm_diag = (farm_span_x ** 2 + farm_span_y ** 2) ** 0.5
            p.resetDebugVisualizerCamera(
                cameraDistance=max(2.6, farm_diag * 0.9), cameraYaw=35, cameraPitch=-35,
                cameraTargetPosition=[farm_span_x / 2, farm_span_y / 2, 0.9],
            )

    # ---------- scene construction ----------

    def _build_poles_and_troughs(self):
        """A ROWS x COLUMNS grid of vertical poles (each row its own
        aisle, spaced per pod_registry.ROW_SPACING_M / COLUMN_SPACING_M),
        plus a trough marker at every (row, col, layer, side) position
        from the pod registry. These are visual-only (no collision
        shape) -- they're scene furniture the gantry travels past, not
        obstacles it should physically collide with; giving them
        collision geometry made the carriage jam against the poles
        while sliding laterally past them."""
        pole_top_z = BASE_HEIGHT_M + (LAYERS_PER_STATION - 1) * LAYER_SPACING_M + 0.35
        pole_grey = (0.42, 0.42, 0.45, 1.0)

        for row in range(ROWS):
            row_y = row * ROW_SPACING_M
            for col in range(COLUMNS):
                station_x = col * COLUMN_SPACING_M
                self._add_static_cylinder(
                    radius=POLE_RADIUS_M, height=pole_top_z,
                    position=[station_x, row_y, pole_top_z / 2],
                    color=pole_grey,
                )

        half = list(TROUGH_HALF_EXTENTS_M)
        for pod in self.pods:
            pos = pod["position_m"]
            color = CROP_MARKER_COLOR[pod["crop_type"]]
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
            body_id = p.createMultiBody(
                baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vis,
                basePosition=[pos["x"], pos["y"], pos["z"]],
            )
            self.pod_marker_ids[pod["pod_id"]] = body_id

    def _add_static_cylinder(self, radius, height, position, color):
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)
        return p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vis,
            basePosition=position,
        )

    def mark_pod_decision(self, pod_id, action):
        color = DECISION_COLOR.get(action, DECISION_COLOR[None])
        p.changeVisualShape(self.pod_marker_ids[pod_id], -1, rgbaColor=color)

    # ---------- movement ----------

    def move_to(self, pod, step_sleep=0.0):
        """
        Drives lift_joint to the pod's z, carriage_joint to the pod's x,
        and row_joint to the pod's y using POSITION_CONTROL, stepping
        the simulation until all three joints are within
        POSITION_TOLERANCE_M of target (or the step cap is hit, so a bad
        target can't hang the run forever). Returns steps taken.
        """
        target_x = pod["position_m"]["x"]
        target_y = pod["position_m"]["y"]
        target_z = pod["position_m"]["z"]

        p.setJointMotorControl2(
            self.robot_id, LIFT_JOINT, p.POSITION_CONTROL,
            targetPosition=target_z, force=200, maxVelocity=0.6,
        )
        p.setJointMotorControl2(
            self.robot_id, CARRIAGE_JOINT, p.POSITION_CONTROL,
            targetPosition=target_x, force=200, maxVelocity=0.6,
        )
        p.setJointMotorControl2(
            self.robot_id, ROW_JOINT, p.POSITION_CONTROL,
            targetPosition=target_y, force=200, maxVelocity=0.6,
        )

        steps = 0
        while steps < MAX_SETTLE_STEPS:
            p.stepSimulation()
            steps += 1
            if step_sleep:
                time.sleep(step_sleep)

            lift_pos = p.getJointState(self.robot_id, LIFT_JOINT)[0]
            carriage_pos = p.getJointState(self.robot_id, CARRIAGE_JOINT)[0]
            row_pos = p.getJointState(self.robot_id, ROW_JOINT)[0]
            if (abs(lift_pos - target_z) < POSITION_TOLERANCE_M
                    and abs(carriage_pos - target_x) < POSITION_TOLERANCE_M
                    and abs(row_pos - target_y) < POSITION_TOLERANCE_M):
                break

        return steps

    def dwell(self, seconds):
        """Hold position for `seconds` of sim time -- used to make the
        decision agent's chosen action visibly change how long the robot
        lingers at a pod (see decision_agent.ACTION_DWELL_SECONDS)."""
        n_steps = max(1, int(seconds * 240))
        for _ in range(n_steps):
            p.stepSimulation()

    def disconnect(self):
        p.disconnect(self.client)

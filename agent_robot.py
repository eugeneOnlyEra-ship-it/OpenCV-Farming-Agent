"""
agent_robot.py

Arm-free iteration: a two-axis trolley (see urdf/agent_trolley.urdf)
riding the same overhead rail structure as before, PLUS two new
dock-rail extensions -- one beyond row 0, one beyond the last row --
where the agent parks before starting and after finishing. Pure rail,
no stations, cantilevered off the nearest row's end poles rather than
needing their own dedicated vertical poles (per the confirmed sketch).

The hoist/mast/camera mechanism from gantry_robot.py is intentionally
left out here, not deleted -- this module is about getting the
station/pole structure and the agent's rail-level movement right
first; gantry_robot.py's arm design is expected to be reattached to
this trolley once that's confirmed.
"""

import pybullet as p
import pybullet_data
import os
import time

from pod_registry import (
    ROWS, COLUMNS, LAYERS_PER_STATION, COLUMN_SPACING_M, ROW_SPACING_M,
    LAYER_SPACING_M, BASE_HEIGHT_M, DOCK_SPACING_M, TOP_DOCK_Y, BOTTOM_DOCK_Y,
    PODS, TROUGH_HALF_EXTENTS_M,
)
from gantry_robot import CROP_MARKER_COLOR

CARRIAGE_JOINT = 0
ROW_JOINT = 1

POSITION_TOLERANCE_M = 0.01
MAX_SETTLE_STEPS = 1000

POLE_RADIUS_M = 0.035
BEAM_HALF_THICKNESS_M = 0.03
BRACKET_HALF_THICKNESS_M = 0.02

# Same rail-height logic as the arm version, minus the extra headroom
# that was specifically for hoist clearance -- kept anyway for visual
# continuity with the existing pole grid (and so re-adding the arm
# later doesn't require re-deriving this).
TOP_TROUGH_Z = BASE_HEIGHT_M + (LAYERS_PER_STATION - 1) * LAYER_SPACING_M
RAIL_CLEARANCE_M = 0.75
TOP_RAIL_HEIGHT_M = TOP_TROUGH_Z + RAIL_CLEARANCE_M
POLE_TOP_Z = TOP_RAIL_HEIGHT_M + 0.06

STATION_COLOR = (0.55, 0.55, 0.58, 1.0)
DOCK_COLOR = (0.85, 0.65, 0.15, 1.0)


class AgentRobot:
    def __init__(self, gui=True):
        self.gui = gui
        self.station_marker_ids = {}
        self.pod_marker_ids = {}

        connection_mode = p.GUI if gui else p.DIRECT
        self.client = p.connect(connection_mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(0)
        p.setTimeStep(1.0 / 240.0)

        p.loadURDF("plane.urdf")

        urdf_path = os.path.join(os.path.dirname(__file__), "urdf", "agent_trolley.urdf")
        self.robot_id = p.loadURDF(
            urdf_path, basePosition=[0, TOP_DOCK_Y, TOP_RAIL_HEIGHT_M], useFixedBase=True
        )

        self._build_structure()

        if gui:
            farm_span_x = (COLUMNS - 1) * COLUMN_SPACING_M
            farm_span_y = BOTTOM_DOCK_Y - TOP_DOCK_Y
            p.resetDebugVisualizerCamera(
                cameraDistance=max(2.8, farm_span_y * 0.7), cameraYaw=35, cameraPitch=-30,
                cameraTargetPosition=[farm_span_x / 2, farm_span_y / 2 + TOP_DOCK_Y, TOP_TROUGH_Z / 2],
            )

    # ---------- scene construction ----------

    def _build_structure(self):
        """Poles at every (row, col) -- unchanged from the arm version
        -- plus the row-direction and column-direction overhead beam
        lattice, plus two NEW dock-rail beams (no poles) cantilevered
        off the nearest row's end-pole tops via short brackets. All
        visual-only (no collision), same reasoning as before: this is
        scene furniture the trolley travels past, not something it
        should physically collide with."""
        pole_grey = (0.42, 0.42, 0.45, 1.0)
        beam_grey = (0.5, 0.5, 0.53, 1.0)
        dock_beam_color = (0.55, 0.42, 0.2, 1.0)

        for row in range(ROWS):
            row_y = row * ROW_SPACING_M
            for col in range(COLUMNS):
                station_x = col * COLUMN_SPACING_M
                self._add_static_cylinder(
                    radius=POLE_RADIUS_M, height=POLE_TOP_Z,
                    position=[station_x, row_y, POLE_TOP_Z / 2],
                    color=pole_grey,
                )
                marker = self._add_static_box(
                    half_extents=[0.05, 0.05, 0.03],
                    position=[station_x, row_y, TOP_RAIL_HEIGHT_M],
                    color=STATION_COLOR,
                )
                self.station_marker_ids[f"R{row}C{col}"] = marker

        row_beam_half_len = ((COLUMNS - 1) * COLUMN_SPACING_M) / 2
        for row in range(ROWS):
            row_y = row * ROW_SPACING_M
            self._add_static_box(
                half_extents=[row_beam_half_len, BEAM_HALF_THICKNESS_M, BEAM_HALF_THICKNESS_M],
                position=[row_beam_half_len, row_y, TOP_RAIL_HEIGHT_M],
                color=beam_grey,
            )

        col_beam_half_len = ((ROWS - 1) * ROW_SPACING_M) / 2
        for col in range(COLUMNS):
            station_x = col * COLUMN_SPACING_M
            self._add_static_box(
                half_extents=[BEAM_HALF_THICKNESS_M, col_beam_half_len, BEAM_HALF_THICKNESS_M],
                position=[station_x, col_beam_half_len, TOP_RAIL_HEIGHT_M],
                color=beam_grey,
            )

        # dock rails: horizontal beam spanning the column range, at the
        # dock's Y, cantilevered via short vertical brackets down to
        # the nearest row's two end-pole tops -- NOT full new poles
        self._add_dock_rail(dock_y=TOP_DOCK_Y, nearest_row_y=0.0, color=dock_beam_color)
        self._add_dock_rail(
            dock_y=BOTTOM_DOCK_Y, nearest_row_y=(ROWS - 1) * ROW_SPACING_M, color=dock_beam_color
        )

        # the pods themselves -- dropped when this iteration was scoped
        # down to just the rail/dock structure, added back here so the
        # scene still shows the actual crop trough layout even though
        # nothing services them yet without the arm. Same trough shape
        # and pole-attachment bracket as the arm version, straight from
        # pod_registry.PODS -- that data was never touched by the rail
        # rework, so this is a pure visual reattachment, not new data.
        trough_half = list(TROUGH_HALF_EXTENTS_M)
        bracket_color = (0.3, 0.3, 0.32, 1.0)
        for pod in PODS:
            pos = pod["position_m"]
            color = CROP_MARKER_COLOR[pod["crop_type"]]
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=trough_half, rgbaColor=color)
            body_id = p.createMultiBody(
                baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vis,
                basePosition=[pos["x"], pos["y"], pos["z"]],
            )
            self.pod_marker_ids[pod["pod_id"]] = body_id

            row_y = pod["row"] * ROW_SPACING_M
            bracket_span = pos["y"] - row_y
            bracket_half_y = abs(bracket_span) / 2
            self._add_static_box(
                half_extents=[0.025, max(bracket_half_y, 0.01), 0.018],
                position=[pos["x"], row_y + bracket_span / 2, pos["z"]],
                color=bracket_color,
            )

    def _add_dock_rail(self, dock_y, nearest_row_y, color):
        dock_beam_half_len = ((COLUMNS - 1) * COLUMN_SPACING_M) / 2
        self._add_static_box(
            half_extents=[dock_beam_half_len, BEAM_HALF_THICKNESS_M, BEAM_HALF_THICKNESS_M],
            position=[dock_beam_half_len, dock_y, TOP_RAIL_HEIGHT_M],
            color=color,
        )
        # brackets: short beams connecting the dock rail's two ends
        # down to the end poles' tops (cantilever attachment, no
        # dedicated vertical support of their own)
        bracket_half_len = abs(dock_y - nearest_row_y) / 2
        bracket_mid_y = (dock_y + nearest_row_y) / 2
        for col in (0, COLUMNS - 1):
            station_x = col * COLUMN_SPACING_M
            self._add_static_box(
                half_extents=[BRACKET_HALF_THICKNESS_M, bracket_half_len, BRACKET_HALF_THICKNESS_M],
                position=[station_x, bracket_mid_y, TOP_RAIL_HEIGHT_M],
                color=color,
            )

    def _add_static_box(self, half_extents, position, color):
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
        return p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vis,
            basePosition=position,
        )

    def _add_static_cylinder(self, radius, height, position, color):
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)
        return p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vis,
            basePosition=position,
        )

    def mark_station_visited(self, label):
        if label in self.station_marker_ids:
            p.changeVisualShape(self.station_marker_ids[label], -1, rgbaColor=(0.2, 0.75, 0.25, 1.0))

    # ---------- movement ----------

    def move_to_station(self, station, step_sleep=0.0):
        """Two-joint move: carriage_joint (X) and row_joint (Y) to the
        station's position, simultaneously -- no hoist to sequence
        around, so there's no pod-clipping concern to route past the
        way the arm version needed to. Returns steps taken."""
        target_x = station["x"]
        target_y = station["y"]

        p.setJointMotorControl2(
            self.robot_id, CARRIAGE_JOINT, p.POSITION_CONTROL,
            targetPosition=target_x, force=250, maxVelocity=0.6,
        )
        p.setJointMotorControl2(
            self.robot_id, ROW_JOINT, p.POSITION_CONTROL,
            targetPosition=target_y, force=250, maxVelocity=0.6,
        )

        steps = 0
        while steps < MAX_SETTLE_STEPS:
            p.stepSimulation()
            steps += 1
            if step_sleep:
                time.sleep(step_sleep)
            carriage_pos = p.getJointState(self.robot_id, CARRIAGE_JOINT)[0]
            row_pos = p.getJointState(self.robot_id, ROW_JOINT)[0]
            if (abs(carriage_pos - target_x) < POSITION_TOLERANCE_M
                    and abs(row_pos - target_y) < POSITION_TOLERANCE_M):
                break

        if not station["is_dock"]:
            self.mark_station_visited(station["label"])

        return steps

    def dwell(self, seconds):
        n_steps = max(1, int(seconds * 240))
        for _ in range(n_steps):
            p.stepSimulation()

    def disconnect(self):
        p.disconnect(self.client)

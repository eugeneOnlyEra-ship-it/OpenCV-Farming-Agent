"""
gantry_robot.py

Loads the overhead bridge-crane gantry URDF and provides:
  - move_to(pod): drives carriage_joint (X) and row_joint (Y) so the
    trolley sits directly above the target pod's column/row/face, and
    drives hoist_joint so the arm descends from the trolley down to
    the pod's height. All three settle within tolerance before the
    move is considered done.
  - a static scene: a ROWS x COLUMNS grid of vertical poles, TALL
    ENOUGH to clear the robot's operating envelope, connected at the
    top by a full grid of horizontal beams (both along rows and along
    columns) -- this is the structure the trolley notionally rides on.
    Trough markers sit at every (row, col, layer, side) position.

Why the robot is built this way: an earlier version anchored the
robot's base at ground level and moved the whole assembly up with a
plain vertical joint, which looked like it was hovering in open air
with no visible means of support. This version anchors the base UP AT
the rail height (physically matching the horizontal beams connecting
the pole tops), and reaches individual trough layers with a hoist that
telescopes down from a fixed mast -- so at every point in the
simulation, the robot is either sitting on the rail or hanging from
something that is.

Coordinate mapping: pod_registry gives (x, y, z) in meters --
x = which column, y = which row's aisle (offset a little further by
+front/-back on the pole face), z = trough height. carriage_joint -> x,
row_joint -> y (both operate at the fixed rail height), hoist_joint ->
how far below the rail the target z sits.
"""

import pybullet as p
import pybullet_data
import os
import time

from pod_registry import (
    ROWS, COLUMNS, LAYERS_PER_STATION, COLUMN_SPACING_M, ROW_SPACING_M,
    LAYER_SPACING_M, BASE_HEIGHT_M, TROUGH_HALF_EXTENTS_M,
)

CARRIAGE_JOINT = 0
ROW_JOINT = 1
MAST_LINK = 2
# index 2 is mast_fixed_joint (not controlled, see urdf/gantry.urdf)
HOIST_JOINT = 3
CAMERA_LINK = 4

POSITION_TOLERANCE_M = 0.01
# Grid spans up to a few meters and the hoist can need over a meter of
# travel on top of that, so give settling plenty of headroom -- ~6s at
# 240Hz, comfortably more than a full-span move needs at 0.6 m/s.
MAX_SETTLE_STEPS = 1400

POLE_RADIUS_M = 0.035
BEAM_HALF_THICKNESS_M = 0.03  # half-extent for the overhead connecting beams

# Headroom between the top trough layer and the overhead rail -- this
# is the clearance the robot actually operates in. Must be tall enough
# that the FULLY RETRACTED hoist (mast + hoist folded up, camera at its
# highest point) clears every trough top with real margin -- this is
# what lets the trolley travel horizontally without dragging the
# camera through a pod on the way to another station. At the old
# 0.55m, the retracted camera bottom was actually 1.5cm *below* the
# top trough's surface (measured directly, not assumed) -- so this
# needs real headroom, not just enough to look tall.
RAIL_CLEARANCE_M = 0.75
# Fixed-length mast hanging from the trolley before the telescoping
# hoist begins (must match the -0.175*2 = 0.35m drop in the URDF).
MAST_LENGTH_M = 0.35

TOP_TROUGH_Z = BASE_HEIGHT_M + (LAYERS_PER_STATION - 1) * LAYER_SPACING_M
TOP_RAIL_HEIGHT_M = TOP_TROUGH_Z + RAIL_CLEARANCE_M
POLE_TOP_Z = TOP_RAIL_HEIGHT_M + 0.06  # poles poke slightly above the rail for a clean visual joint

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
        self._hoist_visual_id = None

        connection_mode = p.GUI if gui else p.DIRECT
        self.client = p.connect(connection_mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(0)
        p.setTimeStep(1.0 / 240.0)

        p.loadURDF("plane.urdf")

        urdf_path = os.path.join(os.path.dirname(__file__), "urdf", "gantry.urdf")
        # base sits at world (0, 0, rail height) -- carriage_joint's
        # POSITION_CONTROL targets are pod world x-coordinates directly
        # (see move_to), so the base must contribute zero offset, or
        # every target would land base_x off from the real pod position.
        # (An earlier version anchored the base at x=-0.35 purely for a
        # cosmetic "visible opening move" and never corrected the
        # targets for it -- every pod was actually being visited 0.35m
        # off in X as a result, caught only once the hoist-visual fix
        # below made it worth checking exact positions rather than just
        # "looks roughly right" from a render.)
        self.robot_id = p.loadURDF(
            urdf_path, basePosition=[0, 0, TOP_RAIL_HEIGHT_M], useFixedBase=True
        )

        self._build_farm_structure()
        self._sync_hoist_visual()

        if gui:
            farm_span_x = (COLUMNS - 1) * COLUMN_SPACING_M
            farm_span_y = (ROWS - 1) * ROW_SPACING_M
            farm_diag = (farm_span_x ** 2 + farm_span_y ** 2) ** 0.5
            p.resetDebugVisualizerCamera(
                cameraDistance=max(2.8, farm_diag * 0.95), cameraYaw=35, cameraPitch=-30,
                cameraTargetPosition=[farm_span_x / 2, farm_span_y / 2, TOP_TROUGH_Z / 2],
            )

    # ---------- scene construction ----------

    def _build_farm_structure(self):
        """Poles tall enough to clear the robot's operating envelope,
        connected at the top by a full grid of horizontal beams (along
        rows AND along columns) -- the structure the trolley rides on.
        Plus a trough marker at every (row, col, layer, side) position.
        All of this is visual-only (no collision shape): early on, the
        poles DID have collision geometry, and the carriage physically
        jammed against them while sliding laterally past -- a pole sits
        exactly on the path the carriage has to travel, so with
        collision enabled the robot was rigid-body-blocked by its own
        farm structure."""
        pole_grey = (0.42, 0.42, 0.45, 1.0)
        beam_grey = (0.5, 0.5, 0.53, 1.0)

        for row in range(ROWS):
            row_y = row * ROW_SPACING_M
            for col in range(COLUMNS):
                station_x = col * COLUMN_SPACING_M
                self._add_static_cylinder(
                    radius=POLE_RADIUS_M, height=POLE_TOP_Z,
                    position=[station_x, row_y, POLE_TOP_Z / 2],
                    color=pole_grey,
                )

        # beams along X, one per row, connecting every column's pole at
        # the rail height
        row_beam_half_len = ((COLUMNS - 1) * COLUMN_SPACING_M) / 2
        for row in range(ROWS):
            row_y = row * ROW_SPACING_M
            self._add_static_box(
                half_extents=[row_beam_half_len, BEAM_HALF_THICKNESS_M, BEAM_HALF_THICKNESS_M],
                position=[row_beam_half_len, row_y, TOP_RAIL_HEIGHT_M],
                color=beam_grey,
            )

        # beams along Y, one per column, connecting every row's pole at
        # the rail height -- together with the row beams this forms the
        # full overhead lattice the trolley travels on
        col_beam_half_len = ((ROWS - 1) * ROW_SPACING_M) / 2
        for col in range(COLUMNS):
            station_x = col * COLUMN_SPACING_M
            self._add_static_box(
                half_extents=[BEAM_HALF_THICKNESS_M, col_beam_half_len, BEAM_HALF_THICKNESS_M],
                position=[station_x, col_beam_half_len, TOP_RAIL_HEIGHT_M],
                color=beam_grey,
            )

        half = list(TROUGH_HALF_EXTENTS_M)
        bracket_color = (0.3, 0.3, 0.32, 1.0)
        for pod in self.pods:
            pos = pod["position_m"]
            color = CROP_MARKER_COLOR[pod["crop_type"]]
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
            body_id = p.createMultiBody(
                baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vis,
                basePosition=[pos["x"], pos["y"], pos["z"]],
            )
            self.pod_marker_ids[pod["pod_id"]] = body_id

            # a short bracket physically connecting the trough to its
            # pole -- without this, the trough just floats next to the
            # pole with nothing visibly holding it there. The bracket
            # spans from the pole's centerline out to the trough.
            row_y = pod["row"] * ROW_SPACING_M
            bracket_span = pos["y"] - row_y  # signed: + for front, - for back
            bracket_half_y = abs(bracket_span) / 2
            self._add_static_box(
                half_extents=[0.025, max(bracket_half_y, 0.01), 0.018],
                position=[pos["x"], row_y + bracket_span / 2, pos["z"]],
                color=bracket_color,
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

    def mark_pod_decision(self, pod_id, action):
        color = DECISION_COLOR.get(action, DECISION_COLOR[None])
        p.changeVisualShape(self.pod_marker_ids[pod_id], -1, rgbaColor=color)

    # ---------- hoist visual ----------

    def _sync_hoist_visual(self):
        """Redraws the rod connecting the mast's bottom to the camera's
        actual current position, sized to exactly close that gap -- no
        more, no less. The URDF's own hoist_link is a small fixed-length
        stub (just enough to carry the camera mount); it was previously
        a longer fixed-length rod standing in for the whole reach, which
        left a visible gap for any extension beyond half its own length
        -- every layer in this farm needs 4-15x that much. Since a truly
        gap-free multi-stage telescoping mechanism would either need 5+
        nested segments or still show a minimum length regardless of
        need, and this farm only has a handful of discrete reach
        distances (one per layer), redrawing a rod sized to the exact
        current gap after each settle is the simpler, exactly-correct
        alternative -- recreated only a few times per move (after
        retract and after descend), not every physics step."""
        mast_z = p.getLinkState(self.robot_id, MAST_LINK)[0][2]
        mast_bottom_z = mast_z - (MAST_LENGTH_M / 2)
        cam_x, cam_y, cam_z = p.getLinkState(self.robot_id, CAMERA_LINK)[0]

        rod_length = max(0.02, mast_bottom_z - cam_z)
        mid_z = (mast_bottom_z + cam_z) / 2

        if self._hoist_visual_id is not None:
            p.removeBody(self._hoist_visual_id)

        vis = p.createVisualShape(
            p.GEOM_CYLINDER, radius=0.017, length=rod_length,
            rgbaColor=(0.15, 0.45, 0.75, 1.0),
        )
        self._hoist_visual_id = p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vis,
            basePosition=[cam_x, cam_y, mid_z],
        )

    # ---------- movement ----------

    def _settle(self, targets, step_sleep=0.0, sync_visual=False):
        """Commands each {joint_index: target_position} in `targets`
        with POSITION_CONTROL, then steps the simulation until every one
        of them is within POSITION_TOLERANCE_M (or the step cap is hit,
        so a bad target can't hang the run forever). Returns steps taken.
        Any joint NOT in `targets` keeps whatever position control target
        it was last given -- e.g. during a travel phase the hoist isn't
        re-commanded, so it just continues holding its retracted position.
        When sync_visual is True, the connecting rod is redrawn every
        step so it tracks the camera continuously during the motion
        instead of only snapping into place once settled -- otherwise
        the rod appears to stay put (or vanish) for the whole motion and
        then suddenly jump to the right length at the end."""
        for joint, target in targets.items():
            force = 200 if joint == HOIST_JOINT else 250
            p.setJointMotorControl2(
                self.robot_id, joint, p.POSITION_CONTROL,
                targetPosition=target, force=force, maxVelocity=0.6,
            )

        steps = 0
        while steps < MAX_SETTLE_STEPS:
            p.stepSimulation()
            steps += 1
            if step_sleep:
                time.sleep(step_sleep)
            if sync_visual:
                self._sync_hoist_visual()

            if all(
                abs(p.getJointState(self.robot_id, joint)[0] - target) < POSITION_TOLERANCE_M
                for joint, target in targets.items()
            ):
                break

        return steps

    def move_to(self, pod, step_sleep=0.0):
        """
        Five-phase move -- the way a real automated storage/retrieval
        shuttle actually operates, symmetric on both ends:
          0. retreat: slide sideways (row_joint) back to the CURRENT
             pole's centerline, still at the current height, before
             lifting at all -- backs out of whatever trough it was just
             sitting in instead of yanking straight up through it
          1. retract the hoist fully (camera pulled up above every
             trough top, see RAIL_CLEARANCE_M) -- now climbing a clear
             centerline column, not through the trough it just left
          2. travel horizontally (carriage_joint + row_joint) to the
             pod's COLUMN, but only as far as the NEW pole's own
             centerline in Y (not yet the pod's front/back offset) --
             at that retracted height, clear of every pod and the beams
          3. descend (hoist_joint) straight down the new pole's
             centerline to the target layer's height -- still clear,
             because every trough is offset sideways from the pole's
             own line, so nothing sits directly on it at any height
          4. approach: slide sideways (row_joint) the last short
             distance from the centerline out to the pod's actual
             front/back position, now that the hoist is already at the
             correct height -- no other layer shares that exact height,
             so this final sideways move can't clip a different trough

        An earlier version only had phases 2-4 (no retreat), which
        correctly avoided clipping OTHER poles' pods but not the one it
        was just leaving: retracting straight up from a resting position
        inside a trough's footprint dragged the camera through that same
        trough's box on the way out, before row_joint ever got a chance
        to move away. Retreating to centerline FIRST, at the current
        height, fixes that -- verified directly by checking the camera's
        position against every trough's bounding box at every single
        simulation step across a full 72-pod run: zero intersections
        with any pod other than the one actually being serviced at rest.

        Returns total steps taken across all five phases.
        """
        target_x = pod["position_m"]["x"]
        target_y = pod["position_m"]["y"]
        target_z = pod["position_m"]["z"]
        new_centerline_y = pod["row"] * ROW_SPACING_M
        target_hoist = (TOP_RAIL_HEIGHT_M - MAST_LENGTH_M) - target_z

        # snap the CURRENT row_joint reading to its nearest centerline --
        # this is wherever the robot is actually sitting right now
        # (whichever pole/side it last serviced), not the new target's row
        current_row_y = p.getJointState(self.robot_id, ROW_JOINT)[0]
        current_centerline_y = round(current_row_y / ROW_SPACING_M) * ROW_SPACING_M

        steps = 0
        steps += self._settle({ROW_JOINT: current_centerline_y}, step_sleep, sync_visual=True)
        steps += self._settle({HOIST_JOINT: 0.0}, step_sleep, sync_visual=True)
        steps += self._settle(
            {CARRIAGE_JOINT: target_x, ROW_JOINT: new_centerline_y}, step_sleep
        )
        steps += self._settle({HOIST_JOINT: target_hoist}, step_sleep, sync_visual=True)
        steps += self._settle({ROW_JOINT: target_y}, step_sleep, sync_visual=True)
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

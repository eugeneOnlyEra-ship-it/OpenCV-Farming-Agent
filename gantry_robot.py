"""
gantry_robot.py

Loads the two-joint gantry URDF and provides:
  - move_to(pod): drives both prismatic joints toward a pod's target
    (x, z) using position control, stepping the simulation until both
    joints settle within tolerance (i.e. "stop precisely at the pod").
  - a lightweight static scene: rail struts + a small marker cube at
    every pod position (colored by crop, recolored after each visit to
    reflect the decision made -- green=log_and_continue,
    yellow=schedule_monitoring, red=flag_for_treatment) so the GUI
    demo shows the decision loop's effect at a glance, not just the
    printed log.

Coordinate mapping: pod_registry gives (x, y, z) in meters; the
carriage_joint moves along local X (lateral) and the lift_joint moves
along base-frame Z (vertical). Y is fixed by the URDF's camera offset,
matching the fixed SHELF_DEPTH_M convention in pod_registry.
"""

import pybullet as p
import pybullet_data
import os
import time

LIFT_JOINT = 0
CARRIAGE_JOINT = 1

POSITION_TOLERANCE_M = 0.01
MAX_SETTLE_STEPS = 480  # ~2s at 240Hz sim step, safety cap so a stuck joint can't hang the loop

DECISION_COLOR = {
    "log_and_continue": (0.20, 0.75, 0.25, 1.0),   # green
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
        self.robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=True)

        self._build_static_scene()
        self._build_pod_markers()

        if gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=2.6, cameraYaw=35, cameraPitch=-25,
                cameraTargetPosition=[0.7, 0.2, 0.9],
            )

    # ---------- scene construction ----------

    def _build_static_scene(self):
        """Vertical rail column + horizontal top/bottom rails, visual only."""
        max_x = max(pod["position_m"]["x"] for pod in self.pods) + 0.3
        max_z = max(pod["position_m"]["z"] for pod in self.pods) + 0.3

        rail_grey = (0.4, 0.4, 0.42, 1.0)

        # vertical column at x=-0.15 (behind pod row), full height
        self._add_static_box(
            half_extents=[0.05, 0.05, max_z / 2],
            position=[-0.15, 0.05, max_z / 2],
            color=rail_grey,
        )
        # horizontal rail spanning all lateral pod positions, at the top
        self._add_static_box(
            half_extents=[max_x / 2, 0.05, 0.03],
            position=[max_x / 2 - 0.15, 0.05, max_z + 0.15],
            color=rail_grey,
        )

    def _add_static_box(self, half_extents, position, color):
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
        return p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
            basePosition=position,
        )

    def _build_pod_markers(self):
        """One small cube per pod, positioned at its registry coordinates,
        colored by crop until a visit recolors it by decision."""
        half = [0.10, 0.03, 0.10]
        for pod in self.pods:
            pos = pod["position_m"]
            color = CROP_MARKER_COLOR[pod["crop_type"]]
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
            body_id = p.createMultiBody(
                baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                basePosition=[pos["x"], pos["y"] + 0.15, pos["z"]],
            )
            self.pod_marker_ids[pod["pod_id"]] = body_id

    def mark_pod_decision(self, pod_id, action):
        color = DECISION_COLOR.get(action, DECISION_COLOR[None])
        p.changeVisualShape(self.pod_marker_ids[pod_id], -1, rgbaColor=color)

    # ---------- movement ----------

    def move_to(self, pod, step_sleep=0.0):
        """
        Drives lift_joint to the pod's z and carriage_joint to the pod's
        x using POSITION_CONTROL, stepping the simulation until both
        joints are within POSITION_TOLERANCE_M of target (or the step
        cap is hit, so a bad target can't hang the run forever).
        Returns the number of simulation steps taken.
        """
        target_x = pod["position_m"]["x"]
        target_z = pod["position_m"]["z"]

        p.setJointMotorControl2(
            self.robot_id, LIFT_JOINT, p.POSITION_CONTROL,
            targetPosition=target_z, force=200, maxVelocity=0.6,
        )
        p.setJointMotorControl2(
            self.robot_id, CARRIAGE_JOINT, p.POSITION_CONTROL,
            targetPosition=target_x, force=200, maxVelocity=0.6,
        )

        steps = 0
        while steps < MAX_SETTLE_STEPS:
            p.stepSimulation()
            steps += 1
            if step_sleep:
                time.sleep(step_sleep)

            lift_pos = p.getJointState(self.robot_id, LIFT_JOINT)[0]
            carriage_pos = p.getJointState(self.robot_id, CARRIAGE_JOINT)[0]
            if (abs(lift_pos - target_z) < POSITION_TOLERANCE_M
                    and abs(carriage_pos - target_x) < POSITION_TOLERANCE_M):
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

"""
main.py

Standalone PyBullet demo loop for OpenCV-Farming-Agent, Step 4.

For every pod in pod_visit_order():
  1. move the gantry precisely to that pod's (x, z)
  2. "capture a frame" -- load the pod's assigned ground-truth image
  3. call classify_pod(pod_id, crop_type, image)   [stubbed for now]
  4. call decide(pod_id, crop_type, classification)
  5. log the full row (see run_logger.py)
  6. act on the decision: recolor the pod marker + dwell for an
     action-specific duration, so the physical behavior visibly differs
     by decision, then continue to the next pod

Run modes:
  python3 main.py                       # headless (DIRECT), pure-random stub, all pods
  python3 main.py --gui                 # opens a PyBullet GUI window (needs a local display)
  python3 main.py --oracle              # use the "noisy oracle" stub instead of pure random
  python3 main.py --pods 4              # only visit the first N pods (quick smoke test)
  python3 main.py --sleep 0.01          # slow down sim steps for a watchable GUI demo

`classify_pod` is imported from perception_stub.py today. Once the real
OpenCV 5 pipeline is ready, point PERCEPTION_FN at the real module's
classify_pod (same signature) -- nothing else in this file needs to
change, which is the whole point of the interface contract.
"""

import argparse
import random

from PIL import Image

from pod_registry import pod_visit_order
from perception_stub import classify_pod, classify_pod_noisy_oracle
from decision_agent import decide, ACTION_DWELL_SECONDS
from gantry_robot import GantryRobot
from run_logger import RunLogger


def load_frame(pod):
    """Loads the pod's assigned image as the stand-in camera frame.
    Returns a PIL.Image; the real pipeline can swap this for a cv2
    frame without changing anything upstream/downstream of this call."""
    return Image.open(pod["image_path"]).convert("RGB")


def run(gui=False, use_oracle=False, max_pods=None, step_sleep=0.0, seed=None):
    rng = random.Random(seed)
    pods = pod_visit_order()
    if max_pods:
        pods = pods[:max_pods]

    robot = GantryRobot(pods=pod_visit_order(), gui=gui)  # markers for the FULL farm, even on a partial run
    logger = RunLogger(out_dir="logs")

    print(f"\n=== OpenCV-Farming-Agent | PyBullet demo loop | {len(pods)} pod(s) ===\n")

    try:
        for pod in pods:
            steps = robot.move_to(pod, step_sleep=step_sleep)
            frame = load_frame(pod)

            if use_oracle:
                classification = classify_pod_noisy_oracle(
                    pod["pod_id"], pod["crop_type"], frame,
                    ground_truth=pod["ground_truth"], rng=rng,
                )
            else:
                classification = classify_pod(pod["pod_id"], pod["crop_type"], frame, rng=rng)

            decision = decide(pod["pod_id"], pod["crop_type"], classification)
            row = logger.log(pod, classification, decision)

            robot.mark_pod_decision(pod["pod_id"], decision["action"])
            robot.dwell(ACTION_DWELL_SECONDS.get(decision["action"], 0.4))

            print(
                f"[{pod['pod_id']:>6}] {pod['crop_type']:<9} "
                f"saw stage='{classification['growth_stage']}' "
                f"disease={classification['disease_flag']} "
                f"conf={classification['confidence']:.2f}  "
                f"-> {decision['action']:<20} ({decision['reason']})  "
                f"[settled in {steps} steps]"
            )

        csv_path, json_path = logger.flush()
        summary = logger.summary()

        print("\n--- run summary ---")
        for k, v in summary.items():
            print(f"{k}: {v}")
        print(f"\nlog written to:\n  {csv_path}\n  {json_path}")

    finally:
        robot.disconnect()

    return logger


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="OpenCV-Farming-Agent PyBullet demo loop")
    parser.add_argument("--gui", action="store_true", help="open a PyBullet GUI window")
    parser.add_argument("--oracle", action="store_true",
                         help="use the noisy-oracle stub (mostly-correct) instead of pure random")
    parser.add_argument("--pods", type=int, default=None, help="only visit the first N pods")
    parser.add_argument("--sleep", type=float, default=0.0, help="seconds to sleep per sim step (GUI demos)")
    parser.add_argument("--seed", type=int, default=None, help="RNG seed for reproducible stub output")
    args = parser.parse_args()

    run(gui=args.gui, use_oracle=args.oracle, max_pods=args.pods,
        step_sleep=args.sleep, seed=args.seed)

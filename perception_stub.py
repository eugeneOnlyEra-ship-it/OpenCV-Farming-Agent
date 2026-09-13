"""
perception_stub.py

Stand-in for the real OpenCV 5 + ML perception pipeline (classical
CV growth-stage classifier + YOLOv8/ONNX disease detector). Implements
the exact interface contract locked in for this project:

    classify_pod(pod_id, crop_type, image) -> {
        "growth_stage": str,
        "disease_flag": bool,
        "confidence": float,   # 0-1
    }

so the PyBullet loop, the decision agent, and the logger can all be
built and demoed against this stub today, then keep working unmodified
once real_perception.classify_pod (same signature) is swapped in.

The stub returns *placeholder/random* results by design (per handoff
doc item 4) -- it deliberately does NOT read the ground truth baked
into pod_registry, because the whole point of Step 4 is to prove the
robot/decision/logging loop works against an arbitrary classifier
output, not to fake a perfect model. A `--use-ground-truth` style
"noisy oracle" mode is included separately below for demo runs where
you want the decision agent's branching to look sensible on camera;
use whichever mode fits what you're demoing.
"""

import random

from pod_registry import GROWTH_STAGES


def classify_pod(pod_id, crop_type, image, rng=None):
    """
    Pure stub: ignores `image` entirely and returns a random-but-valid
    classification for the given crop_type. `image` is still accepted
    (and should be a loaded frame, e.g. a PIL.Image or numpy array) so
    the call signature matches what the real pipeline will need.
    """
    rng = rng or random
    stages = GROWTH_STAGES[crop_type]
    return {
        "growth_stage": rng.choice(stages),
        "disease_flag": rng.random() < 0.25,
        "confidence": round(rng.uniform(0.55, 0.99), 3),
    }


def classify_pod_noisy_oracle(pod_id, crop_type, image, ground_truth, rng=None,
                               correct_prob=0.85):
    """
    Optional demo mode: mostly returns the pod's real ground-truth
    condition (from pod_registry) with some injected noise/uncertainty,
    so a live walkthrough produces a believable, mostly-correct trace
    instead of pure noise -- useful for showing judges the decision
    agent branching correctly most of the time, the way the real
    trained models are expected to behave. Still fully stubbed: no
    actual image analysis happens.
    """
    rng = rng or random
    stages = GROWTH_STAGES[crop_type]
    correct = rng.random() < correct_prob

    if correct:
        stage = ground_truth["growth_stage"]
        disease = ground_truth["diseased"]
        confidence = round(rng.uniform(0.8, 0.99), 3)
    else:
        stage = rng.choice(stages)
        disease = rng.random() < 0.3
        confidence = round(rng.uniform(0.4, 0.75), 3)

    return {
        "growth_stage": stage,
        "disease_flag": disease,
        "confidence": confidence,
    }

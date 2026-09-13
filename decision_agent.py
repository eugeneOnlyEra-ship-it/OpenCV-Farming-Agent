"""
decision_agent.py

Consumes classify_pod() output and picks one of three actions. This is
the piece the Agentic Vision Award is judged on: the classification
result has to visibly change what happens next, not just get logged.

Decisions:
  - "log_and_continue"     : healthy, confident reading -> note it, move on
  - "flag_for_treatment"   : disease detected with reasonable confidence
                              -> would trigger human review / treatment in
                              the full system; here it also makes the
                              robot pause longer at that pod as a visible
                              side effect
  - "schedule_monitoring"  : reading is uncertain (low confidence) OR
                              crop is at a stage where closer tracking
                              matters (e.g. approaching harvest) -> queue
                              a re-visit sooner than the normal rotation

Thresholds are intentionally simple and centralized here so they're
easy to defend/tune in the report, and easy to swap for a config file
later without touching the PyBullet or logging code.
"""

CONFIDENCE_FLOOR = 0.6          # below this, don't trust disease_flag at all
LOW_CONFIDENCE_RESCHEDULE = 0.7  # below this (but above the floor), re-check sooner

# Growth stages, per crop, that are "close to harvest" and worth tighter
# monitoring even when the reading looks healthy and confident.
NEAR_HARVEST_STAGES = {
    "cabbage": {"S8", "S9"},
    "lettuce": {"Head formation", "Harvest stage"},
    "mushroom": {"Intermediate", "Harvest"},
}


def decide(pod_id, crop_type, classification):
    """
    Returns a dict: {"action": str, "reason": str} given the pod id,
    crop type, and a classify_pod()-shaped dict
    ({growth_stage, disease_flag, confidence}).
    """
    stage = classification["growth_stage"]
    disease_flag = classification["disease_flag"]
    confidence = classification["confidence"]

    if confidence < CONFIDENCE_FLOOR:
        return {
            "action": "schedule_monitoring",
            "reason": f"low-confidence reading ({confidence:.2f} < {CONFIDENCE_FLOOR}) "
                      f"-- re-check before trusting it",
        }

    if disease_flag:
        return {
            "action": "flag_for_treatment",
            "reason": f"disease detected at stage '{stage}' with confidence {confidence:.2f}",
        }

    if confidence < LOW_CONFIDENCE_RESCHEDULE:
        return {
            "action": "schedule_monitoring",
            "reason": f"healthy but marginal confidence ({confidence:.2f}) -- re-check sooner",
        }

    if stage in NEAR_HARVEST_STAGES.get(crop_type, set()):
        return {
            "action": "schedule_monitoring",
            "reason": f"healthy, confident, and near harvest ('{stage}') -- tighten monitoring cadence",
        }

    return {
        "action": "log_and_continue",
        "reason": f"healthy, confident reading ({confidence:.2f}) at stage '{stage}'",
    }


# Per-action robot behavior, consumed by main.py so the *physical*
# response is visibly different depending on the decision (this is what
# turns "we printed a label" into "the robot did something different").
ACTION_DWELL_SECONDS = {
    "log_and_continue": 0.4,
    "schedule_monitoring": 0.9,
    "flag_for_treatment": 1.6,
}

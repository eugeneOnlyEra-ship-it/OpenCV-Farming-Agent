"""
run_logger.py

Writes one row per pod visit: timestamp, pod id, crop, growth stage,
disease flag, confidence, and the decision that was made. This is the
artifact that (a) later feeds the DynamoDB per-pod health history table
and the report's evaluation section, and (b) is the direct evidence for
the Agentic Vision Award -- a reviewer can open this file and see
"saw X -> decided Y" for every stop in the run, without needing to
watch the GUI live.

Writes both CSV (for spreadsheet/report use) and JSON (for programmatic
use, e.g. later DynamoDB batch-loading) from the same in-memory rows.
"""

import csv
import json
import os
from datetime import datetime, timezone

FIELDNAMES = [
    "timestamp", "pod_id", "station", "layer", "side", "crop_type",
    "growth_stage", "disease_flag", "confidence",
    "action", "reason", "ground_truth_stage", "ground_truth_diseased",
]


class RunLogger:
    def __init__(self, out_dir):
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.csv_path = os.path.join(out_dir, f"run_{run_id}.csv")
        self.json_path = os.path.join(out_dir, f"run_{run_id}.json")
        self.rows = []

    def log(self, pod, classification, decision):
        row = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "pod_id": pod["pod_id"],
            "station": pod["station"],
            "layer": pod["layer"],
            "side": pod["side"],
            "crop_type": pod["crop_type"],
            "growth_stage": classification["growth_stage"],
            "disease_flag": classification["disease_flag"],
            "confidence": classification["confidence"],
            "action": decision["action"],
            "reason": decision["reason"],
            "ground_truth_stage": pod["ground_truth"]["growth_stage"],
            "ground_truth_diseased": pod["ground_truth"]["diseased"],
        }
        self.rows.append(row)
        return row

    def flush(self):
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
            writer.writeheader()
            writer.writerows(self.rows)
        with open(self.json_path, "w") as f:
            json.dump(self.rows, f, indent=2)
        return self.csv_path, self.json_path

    def summary(self):
        total = len(self.rows)
        by_action = {}
        for r in self.rows:
            by_action[r["action"]] = by_action.get(r["action"], 0) + 1
        agreement = sum(
            1 for r in self.rows
            if r["growth_stage"] == r["ground_truth_stage"]
            and r["disease_flag"] == r["ground_truth_diseased"]
        )
        return {
            "total_pods_visited": total,
            "actions": by_action,
            "stub_agreement_with_ground_truth": f"{agreement}/{total}",
        }

"""
agent_rail_demo.py

Standalone demo of just the station/pole structure and the agent's
rail-level movement -- no arm, no perception, no decision logic (all
set aside for this iteration per the current focus). Confirms the
structure and the travel order before the arm gets reattached:

    dock-top -> every station, row by row -> dock-bottom

Run modes:
  python3 agent_rail_demo.py                 # headless, full run
  python3 agent_rail_demo.py --gui            # opens a PyBullet window
  python3 agent_rail_demo.py --sleep 0.004     # slows it down for --gui
"""

import argparse

from pod_registry import station_visit_order
from agent_robot import AgentRobot


def run(gui=False, step_sleep=0.0, dwell_seconds=0.3):
    stations = station_visit_order()
    robot = AgentRobot(gui=gui)

    print(f"\n=== Agent rail demo | {len(stations)} stops "
          f"({len(stations) - 2} stations + 2 docks) ===\n")

    try:
        for stop in stations:
            steps = robot.move_to_station(stop, step_sleep=step_sleep)
            kind = "DOCK" if stop["is_dock"] else "station"
            print(f"[{stop['label']:>10}] {kind:<8} "
                  f"x={stop['x']:.2f} y={stop['y']:.2f}  [settled in {steps} steps]")
            robot.dwell(dwell_seconds)

        print("\nRun complete -- agent parked at dock-bottom.")

    finally:
        robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Agent rail-travel demo (no arm)")
    parser.add_argument("--gui", action="store_true", help="open a PyBullet GUI window")
    parser.add_argument("--sleep", type=float, default=0.0, help="seconds to sleep per sim step")
    parser.add_argument("--dwell", type=float, default=0.3, help="seconds to pause at each stop")
    args = parser.parse_args()

    run(gui=args.gui, step_sleep=args.sleep, dwell_seconds=args.dwell)

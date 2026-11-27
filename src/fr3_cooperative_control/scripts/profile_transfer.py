#!/usr/bin/env python3
"""Observe the existing full-transfer launch without changing robot commands."""
import argparse
import datetime
import json
import os
import re
from pathlib import Path
import signal
import subprocess
import time


MARKERS = (
    ("left_grasp_ready", "PICK TARGET SETTLED:"),
    ("left_pick_verified", "PICK TEST PASSED:"),
    ("left_table_transfer", "TABLE TRANSFER START:"),
    ("left_table_approach", "PLACE TOP-DOWN ALIGN AND APPROACH:"),
    ("left_descent", "PLACE DESCENT:"),
    ("left_release", "OPEN ON TABLE:"),
    ("left_retreat", "PLACE RETREAT:"),
    ("left_place_verified", "PLACE TEST PASSED:"),
    ("left_park", "LEFT PARK START:"),
    ("left_park_verified", "LEFT PARK PASSED"),
    ("right_grasp_ready", "RIGHT PICK TARGET SETTLED:"),
    ("right_pick_verified", "RIGHT PICK TEST PASSED:"),
    ("right_clearance", "RIGHT CLEARANCE START:"),
    ("shelf_search", "SHELF SEARCH CANDIDATE:"),
    ("shelf_turn", "SHELF TRANSFER TURN START"),
    ("shelf_advance", "SHELF TRANSFER ADVANCE START"),
    ("shelf_insert", "SHELF INSERT START:"),
    ("shelf_descent", "SHELF DESCENT START:"),
    ("shelf_release", "SHELF OPEN:"),
    ("shelf_raise", "SHELF RETREAT CLEARANCE:"),
    ("shelf_retreat", "SHELF RETREAT START:"),
    ("shelf_place_verified", "RIGHT SHELF PLACE PASSED:"),
)


class Recorder:
    def __init__(self):
        self.events = []
        self.seen = set()
        self.result = "incomplete"
        self.finished = None
        self.pending = {}
        self.operations = []

    def feed(self, line, elapsed):
        if self.finished is not None:
            return
        self.observe_operations(line, elapsed)
        # First task message is observable, not an exact simulator-ready signal.
        if "[single_arm_pick_test]:" in line and "first_task_message" not in self.seen:
            self.add("first_task_message", elapsed)
        for name, marker in MARKERS:
            if marker in line and name not in self.seen:
                self.add(name, elapsed)
        if "FULL TRANSFER TEST PASSED" in line:
            self.result = "passed"
            self.finished = elapsed
        elif "TABLE HANDOFF FAILED:" in line:
            self.result = "failed"
            self.finished = elapsed

    def add(self, name, elapsed):
        self.seen.add(name)
        self.events.append({"event": name, "elapsed_s": round(elapsed, 3)})

    def observe_operations(self, line, elapsed):
        start = re.search(r"sending trajectory to (arm_[12]_controller)", line)
        end = re.search(r"Controller '(arm_[12]_controller)' successfully finished", line)
        grip = re.search(r"\[(left|right)_gripper_controller\]: Received & accepted", line)
        finger = "PHYSICAL FINGERS:" in line
        if start:
            key = start.group(1)
            self.pending.setdefault(key, (elapsed, "arm_controller"))
        if grip:
            key = grip.group(1) + "_gripper"
            self.pending.setdefault(key, (elapsed, "gripper_until_finger_check"))
        key = end.group(1) if end else None
        if finger and "[single_arm_pick_test]:" in line:
            key = "left_gripper"
        elif finger and "[right_arm_table_pick]:" in line:
            key = "right_gripper"
        if key in self.pending and (end or finger):
            began, kind = self.pending.pop(key)
            self.operations.append({"controller": key, "kind": kind,
                                    "start_s": began, "end_s": elapsed,
                                    "duration_s": elapsed - began})

    def report(self):
        intervals = []
        previous_name, previous_time = "launch_start", 0.0
        for event in self.events:
            intervals.append({"from": previous_name, "to": event["event"],
                              "duration_s": round(event["elapsed_s"] - previous_time, 3)})
            previous_name, previous_time = event["event"], event["elapsed_s"]
        if self.finished is not None:
            intervals.append({"from": previous_name, "to": self.result,
                              "duration_s": round(self.finished - previous_time, 3)})
        operations = self.operations
        return {"result": self.result, "total_until_result_s": self.finished,
                "events": self.events, "intervals": intervals,
                "operations": operations,
                "operation_totals_s": {
                    kind: sum(op["duration_s"] for op in operations if op["kind"] == kind)
                    for kind in ("arm_controller", "gripper_until_finger_check")},
                "unfinished_operations": list(self.pending),
                "measurement": "monotonic wall time when log lines arrive; includes startup; "
                "intervals combine planning, motion and waits; shutdown excluded from result; "
                "controller durations include action communication; gripper durations end at "
                "physical finger verification; overlapping durations must not be added as a partition"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="log/transfer_profiles")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--optimize-shelf-route", action="store_true")
    parser.add_argument("--optimize-table-transfer", action="store_true")
    args = parser.parse_args()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    base = output / stamp
    command = ["ros2", "launch", "fr3_cooperative_control", "table_handoff_test.launch.py",
               "right_stage:=shelf_place"]
    if args.headless:
        command.append("headless:=true")
    if args.optimize_shelf_route:
        command.append("optimize_shelf_route:=true")
    if args.optimize_table_transfer:
        command.append("optimize_table_transfer:=true")
    recorder = Recorder()
    start = time.monotonic()
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, errors="replace", bufsize=1, start_new_session=True)

    def interrupt(signum, frame):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)

    old_handler = signal.signal(signal.SIGINT, interrupt)
    try:
        with base.with_suffix(".log").open("w", encoding="utf-8") as log, \
                base.with_suffix(".events.jsonl").open("w", encoding="utf-8") as events:
            for line in process.stdout:
                elapsed = time.monotonic() - start
                print(line, end="", flush=True)
                log.write(line)
                log.flush()
                previous = recorder.finished
                events.write(json.dumps({"elapsed_s": elapsed, "line": line.rstrip("\n")},
                                        ensure_ascii=False) + "\n")
                events.flush()
                recorder.feed(line, elapsed)
                if previous is None and recorder.finished is not None:
                    report = recorder.report()
                    report["command"] = command
                    base.with_suffix(".json").write_text(
                        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
                    print(f"\nPROFILE RESULT: {recorder.result}; {recorder.finished:.1f} s", flush=True)
                    for interval in report["intervals"]:
                        print(f"  {interval['from']} -> {interval['to']}: "
                              f"{interval['duration_s']:.1f} s", flush=True)
                    for kind, duration in report["operation_totals_s"].items():
                        print(f"  {kind}: {duration:.1f} s (matched operations)", flush=True)
                    print(f"PROFILE FILE: {base.with_suffix('.json')}", flush=True)
                    print("Close the simulation with Ctrl+C when finished.", flush=True)
            process.wait()
    finally:
        signal.signal(signal.SIGINT, old_handler)
        report = recorder.report()
        report["command"] = command
        report["process_exit_code"] = process.poll()
        base.with_suffix(".json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0 if recorder.result == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

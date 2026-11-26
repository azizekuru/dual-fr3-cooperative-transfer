#!/usr/bin/env python3
"""Run many transfer tasks one after another and summarise reliability, time and accuracy.

Each task is one full `table_handoff_test.launch.py` run (source cell -> table -> target cell).
Runs never overlap: a run is stopped once its result is known, and the next one starts only
after every Gazebo / launch process of the previous run has exited.

Examples:
  ros2 run fr3_cooperative_control batch_transfer_test.py --random 10
  ros2 run fr3_cooperative_control batch_transfer_test.py --all --levels 2 3
  ros2 run fr3_cooperative_control batch_transfer_test.py --random 5 \\
      --extra rigid_grasp:=true speed_scale:=2.0
"""
import argparse
import csv
import itertools
import os
import queue
import random
import re
import shutil
import signal
import statistics
import subprocess
import sys
import threading
import time
from datetime import datetime

PASS = 'FULL TRANSFER TEST PASSED'
FAIL = 'TABLE HANDOFF FAILED'
SLOTS = ['left', 'center', 'right']


def sim_processes():
    """PIDs of running Gazebo / launch processes (any run, not only ours)."""
    out = subprocess.run(['pgrep', '-f', '[g]z sim|[p]ython3 .*bin/ros2 launch'], capture_output=True, text=True)
    return [int(p) for p in out.stdout.split()]


def wait_until_idle(timeout):
    end = time.time() + timeout
    while time.time() < end:
        if not sim_processes():
            return True
        time.sleep(1.0)
    return False


def stop(process, grace=30.0):
    """SIGINT the launch process group (like Ctrl+C), escalate only if it does not exit."""
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 10.0), (signal.SIGKILL, 5.0)):
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            continue


def parse(lines):
    """Metrics from the coordinator's log lines."""
    text = '\n'.join(lines)

    def first(pattern, cast=str):
        m = re.search(pattern, text)
        return cast(m.group(1)) if m else None

    stamps = {}
    for line in lines:
        m = re.search(r'\[(\d+\.\d+)\] \[transfer_task\]: (PHYSICAL LIFT|PICK TEST PASSED|'
                      r'LEFT PLACE COMPLETE|RIGHT SHELF PLACE PASSED)', line)
        if m and m.group(2) not in stamps:
            stamps[m.group(2)] = float(m.group(1))
    rtf = None
    sim_hold = first(r'PICK TEST PASSED: .*?hold=([0-9.]+) sim s', float) or 3.0
    if 'PHYSICAL LIFT' in stamps and 'PICK TEST PASSED' in stamps:
        hold = stamps['PICK TEST PASSED'] - stamps['PHYSICAL LIFT']
        rtf = round(sim_hold / hold, 2) if hold > 0 else None
    reason = first(r'(?:SINGLE ARM|RIGHT ARM) TEST FAILED: ([^\n;]+)')
    return {
        'table_xy_mm': first(r'PLACE TEST PASSED: .*?xy_error=([0-9.]+) mm', float),
        'shelf_xy_mm': first(r'RIGHT SHELF PLACE PASSED: .*?xy_error=([0-9.]+) mm', float),
        'pick_grasp': first(r'PICK GRASP SELECTED: (pitch=\S+ deg retreat=\S+ mm)'),
        'rtf': rtf,
        'failure': reason,
        'failed_arm': ('left' if 'SINGLE ARM TEST FAILED' in text else
                       'right' if 'RIGHT ARM TEST FAILED' in text else None),
    }


def run_task(task, args, log_dir):
    name = (f"L{task['source_level']}{task['source_slot'][0]}-"
            f"L{task['target_level']}{task['target_slot'][0]}")
    cmd = ['ros2', 'launch', 'fr3_cooperative_control', 'table_handoff_test.launch.py',
           'right_stage:=shelf_place', 'single_process:=true',
           f"headless:={'true' if args.headless else 'false'}",
           f"source_level:={task['source_level']}", f"source_slot:={task['source_slot']}",
           f"target_level:={task['target_level']}", f"target_slot:={task['target_slot']}"]
    cmd += args.extra
    log_path = os.path.join(log_dir, f"{task['index']:03d}_{name}.log")
    print(f"[{task['index']}/{task['count']}] {name}: {' '.join(cmd[4:])}", flush=True)
    start = time.time()
    lines, result = [], 'TIMEOUT'
    feed = queue.Queue()
    with open(log_path, 'w') as log:
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, start_new_session=True, bufsize=1)
        reader = threading.Thread(target=lambda: [feed.put(l) for l in process.stdout],
                                  daemon=True)
        reader.start()
        try:
            while time.time() - start < args.timeout:
                try:
                    line = feed.get(timeout=0.5)
                except queue.Empty:
                    if process.poll() is not None and not reader.is_alive():
                        result = 'EXITED'
                        break
                    continue
                log.write(line)
                lines.append(line.rstrip())
                if PASS in line:
                    result = 'PASS'
                    break
                if FAIL in line:
                    result = 'FAIL'
                    break
        finally:
            elapsed = time.time() - start
            stop(process)
            reader.join(timeout=10.0)
            while not feed.empty():  # keep the shutdown output in the log
                log.write(feed.get_nowait())
    metrics = parse(lines)
    row = {'index': task['index'], 'task': name, 'result': result,
           'wall_s': round(elapsed, 1), **{k: task[k] for k in
                                           ('source_level', 'source_slot', 'target_level',
                                            'target_slot')},
           **metrics, 'log': log_path}
    detail = (f"table {metrics['table_xy_mm']} mm, shelf {metrics['shelf_xy_mm']} mm"
              if result == 'PASS' else f"{metrics['failed_arm']}: {metrics['failure']}")
    print(f"    -> {result} in {elapsed:.0f} s (RTF {metrics['rtf']}); {detail}", flush=True)
    return row


def summarise(rows):
    print('\n=== SUMMARY ===')
    passed = [r for r in rows if r['result'] == 'PASS']
    print(f"passed {len(passed)}/{len(rows)}")
    if passed:
        times = [r['wall_s'] for r in passed]
        print(f"time (passed): mean {statistics.mean(times):.0f} s, "
              f"min {min(times):.0f} s, max {max(times):.0f} s")
        for key in ('table_xy_mm', 'shelf_xy_mm'):
            values = [r[key] for r in passed if r[key] is not None]
            if values:
                print(f"{key}: mean {statistics.mean(values):.2f}, max {max(values):.2f}")
    failures = {}
    for r in rows:
        if r['result'] != 'PASS':
            key = f"{r['failed_arm'] or '-'}: {r['failure'] or r['result']}"
            failures.setdefault(key, []).append(r['task'])
    for key, tasks in sorted(failures.items(), key=lambda kv: -len(kv[1])):
        print(f"FAIL x{len(tasks)}  {key}  [{', '.join(tasks)}]")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    pick = parser.add_mutually_exclusive_group()
    pick.add_argument('--all', action='store_true', help='every source x target combination')
    pick.add_argument('--random', type=int, default=10, help='number of random tasks (default)')
    parser.add_argument('--levels', type=int, nargs='+', default=[2, 3])
    parser.add_argument('--slots', nargs='+', default=SLOTS, choices=SLOTS)
    parser.add_argument('--repeat', type=int, default=1, help='run every selected task N times')
    parser.add_argument('--timeout', type=float, default=900.0, help='seconds per run')
    parser.add_argument('--gui', dest='headless', action='store_false',
                        help='open the Gazebo window (default: headless)')
    parser.add_argument('--seed', type=int, default=None)
    parser.add_argument('--out', default=None, help='output directory')
    parser.add_argument('--extra', nargs='*', default=[], help='extra launch arguments')
    parser.add_argument('--dry-run', action='store_true', help='only list the selected runs')
    args = parser.parse_args()

    if sim_processes() and not args.dry_run:
        sys.exit('A Gazebo / launch process is already running; stop it first '
                 '(two simulations corrupt each other).')
    cells = list(itertools.product(args.levels, args.slots))
    combos = [dict(source_level=s[0], source_slot=s[1], target_level=t[0], target_slot=t[1])
              for s, t in itertools.product(cells, cells)]
    if not args.all:
        random.Random(args.seed).shuffle(combos)
        combos = combos[:args.random]
    tasks = [dict(c) for c in combos for _ in range(args.repeat)]
    for i, t in enumerate(tasks, 1):
        t.update(index=i, count=len(tasks))

    if args.dry_run:
        for t in tasks:
            print(f"{t['index']:3d}: L{t['source_level']} {t['source_slot']:6} -> "
                  f"L{t['target_level']} {t['target_slot']}")
        return
    out = args.out or os.path.expanduser(
        f"~/fr3_batch/{datetime.now().strftime('%Y%m%d-%H%M%S')}")
    os.makedirs(out, exist_ok=True)
    csv_path = os.path.join(out, 'results.csv')
    print(f"{len(tasks)} runs, results in {out}", flush=True)
    rows = []
    fields = None
    for task in tasks:
        row = run_task(task, args, out)
        rows.append(row)
        if fields is None:
            fields = list(row.keys())
            with open(csv_path, 'w', newline='') as f:
                csv.DictWriter(f, fields).writeheader()
        with open(csv_path, 'a', newline='') as f:
            csv.DictWriter(f, fields).writerow(row)
        if not wait_until_idle(60.0):
            print('WARNING: simulation processes still running after 60 s; '
                  'stopping the batch to avoid overlapping runs.', flush=True)
            break
        if shutil.which('fastdds'):
            subprocess.run(['fastdds', 'shm', 'clean'], capture_output=True)
    summarise(rows)
    print(f"\nCSV: {csv_path}")


if __name__ == '__main__':
    main()

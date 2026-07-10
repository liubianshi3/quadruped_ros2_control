#!/usr/bin/env python3
"""Run repeatable Dog2 LAV1 trials and aggregate their reports."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Iterable

from dog2_bringup.acceptance_metrics import atomic_write_json


def _flatten(prefix: str, value: object, output: dict[str, object]) -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            _flatten(child_prefix, child, output)
    elif isinstance(value, (str, int, float, bool)) or value is None:
        output[prefix] = value


def _finite_numbers(values: Iterable[object]) -> list[float]:
    result = []
    for value in values:
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            number = float(value)
            if math.isfinite(number):
                result.append(number)
    return result


def _run_process(command: list[str], timeout_sec: float) -> tuple[int, bool]:
    process = subprocess.Popen(command, start_new_session=True)
    timed_out = False
    try:
        return_code = process.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        timed_out = True
        os.killpg(process.pid, signal.SIGTERM)
        try:
            return_code = process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            return_code = process.wait()
    return return_code, timed_out


def _git_metadata() -> dict:
    try:
        revision = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                ["git", "status", "--porcelain"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
        )
        return {"revision": revision, "dirty": dirty}
    except Exception:
        return {"revision": None, "dirty": None}


def _load_report(path: Path, trial_id: str, timed_out: bool) -> dict:
    if path.exists():
        with path.open("r", encoding="utf-8") as handle:
            return json.load(handle)
    return {
        "schema_version": 1,
        "protocol_id": "dog2_lav1_flat",
        "trial_id": trial_id,
        "status": "FAIL_INFRASTRUCTURE",
        "passed": False,
        "message": (
            "batch wrapper timeout before report"
            if timed_out
            else "checker exited without writing a report"
        ),
        "failure": {
            "code": "BATCH_TIMEOUT" if timed_out else "MISSING_REPORT",
            "provenance": "TEST_INFRASTRUCTURE",
        },
        "metrics": {},
    }


def _write_trials_csv(path: Path, reports: list[dict]) -> None:
    rows = []
    fields = {"trial_id", "status", "passed", "failure_code"}
    for report in reports:
        flattened: dict[str, object] = {}
        _flatten("metrics", report.get("metrics", {}), flattened)
        row = {
            "trial_id": report.get("trial_id"),
            "status": report.get("status"),
            "passed": report.get("passed"),
            "failure_code": (report.get("failure") or {}).get("code"),
            **flattened,
        }
        rows.append(row)
        fields.update(row)
    ordered_fields = [
        "trial_id",
        "status",
        "passed",
        "failure_code",
        *sorted(fields - {"trial_id", "status", "passed", "failure_code"}),
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=ordered_fields)
        writer.writeheader()
        writer.writerows(rows)


def _aggregate(reports: list[dict], commands: list[dict]) -> dict:
    metric_rows = []
    for report in reports:
        flattened: dict[str, object] = {}
        _flatten("", report.get("metrics", {}), flattened)
        metric_rows.append(flattened)

    metric_names = sorted({key for row in metric_rows for key in row})
    metric_stats = {}
    for name in metric_names:
        values = _finite_numbers(row.get(name) for row in metric_rows)
        if not values:
            continue
        metric_stats[name] = {
            "count": len(values),
            "mean": statistics.fmean(values),
            "std": statistics.pstdev(values),
            "min": min(values),
            "max": max(values),
        }

    failure_codes = Counter(
        str((report.get("failure") or {}).get("code", "UNKNOWN"))
        for report in reports
        if not report.get("passed", False)
    )
    passed = sum(bool(report.get("passed", False)) for report in reports)
    return {
        "schema_version": 1,
        "protocol_id": "dog2_lav1_flat",
        "trial_count": len(reports),
        "passed_count": passed,
        "success_rate": passed / len(reports) if reports else 0.0,
        "status_counts": dict(Counter(str(r.get("status")) for r in reports)),
        "failure_code_counts": dict(failure_codes),
        "metric_statistics": metric_stats,
        "git": _git_metadata(),
        "commands": commands,
        "reports": reports,
    }


def _parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run repeated Dog2 LAV1 headless trials."
    )
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--domain-start", type=int, default=64)
    parser.add_argument(
        "--result-dir",
        type=Path,
        default=Path("/tmp/dog2_locomotion_acceptance_batch"),
    )
    parser.add_argument(
        "--model-variant", choices=("real", "symmetric"), default="symmetric"
    )
    parser.add_argument("--use-gui", action="store_true")
    parser.add_argument("--trial-timeout-sec", type=float, default=240.0)
    parser.add_argument(
        "--launch-arg",
        action="append",
        default=[],
        help="Additional ros2 launch argument in name:=value form.",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)
    if args.trials < 1:
        raise SystemExit("--trials must be at least 1")
    args.result_dir.mkdir(parents=True, exist_ok=True)

    reports = []
    commands = []
    for index in range(args.trials):
        trial_id = f"trial_{index + 1:03d}"
        domain_id = args.domain_start + index
        result_json = args.result_dir / f"{trial_id}.json"
        samples_csv = args.result_dir / f"{trial_id}_samples.csv"
        junit_xml = args.result_dir / f"{trial_id}.junit.xml"
        command = [
            "ros2",
            "launch",
            "dog2_bringup",
            "locomotion_acceptance_test.launch.py",
            f"ros_domain_id:={domain_id}",
            f"trial_id:={trial_id}",
            f"model_variant:={args.model_variant}",
            f"use_gui:={'true' if args.use_gui else 'false'}",
            f"result_json:={result_json}",
            f"samples_csv:={samples_csv}",
            f"junit_xml:={junit_xml}",
            *args.launch_arg,
        ]
        started = time.time()
        print(f"[LAV1] starting {trial_id} domain={domain_id}", flush=True)
        return_code, timed_out = _run_process(command, args.trial_timeout_sec)
        report = _load_report(result_json, trial_id, timed_out)
        reports.append(report)
        commands.append(
            {
                "trial_id": trial_id,
                "command": command,
                "process_return_code": return_code,
                "timed_out": timed_out,
                "wall_duration_sec": time.time() - started,
            }
        )
        print(
            f"[LAV1] {trial_id}: {report.get('status')} "
            f"{(report.get('failure') or {}).get('code', '')}",
            flush=True,
        )

    aggregate = _aggregate(reports, commands)
    aggregate_json = args.result_dir / "aggregate.json"
    trials_csv = args.result_dir / "aggregate_trials.csv"
    atomic_write_json(aggregate_json, aggregate)
    _write_trials_csv(trials_csv, reports)
    print(
        f"[LAV1] success_rate={aggregate['success_rate']:.3f} "
        f"aggregate={aggregate_json}",
        flush=True,
    )
    return 0 if aggregate["passed_count"] == aggregate["trial_count"] else 1


if __name__ == "__main__":
    sys.exit(main())

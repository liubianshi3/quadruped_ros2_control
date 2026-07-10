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
import uuid
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

from dog2_bringup.acceptance_metrics import (
    atomic_write_json,
    percentile,
    sha256_file,
)


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


def _signal_process_group(process_group: int, sig: signal.Signals) -> bool:
    try:
        os.killpg(process_group, sig)
        return True
    except ProcessLookupError:
        return False


def _process_group_exists(process_group: int) -> bool:
    try:
        os.killpg(process_group, 0)
        return True
    except ProcessLookupError:
        return False


def _cleanup_process_group(process_group: int, grace_sec: float = 2.0) -> None:
    if not _signal_process_group(process_group, signal.SIGTERM):
        return
    deadline = time.monotonic() + max(0.0, grace_sec)
    while time.monotonic() < deadline:
        time.sleep(0.05)
        if not _process_group_exists(process_group):
            return
    _signal_process_group(process_group, signal.SIGKILL)


def _run_process(command: list[str], timeout_sec: float) -> tuple[int, bool]:
    process = subprocess.Popen(command, start_new_session=True)
    timed_out = False
    try:
        try:
            return_code = process.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            timed_out = True
            _signal_process_group(process.pid, signal.SIGTERM)
            try:
                return_code = process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                _signal_process_group(process.pid, signal.SIGKILL)
                return_code = process.wait()
    finally:
        # ros2 launch can exit after its Ruby `ign gazebo` wrapper while the
        # simulator child remains alive. Always reap the complete trial group
        # so later trials do not inherit CPU load or stale controllers.
        _cleanup_process_group(process.pid)
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


def _infrastructure_report(
    trial_id: str,
    run_uuid: str,
    code: str,
    message: str,
    *,
    measured: object = None,
) -> dict:
    return {
        "schema_version": 2,
        "protocol_id": "dog2_lav1_flat",
        "run_uuid": run_uuid,
        "trial_id": trial_id,
        "status": "FAIL_INFRASTRUCTURE",
        "passed": False,
        "provisional_pass": False,
        "artifact_complete": False,
        "message": message,
        "failure": {
            "code": code,
            "message": message,
            "measured": measured,
            "provenance": "TEST_INFRASTRUCTURE",
        },
        "metrics": {},
    }


def _load_report(
    path: Path,
    trial_id: str,
    run_uuid: str,
    *,
    started_ns: int,
    timed_out: bool,
    process_return_code: int,
) -> dict:
    if timed_out:
        return _infrastructure_report(
            trial_id,
            run_uuid,
            "BATCH_TIMEOUT",
            "batch wrapper timeout before a trustworthy report was committed",
        )
    if not path.exists():
        return _infrastructure_report(
            trial_id,
            run_uuid,
            "MISSING_REPORT",
            "checker exited without writing a report",
        )
    # Some filesystems expose coarse mtimes. UUID / trial identity is the
    # primary freshness proof; allow one second of timestamp quantization.
    if path.stat().st_mtime_ns + 1_000_000_000 < started_ns:
        return _infrastructure_report(
            trial_id,
            run_uuid,
            "STALE_REPORT",
            "report predates this trial invocation",
            measured=path.stat().st_mtime_ns,
        )
    try:
        with path.open("r", encoding="utf-8") as handle:
            report = json.load(handle)
    except Exception as exc:
        return _infrastructure_report(
            trial_id,
            run_uuid,
            "INVALID_REPORT_JSON",
            f"could not decode checker report: {exc}",
        )

    identity_errors = []
    if report.get("schema_version") != 2:
        identity_errors.append("schema_version")
    if report.get("run_uuid") != run_uuid:
        identity_errors.append("run_uuid")
    if report.get("trial_id") != trial_id:
        identity_errors.append("trial_id")
    if report.get("artifact_complete") is not True:
        identity_errors.append("artifact_complete")
    passed = bool(report.get("passed", False))
    provisional = bool(report.get("provisional_pass", False))
    status = str(report.get("status", ""))
    if passed != (status == "PASS_LOCOMOTION_BASELINE"):
        identity_errors.append("passed/status")
    if provisional != (status == "PASS_SAFETY_ROUTE_PROVISIONAL"):
        identity_errors.append("provisional/status")
    if passed and process_return_code != 0:
        identity_errors.append("pass/process_return_code")

    for name, artifact in (report.get("artifacts") or {}).items():
        artifact_path = Path(str((artifact or {}).get("path", "")))
        expected_hash = str((artifact or {}).get("sha256", ""))
        if (
            not artifact_path.is_file()
            or artifact_path.resolve().parent != path.resolve().parent
            or not expected_hash
            or sha256_file(artifact_path) != expected_hash
        ):
            identity_errors.append(f"artifact:{name}")
    if set((report.get("artifacts") or {})) != {"samples_csv", "junit_xml"}:
        identity_errors.append("artifact_set")

    if identity_errors:
        return _infrastructure_report(
            trial_id,
            run_uuid,
            "REPORT_INTEGRITY",
            "checker report failed identity or artifact validation",
            measured=sorted(identity_errors),
        )
    return report


def _write_trials_csv(path: Path, reports: list[dict]) -> None:
    rows = []
    fields = {
        "run_uuid",
        "trial_id",
        "status",
        "passed",
        "provisional_pass",
        "failure_code",
    }
    for report in reports:
        flattened: dict[str, object] = {}
        _flatten("metrics", report.get("metrics", {}), flattened)
        row = {
            "run_uuid": report.get("run_uuid"),
            "trial_id": report.get("trial_id"),
            "status": report.get("status"),
            "passed": report.get("passed"),
            "provisional_pass": report.get("provisional_pass"),
            "failure_code": (report.get("failure") or {}).get("code"),
            **flattened,
        }
        rows.append(row)
        fields.update(row)
    ordered_fields = [
        "run_uuid",
        "trial_id",
        "status",
        "passed",
        "provisional_pass",
        "failure_code",
        *sorted(
            fields
            - {
                "run_uuid",
                "trial_id",
                "status",
                "passed",
                "provisional_pass",
                "failure_code",
            }
        ),
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=ordered_fields)
        writer.writeheader()
        writer.writerows(rows)


def _aggregate(
    reports: list[dict], commands: list[dict], *, run_uuid: str
) -> dict:
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
            "p95": percentile(values, 0.95),
            "max": max(values),
        }

    failure_codes = Counter(
        str((report.get("failure") or {}).get("code", "UNKNOWN"))
        for report in reports
        if str(report.get("status", "")).startswith("FAIL_")
    )
    final_passed = sum(bool(report.get("passed", False)) for report in reports)
    provisional = sum(
        bool(report.get("provisional_pass", False)) for report in reports
    )
    infrastructure_failed = sum(
        report.get("status") == "FAIL_INFRASTRUCTURE" for report in reports
    )
    locomotion_failed = sum(
        report.get("status") == "FAIL_LOCOMOTION" for report in reports
    )
    return {
        "schema_version": 2,
        "protocol_id": "dog2_lav1_flat",
        "run_uuid": run_uuid,
        "trial_count": len(reports),
        "passed_count": final_passed,
        "provisional_passed_count": provisional,
        "locomotion_failed_count": locomotion_failed,
        "infrastructure_failed_count": infrastructure_failed,
        "success_rate": final_passed / len(reports) if reports else 0.0,
        "provisional_rate": provisional / len(reports) if reports else 0.0,
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
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--domain-start", type=int, default=64)
    parser.add_argument(
        "--run-uuid",
        default=None,
        help="Optional UUID for reproducible harness tests; generated by default.",
    )
    parser.add_argument(
        "--result-dir",
        type=Path,
        default=Path("/tmp/dog2_locomotion_acceptance_batch"),
    )
    parser.add_argument(
        "--model-variant", choices=("real", "symmetric"), default="symmetric"
    )
    parser.add_argument("--use-gui", action="store_true")
    parser.add_argument("--trial-timeout-sec", type=float, default=360.0)
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
    if args.domain_start < 0 or args.domain_start + args.trials - 1 > 232:
        raise SystemExit("ROS_DOMAIN_ID range must remain within 0..232")
    try:
        run_uuid = str(uuid.UUID(args.run_uuid)) if args.run_uuid else str(uuid.uuid4())
    except ValueError as exc:
        raise SystemExit(f"--run-uuid must be a UUID: {exc}") from exc

    args.result_dir.mkdir(parents=True, exist_ok=True)
    run_stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = args.result_dir / f"run_{run_stamp}_{run_uuid[:8]}"
    run_dir.mkdir(parents=False, exist_ok=False)

    reports = []
    commands = []
    for index in range(args.trials):
        trial_id = f"trial_{index + 1:03d}"
        domain_id = args.domain_start + index
        result_json = run_dir / f"{trial_id}.json"
        samples_csv = run_dir / f"{trial_id}_samples.csv"
        junit_xml = run_dir / f"{trial_id}.junit.xml"
        transport_partition = f"dog2_lav1_{run_uuid.replace('-', '')}_{index + 1:03d}"
        command = [
            "ros2",
            "launch",
            "dog2_bringup",
            "locomotion_acceptance_test.launch.py",
            f"ros_domain_id:={domain_id}",
            f"trial_id:={trial_id}",
            f"run_uuid:={run_uuid}",
            f"transport_partition:={transport_partition}",
            f"model_variant:={args.model_variant}",
            f"use_gui:={'true' if args.use_gui else 'false'}",
            f"result_json:={result_json}",
            f"samples_csv:={samples_csv}",
            f"junit_xml:={junit_xml}",
            *args.launch_arg,
        ]
        started_monotonic = time.monotonic()
        started_ns = time.time_ns()
        started_utc = datetime.now(timezone.utc).isoformat()
        print(
            f"[LAV1] starting {trial_id} domain={domain_id} "
            f"partition={transport_partition}",
            flush=True,
        )
        return_code, timed_out = _run_process(command, args.trial_timeout_sec)
        report = _load_report(
            result_json,
            trial_id,
            run_uuid,
            started_ns=started_ns,
            timed_out=timed_out,
            process_return_code=return_code,
        )
        reports.append(report)
        commands.append(
            {
                "run_uuid": run_uuid,
                "trial_id": trial_id,
                "started_utc": started_utc,
                "transport_partition": transport_partition,
                "command": command,
                "process_return_code": return_code,
                "timed_out": timed_out,
                "wall_duration_sec": time.monotonic() - started_monotonic,
            }
        )
        print(
            f"[LAV1] {trial_id}: {report.get('status')} "
            f"{(report.get('failure') or {}).get('code', '')}",
            flush=True,
        )

    aggregate = _aggregate(reports, commands, run_uuid=run_uuid)
    aggregate_json = run_dir / "aggregate.json"
    trials_csv = run_dir / "aggregate_trials.csv"
    _write_trials_csv(trials_csv, reports)
    aggregate["artifacts"] = {
        "trials_csv": {
            "path": str(trials_csv),
            "sha256": sha256_file(trials_csv),
        }
    }
    atomic_write_json(aggregate_json, aggregate)
    print(
        f"[LAV1] success_rate={aggregate['success_rate']:.3f} "
        f"provisional_rate={aggregate['provisional_rate']:.3f} "
        f"aggregate={aggregate_json}",
        flush=True,
    )
    if aggregate["infrastructure_failed_count"]:
        return 2
    if aggregate["locomotion_failed_count"]:
        return 1
    if aggregate["passed_count"] == aggregate["trial_count"]:
        return 0
    return 3


if __name__ == "__main__":
    sys.exit(main())

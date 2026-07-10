import json
import os
import sys
import time
import uuid
from pathlib import Path

from dog2_bringup.acceptance_metrics import sha256_file
from dog2_bringup.locomotion_acceptance_batch import (
    _aggregate,
    _load_report,
    _run_process,
)


def _write_report(
    directory: Path,
    *,
    run_uuid: str,
    trial_id: str = "trial_001",
    status: str = "PASS_SAFETY_ROUTE_PROVISIONAL",
) -> Path:
    samples = directory / f"{trial_id}_samples.csv"
    junit = directory / f"{trial_id}.junit.xml"
    samples.write_text("time_sec,stage\n0,STAND\n", encoding="utf-8")
    junit.write_text("<testsuite tests='1'/>", encoding="utf-8")
    report = {
        "schema_version": 2,
        "protocol_id": "dog2_lav1_flat",
        "run_uuid": run_uuid,
        "trial_id": trial_id,
        "status": status,
        "passed": status == "PASS_LOCOMOTION_BASELINE",
        "provisional_pass": status == "PASS_SAFETY_ROUTE_PROVISIONAL",
        "artifact_complete": True,
        "failure": None,
        "metrics": {"distance": 1.0},
        "artifacts": {
            "samples_csv": {
                "path": str(samples),
                "sha256": sha256_file(samples),
            },
            "junit_xml": {
                "path": str(junit),
                "sha256": sha256_file(junit),
            },
        },
    }
    path = directory / f"{trial_id}.json"
    path.write_text(json.dumps(report), encoding="utf-8")
    return path


def test_batch_accepts_only_current_complete_identity(tmp_path: Path) -> None:
    run_uuid = str(uuid.uuid4())
    started_ns = time.time_ns()
    path = _write_report(tmp_path, run_uuid=run_uuid)
    report = _load_report(
        path,
        "trial_001",
        run_uuid,
        started_ns=started_ns,
        timed_out=False,
        process_return_code=0,
    )
    assert report["status"] == "PASS_SAFETY_ROUTE_PROVISIONAL"


def test_batch_rejects_stale_and_wrong_uuid_reports(tmp_path: Path) -> None:
    run_uuid = str(uuid.uuid4())
    path = _write_report(tmp_path, run_uuid=str(uuid.uuid4()))
    wrong_identity = _load_report(
        path,
        "trial_001",
        run_uuid,
        started_ns=path.stat().st_mtime_ns - 1,
        timed_out=False,
        process_return_code=0,
    )
    assert wrong_identity["failure"]["code"] == "REPORT_INTEGRITY"

    old_ns = time.time_ns() - 10_000_000_000
    os.utime(path, ns=(old_ns, old_ns))
    stale = _load_report(
        path,
        "trial_001",
        run_uuid,
        started_ns=time.time_ns(),
        timed_out=False,
        process_return_code=0,
    )
    assert stale["failure"]["code"] == "STALE_REPORT"


def test_batch_rejects_artifact_tampering(tmp_path: Path) -> None:
    run_uuid = str(uuid.uuid4())
    started_ns = time.time_ns()
    path = _write_report(tmp_path, run_uuid=run_uuid)
    (tmp_path / "trial_001_samples.csv").write_text("tampered\n", encoding="utf-8")
    report = _load_report(
        path,
        "trial_001",
        run_uuid,
        started_ns=started_ns,
        timed_out=False,
        process_return_code=0,
    )
    assert report["failure"]["code"] == "REPORT_INTEGRITY"
    assert "artifact:samples_csv" in report["failure"]["measured"]


def test_aggregate_separates_final_provisional_and_failures() -> None:
    run_uuid = str(uuid.uuid4())
    reports = [
        {
            "status": "PASS_LOCOMOTION_BASELINE",
            "passed": True,
            "provisional_pass": False,
            "failure": None,
            "metrics": {"x": 1.0},
        },
        {
            "status": "PASS_SAFETY_ROUTE_PROVISIONAL",
            "passed": False,
            "provisional_pass": True,
            "failure": None,
            "metrics": {"x": 2.0},
        },
        {
            "status": "FAIL_INFRASTRUCTURE",
            "passed": False,
            "provisional_pass": False,
            "failure": {"code": "STALE_REPORT"},
            "metrics": {},
        },
    ]
    aggregate = _aggregate(reports, [], run_uuid=run_uuid)
    assert aggregate["passed_count"] == 1
    assert aggregate["provisional_passed_count"] == 1
    assert aggregate["infrastructure_failed_count"] == 1
    assert aggregate["success_rate"] == 1 / 3
    assert aggregate["metric_statistics"]["x"]["p95"] == 1.95


def test_run_process_reaps_descendants_after_parent_exits(
    tmp_path: Path,
) -> None:
    child_pid_path = tmp_path / "child.pid"
    script = (
        "import pathlib, subprocess, sys; "
        "child = subprocess.Popen(['sleep', '60']); "
        "pathlib.Path(sys.argv[1]).write_text(str(child.pid))"
    )

    return_code, timed_out = _run_process(
        [sys.executable, "-c", script, str(child_pid_path)], 5.0
    )

    assert return_code == 0
    assert not timed_out
    child_pid = int(child_pid_path.read_text())
    deadline = time.monotonic() + 2.0
    while Path(f"/proc/{child_pid}").exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not Path(f"/proc/{child_pid}").exists()

#!/usr/bin/env python3
"""Build a reviewable quality-threshold candidate from labeled LAV2 reports."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Iterable

from dog2_bringup.acceptance_metrics import (
    atomic_write_json,
    percentile,
    sha256_file,
    sha256_text,
)


METRICS = {
    "max_straight_heading_error_rad": (
        "metrics.straight_heading_error_abs_rad.max",
        "max",
    ),
    "max_linear_velocity_error_rms_mps": (
        "metrics.linear_velocity_error_mps.rms",
        "max",
    ),
    "max_contact_mismatch_ratio": (
        "metrics.foot.contact_mismatch_ratio",
        "max",
    ),
    "max_stance_slip_per_base_path": (
        "metrics.stance_slip_per_base_path",
        "max",
    ),
    "min_swing_peak_clearance_m": (
        "metrics.foot.swing_peak_clearance_min_m",
        "min",
    ),
}


def _nested(report: dict, dotted_path: str) -> object:
    value: object = report
    for key in dotted_path.split("."):
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def _finite(values: Iterable[object]) -> list[float]:
    result = []
    for value in values:
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            number = float(value)
            if math.isfinite(number):
                result.append(number)
    return result


def calibrate_metric(
    acceptable: list[float],
    rejected: list[float],
    *,
    direction: str,
    minimum_per_class: int,
) -> dict:
    result = {
        "direction": direction,
        "acceptable_count": len(acceptable),
        "rejected_count": len(rejected),
        "acceptable_p95": percentile(acceptable, 0.95),
        "rejected_p05": percentile(rejected, 0.05),
        "candidate_threshold": None,
        "separable": False,
    }
    if len(acceptable) < minimum_per_class or len(rejected) < minimum_per_class:
        result["reason"] = "insufficient_labeled_samples"
        return result
    if direction == "max":
        acceptable_edge = max(acceptable)
        rejected_edge = min(rejected)
        separable = acceptable_edge < rejected_edge
    elif direction == "min":
        acceptable_edge = min(acceptable)
        rejected_edge = max(rejected)
        separable = acceptable_edge > rejected_edge
    else:
        raise ValueError(f"unknown calibration direction: {direction}")
    result["acceptable_worst"] = acceptable_edge
    result["rejected_best"] = rejected_edge
    result["separable"] = separable
    if separable:
        result["candidate_threshold"] = (acceptable_edge + rejected_edge) / 2.0
    else:
        result["reason"] = "class_overlap_requires_more_data_or_feature_review"
    return result


def build_candidate(
    labeled_reports: list[tuple[str, Path, dict]],
    *,
    minimum_per_class: int,
) -> dict:
    metrics = {}
    for parameter, (path, direction) in METRICS.items():
        acceptable = _finite(
            _nested(report, path)
            for label, _, report in labeled_reports
            if label == "accept"
        )
        rejected = _finite(
            _nested(report, path)
            for label, _, report in labeled_reports
            if label == "reject"
        )
        metrics[parameter] = calibrate_metric(
            acceptable,
            rejected,
            direction=direction,
            minimum_per_class=minimum_per_class,
        )
    source_reports = [
        {
            "label": label,
            "path": str(path),
            "sha256": sha256_file(path),
            "run_uuid": report.get("run_uuid"),
            "trial_id": report.get("trial_id"),
        }
        for label, path, report in labeled_reports
    ]
    candidate_ready = all(metric["separable"] for metric in metrics.values())
    profile_material = {
        "minimum_per_class": minimum_per_class,
        "thresholds": {
            name: metric["candidate_threshold"] for name, metric in metrics.items()
        },
        "source_sha256": [source["sha256"] for source in source_reports],
    }
    candidate_profile_id = (
        f"lav-quality-{sha256_text(json.dumps(profile_material, sort_keys=True))[:16]}"
        if candidate_ready
        else None
    )
    return {
        "schema_version": 1,
        # A tool can only produce a candidate. Human review and an independent
        # holdout run are required before config may say "calibrated".
        "profile_status": "candidate" if candidate_ready else "insufficient_data",
        "candidate_profile_id": candidate_profile_id,
        "human_review_required": True,
        "holdout_validation_required": True,
        "minimum_per_class": minimum_per_class,
        "labeled_report_count": len(labeled_reports),
        "metrics": metrics,
        "source_reports": source_reports,
    }


def _load_manifest(path: Path) -> list[tuple[str, Path, dict]]:
    rows = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if not {"report_json", "label"}.issubset(reader.fieldnames or []):
            raise ValueError("manifest requires report_json,label columns")
        for index, row in enumerate(reader, start=2):
            label = str(row["label"]).strip().lower()
            if label not in {"accept", "reject"}:
                raise ValueError(f"manifest line {index}: invalid label {label!r}")
            report_path = Path(str(row["report_json"]))
            if not report_path.is_absolute():
                report_path = path.parent / report_path
            with report_path.open("r", encoding="utf-8") as report_handle:
                report = json.load(report_handle)
            if (
                report.get("schema_version") != 2
                or report.get("artifact_complete") is not True
            ):
                raise ValueError(
                    f"manifest line {index}: report is not complete schema v2"
                )
            rows.append((label, report_path.resolve(), report))
    return rows


def _parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a review-only LAV quality calibration candidate."
    )
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minimum-per-class", type=int, default=10)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)
    if args.minimum_per_class < 2:
        raise SystemExit("--minimum-per-class must be at least 2")
    try:
        labeled_reports = _load_manifest(args.manifest)
        candidate = build_candidate(
            labeled_reports,
            minimum_per_class=args.minimum_per_class,
        )
        atomic_write_json(args.output, candidate)
    except Exception as exc:
        print(f"calibration failed: {exc}", file=sys.stderr)
        return 2
    print(f"profile_status={candidate['profile_status']} output={args.output}")
    return 0 if candidate["profile_status"] == "candidate" else 1


if __name__ == "__main__":
    sys.exit(main())

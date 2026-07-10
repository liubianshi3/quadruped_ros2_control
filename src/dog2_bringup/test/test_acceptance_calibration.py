import csv
import json

from dog2_bringup.locomotion_acceptance_calibrate import (
    _load_manifest,
    build_candidate,
    calibrate_metric,
)


def _report(value: float, *, accepted: bool) -> dict:
    scale = value if accepted else value + 1.0
    clearance = 0.10 + value if accepted else 0.01 + value
    return {
        "schema_version": 2,
        "artifact_complete": True,
        "run_uuid": f"run-{accepted}-{value}",
        "trial_id": f"trial-{value}",
        "metrics": {
            "straight_heading_error_abs_rad": {"max": scale},
            "linear_velocity_error_mps": {"rms": scale},
            "foot": {
                "contact_mismatch_ratio": scale,
                "swing_peak_clearance_min_m": clearance,
            },
            "stance_slip_per_base_path": scale,
        },
    }


def test_calibrate_metric_requires_samples_and_class_separation():
    insufficient = calibrate_metric(
        [0.1],
        [0.9],
        direction="max",
        minimum_per_class=2,
    )
    assert insufficient["separable"] is False
    assert insufficient["reason"] == "insufficient_labeled_samples"

    overlap = calibrate_metric(
        [0.1, 0.8],
        [0.4, 0.9],
        direction="max",
        minimum_per_class=2,
    )
    assert overlap["separable"] is False
    assert overlap["reason"] == "class_overlap_requires_more_data_or_feature_review"

    separated = calibrate_metric(
        [0.1, 0.2],
        [0.8, 0.9],
        direction="max",
        minimum_per_class=2,
    )
    assert separated["separable"] is True
    assert separated["candidate_threshold"] == 0.5


def test_build_candidate_never_claims_calibrated(tmp_path):
    labeled_reports = []
    for accepted in (True, False):
        for index in range(2):
            value = index * 0.001
            report = _report(value, accepted=accepted)
            path = tmp_path / f"{accepted}-{index}.json"
            path.write_text(json.dumps(report), encoding="utf-8")
            labeled_reports.append(
                ("accept" if accepted else "reject", path, report)
            )

    candidate = build_candidate(labeled_reports, minimum_per_class=2)

    assert candidate["profile_status"] == "candidate"
    assert candidate["candidate_profile_id"].startswith("lav-quality-")
    assert candidate["human_review_required"] is True
    assert candidate["holdout_validation_required"] is True
    assert all(
        metric["separable"] for metric in candidate["metrics"].values()
    )
    assert all(source["sha256"] for source in candidate["source_reports"])


def test_manifest_rejects_incomplete_report(tmp_path):
    report_path = tmp_path / "report.json"
    report_path.write_text(
        json.dumps({"schema_version": 2, "artifact_complete": False}),
        encoding="utf-8",
    )
    manifest_path = tmp_path / "manifest.csv"
    with manifest_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["report_json", "label"])
        writer.writeheader()
        writer.writerow({"report_json": report_path.name, "label": "accept"})

    try:
        _load_manifest(manifest_path)
    except ValueError as exc:
        assert "not complete schema v2" in str(exc)
    else:
        raise AssertionError("incomplete report should be rejected")

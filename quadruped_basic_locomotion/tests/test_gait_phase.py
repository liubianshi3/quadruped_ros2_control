from pathlib import Path
import sys

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controllers.gait_scheduler import GaitScheduler


def load_gait_scheduler() -> GaitScheduler:
    config_path = PROJECT_ROOT / "config" / "gait_config.yaml"
    with config_path.open("r", encoding="utf-8") as f:
        gait_config = yaml.safe_load(f) or {}
    return GaitScheduler.from_config_dict(gait_config)


def test_configured_phase_pattern_matches_gait_type():
    scheduler = load_gait_scheduler()
    state = scheduler.update(scheduler.config.initial_stand_time + 0.35)
    gait_type = scheduler.config.gait_type.lower()
    if gait_type == "trot":
        assert abs(state["FL"]["phase"] - state["RR"]["phase"]) < 1e-9
        assert abs(state["FR"]["phase"] - state["RL"]["phase"]) < 1e-9
    else:
        phases = sorted(round(state[leg]["phase"], 6) for leg in ("FL", "FR", "RL", "RR"))
        gaps = [round((phases[(i + 1) % 4] - phases[i]) % 1.0, 6) for i in range(4)]
        assert all(abs(gap - 0.25) < 1e-6 for gap in gaps)


def test_configured_gait_support_count():
    scheduler = load_gait_scheduler()
    assert scheduler.config.min_stance_legs >= 2
    if scheduler.config.gait_type.lower() in {"crawl", "walk"}:
        assert scheduler.config.min_stance_legs >= 3
        assert scheduler.config.duty_factor > 0.75


def test_at_least_configured_stance_legs():
    scheduler = load_gait_scheduler()
    for i in range(200):
        t = 2.0 + i * 0.01
        state = scheduler.update(t)
        stance_count = sum(1 for leg in state.values() if leg["state"] == "stance")
        assert stance_count >= scheduler.config.min_stance_legs


def test_crawl_has_no_overlapping_swing_legs():
    scheduler = load_gait_scheduler()
    if scheduler.config.gait_type.lower() not in {"crawl", "walk"}:
        return
    for i in range(300):
        t = scheduler.config.initial_stand_time + i * scheduler.config.period / 300.0
        state = scheduler.update(t)
        swing_count = sum(1 for leg in state.values() if leg["state"] == "swing")
        assert swing_count <= 1

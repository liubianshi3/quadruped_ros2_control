import xml.etree.ElementTree as ET
from pathlib import Path


def test_obstacle_world_exists():
    world = Path(__file__).resolve().parent.parent / "worlds" / "step_block.sdf"
    assert world.exists()


def test_window_world_exists():
    world = Path(__file__).resolve().parent.parent / "worlds" / "window_frame.sdf"
    assert world.exists()


def test_obstacle_worlds_enable_contact_system():
    worlds = Path(__file__).resolve().parent.parent / "worlds"
    expected_plugin = (
        "gz-sim-contact-system",
        "gz::sim::systems::Contact",
    )

    for filename in ("window_frame.sdf", "step_block.sdf"):
        world = ET.parse(worlds / filename).getroot().find("world")
        assert world is not None
        plugins = {
            (plugin.get("filename"), plugin.get("name"))
            for plugin in world.findall("plugin")
        }
        assert expected_plugin in plugins


def test_crossing_trial_launch_exists():
    launch_file = Path(__file__).resolve().parent.parent / "launch" / "crossing_trial.launch.py"
    assert launch_file.exists()


def test_crossing_trial_selects_complete_research_stack():
    launch_file = Path(__file__).resolve().parent.parent / "launch" / "crossing_trial.launch.py"
    launch_text = launch_file.read_text(encoding="utf-8")

    assert '"research_stack": "true"' in launch_text
    assert '"flat_locomotion": "false"' in launch_text


def test_window_crossing_assets_exist():
    root = Path(__file__).resolve().parent.parent
    assert (root / "launch" / "window_crossing_test.launch.py").exists()
    assert (root / "dog2_bringup" / "crossing_check.py").exists()


def test_locomotion_acceptance_assets_exist():
    root = Path(__file__).resolve().parent.parent
    assert (root / "launch" / "locomotion_acceptance_test.launch.py").exists()
    assert (root / "config" / "locomotion_acceptance.yaml").exists()
    assert (root / "dog2_bringup" / "locomotion_acceptance.py").exists()
    assert (root / "dog2_bringup" / "acceptance_oracle.py").exists()
    assert (root / "dog2_bringup" / "locomotion_acceptance_batch.py").exists()
    assert (root / "dog2_bringup" / "locomotion_acceptance_calibrate.py").exists()
    assert (root / "test" / "test_acceptance_oracle.py").exists()
    assert (root / "test" / "test_acceptance_batch.py").exists()
    assert (root / "test" / "test_acceptance_calibration.py").exists()


def test_locomotion_acceptance_launch_is_partitioned_without_global_pkill():
    root = Path(__file__).resolve().parent.parent
    launch_text = (
        root / "launch" / "locomotion_acceptance_test.launch.py"
    ).read_text(encoding="utf-8")
    assert "pkill" not in launch_text
    assert "GZ_PARTITION" in launch_text
    assert "IGN_PARTITION" in launch_text


def test_locomotion_acceptance_contact_event_contract_has_no_heartbeat_timeout():
    root = Path(__file__).resolve().parent.parent
    config_text = (root / "config" / "locomotion_acceptance.yaml").read_text(
        encoding="utf-8"
    )
    node_text = (
        root / "dog2_bringup" / "locomotion_acceptance.py"
    ).read_text(encoding="utf-8")

    assert "foot_contact_freshness_timeout_sec" not in config_text
    assert "foot_contact_freshness_timeout_sec" not in node_text


def test_contact_bridges_use_sensor_owned_transport_topics():
    root = Path(__file__).resolve().parent.parent
    launch_text = (
        root / "launch" / "effort_research_sim.launch.py"
    ).read_text(encoding="utf-8")
    assert 'f"/dog2/gz_contact/{leg}_foot"' in launch_text
    assert 'f"/dog2/gz_contact/{leg}_tibia"' in launch_text


def test_flat_locomotion_preshift_feedforward_is_enabled():
    root = Path(__file__).resolve().parent.parent
    config_text = (
        root / "config" / "flat_locomotion.yaml"
    ).read_text(encoding="utf-8")
    assert "body_shift_kp: 250.0" in config_text
    assert "body_shift_max_tilt: 0.15" in config_text


def test_flat_locomotion_support_target_keeps_15n_gate():
    root = Path(__file__).resolve().parent.parent
    config_text = (
        root / "config" / "flat_locomotion.yaml"
    ).read_text(encoding="utf-8")
    # The readiness gate must stay at 15 N; the pre-shift target may only
    # add margin on top of it, never replace or dilute it.
    assert "support_ready_min_normal_force: 15.0" in config_text
    assert "support_target_force_margin: 25.0" in config_text

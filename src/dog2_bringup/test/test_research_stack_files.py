from pathlib import Path


def test_obstacle_world_exists():
    world = Path(__file__).resolve().parent.parent / "worlds" / "step_block.sdf"
    assert world.exists()


def test_window_world_exists():
    world = Path(__file__).resolve().parent.parent / "worlds" / "window_frame.sdf"
    assert world.exists()


def test_crossing_trial_launch_exists():
    launch_file = Path(__file__).resolve().parent.parent / "launch" / "crossing_trial.launch.py"
    assert launch_file.exists()


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


def test_contact_bridges_use_sensor_owned_transport_topics():
    root = Path(__file__).resolve().parent.parent
    launch_text = (
        root / "launch" / "effort_research_sim.launch.py"
    ).read_text(encoding="utf-8")
    assert 'f"/dog2/gz_contact/{leg}_foot"' in launch_text
    assert 'f"/dog2/gz_contact/{leg}_tibia"' in launch_text

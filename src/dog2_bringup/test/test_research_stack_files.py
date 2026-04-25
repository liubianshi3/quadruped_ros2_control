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

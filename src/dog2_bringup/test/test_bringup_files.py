from pathlib import Path


def test_world_exists():
    world = Path(__file__).resolve().parent.parent / "worlds" / "flat_ground.sdf"
    assert world.exists()


def test_system_launch_exists():
    launch_file = Path(__file__).resolve().parent.parent / "launch" / "system.launch.py"
    assert launch_file.exists()

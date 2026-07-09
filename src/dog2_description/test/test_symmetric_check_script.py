#!/usr/bin/env python3
"""
Test: check_symmetric_urdf.py guardrail script passes for symmetric model.

This test runs the check script as a subprocess, ensuring CI catches any
regression in the symmetric model's leg mounts, trunk geometry, leg inertial
symmetry, mesh paths, foot primitives, and Python control parameter alignment.
"""

import subprocess
import sys
from pathlib import Path


def test_check_symmetric_urdf_script():
    """Run check_symmetric_urdf.py and assert zero exit code."""
    script = (
        Path(__file__).parent.parent
        / "scripts"
        / "check_symmetric_urdf.py"
    )
    assert script.exists(), f"Script not found: {script}"

    result = subprocess.run(
        [sys.executable, str(script)],
        capture_output=True,
        text=True,
        cwd=script.parent.parent.parent.parent,
    )

    # Print output for CI logs regardless of pass/fail
    print(result.stdout)
    if result.stderr:
        print(result.stderr)

    assert result.returncode == 0, (
        f"check_symmetric_urdf.py exited with code {result.returncode}\n"
        f"stdout:\n{result.stdout}\n"
        f"stderr:\n{result.stderr}"
    )


if __name__ == "__main__":
    test_check_symmetric_urdf_script()
    print("[PASS] Symmetric check script test passed.")

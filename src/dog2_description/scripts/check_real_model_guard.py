#!/usr/bin/env python3
"""Check that real-model check_urdf_shift_boundary expected values still match dog2.urdf.xacro."""
import sys
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))
import subprocess
subprocess.run([sys.executable, str(__import__("pathlib").Path(__file__).resolve().parents[1] / "scripts" / "check_urdf_shift_boundary.py"), "--strict"], check=True)
print("[PASS] Real model boundary checks still pass (strict).")

#!/usr/bin/env python3
"""Expand dog2.urdf.xacro to a standalone URDF file (migration baseline / CI diff)."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def _default_xacro_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("dog2_description")) / "urdf" / "dog2.urdf.xacro"
    except Exception:
        return Path(__file__).resolve().parent.parent / "urdf" / "dog2.urdf.xacro"


def _default_controllers_yaml() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        p = Path(get_package_share_directory("dog2_motion_control")) / "config" / "effort_controllers.yaml"
        if p.is_file():
            return str(p)
    except Exception:
        pass
    try:
        from ament_index_python.packages import get_package_share_directory

        p = Path(get_package_share_directory("dog2_description")) / "config" / "ros2_controllers.yaml"
        if p.is_file():
            return str(p)
    except Exception:
        pass
    return ""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-i",
        "--input",
        type=Path,
        default=None,
        help="Path to dog2.urdf.xacro (default: dog2_description share or source tree)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        required=True,
        help="Output URDF path",
    )
    parser.add_argument(
        "--controllers-yaml",
        type=str,
        default=None,
        help="controllers_yaml xacro arg (default: effort_controllers.yaml or ros2_controllers.yaml)",
    )
    args = parser.parse_args()

    xacro_path = args.input or _default_xacro_path()
    if not xacro_path.is_file():
        print(f"error: xacro not found: {xacro_path}", file=sys.stderr)
        return 1

    cy = args.controllers_yaml if args.controllers_yaml is not None else _default_controllers_yaml()
    cmd = ["xacro", str(xacro_path), "-o", str(args.output)]
    if cy:
        cmd.append(f"controllers_yaml:={cy}")

    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        print(proc.stderr or proc.stdout, file=sys.stderr)
        return proc.returncode

    print(f"wrote {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

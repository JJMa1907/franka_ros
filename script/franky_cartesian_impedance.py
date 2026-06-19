#!/usr/bin/env python3
"""Compatibility wrapper for the package-backed franky ROS bridge."""

from pathlib import Path
import runpy
import sys


def main() -> int:
    script_path = (
        Path(__file__).resolve().parents[1]
        / "franka_example_controllers"
        / "scripts"
        / "franky_cartesian_impedance_compat.py"
    )
    if not script_path.is_file():
        print(f"fatal: compatibility script not found: {script_path}", file=sys.stderr)
        return 1
    runpy.run_path(str(script_path), run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

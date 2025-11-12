#!/usr/bin/env python3
"""Generate cinematic camera waypoint JSON files for Story 1.4."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List


def parse_keyframe(keyframe: str) -> dict:
    parts = [p.strip() for p in keyframe.split(',') if p.strip()]
    if len(parts) != 8:
        raise ValueError(
            "Keyframes must provide 8 comma-separated values: time,x,y,z,roll,pitch,yaw,fov"
        )
    time, x, y, z, roll, pitch, yaw, fov = map(float, parts)
    return {
        "time_from_start": time,
        "position": {"x": x, "y": y, "z": z},
        "orientation_rpy": {"roll": roll, "pitch": pitch, "yaw": yaw},
        "fov_deg": fov,
        "focal_distance_m": 50.0,
    }


def build_default_path() -> List[dict]:
    return [
        {
            "time_from_start": 0.0,
            "position": {"x": -60.0, "y": 0.0, "z": 70.0},
            "orientation_rpy": {"roll": 0.0, "pitch": -0.5, "yaw": 0.0},
            "fov_deg": 55.0,
            "focal_distance_m": 60.0,
        },
        {
            "time_from_start": 4.0,
            "position": {"x": -10.0, "y": 20.0, "z": 45.0},
            "orientation_rpy": {"roll": 0.0, "pitch": -0.3, "yaw": 1.2},
            "fov_deg": 45.0,
            "focal_distance_m": 40.0,
        },
        {
            "time_from_start": 7.5,
            "position": {"x": 30.0, "y": -15.0, "z": 35.0},
            "orientation_rpy": {"roll": 0.0, "pitch": -0.2, "yaw": -1.0},
            "fov_deg": 35.0,
            "focal_distance_m": 30.0,
        },
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description="Camera path JSON generator")
    parser.add_argument(
        "--output",
        default="software/sim/config/camera_path_example.json",
        help="Path to save JSON output",
    )
    parser.add_argument(
        "--keyframe",
        action="append",
        help="Keyframe definition time,x,y,z,roll,pitch,yaw,fov (degrees)",
    )
    parser.add_argument(
        "--template",
        action="store_true",
        help="Write the default cinematic arc template",
    )
    args = parser.parse_args()

    if args.template and args.keyframe:
        raise SystemExit("Use either --template or --keyframe entries, not both")

    waypoints: List[dict]
    if args.template:
        waypoints = build_default_path()
    elif args.keyframe:
        waypoints = [parse_keyframe(frame) for frame in args.keyframe]
    else:
        raise SystemExit("Provide --template or at least one --keyframe")

    payload = {
        "metadata": {
            "version": 1,
            "description": "Cinematic camera path",
        },
        "waypoints": waypoints,
    }

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2))
    print(f"Camera path saved to {output_path}")


if __name__ == "__main__":
    main()

"""Helper to start MCAP-based rosbag recordings with storage safeguards.

This module provides functionality to record ROS 2 simulation sessions
using rosbag2 with configurable profiles, automatic metadata capture,
and storage size monitoring.

Typical usage example:
    # Record with default profile
    python record_sim_session.py --config profile.yaml

    # Record with duration limit
    python record_sim_session.py --duration 300 --topics /topic1 /topic2
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def load_profile(path: Path) -> dict[str, Any]:
    """Load and validate a recording profile from a JSON or YAML file.

    Args:
        path: Path to the profile configuration file.

    Returns:
        Parsed profile dictionary containing rosbag configuration.

    Raises:
        ValueError: If the file is not valid JSON or missing required sections.
    """
    try:
        data = json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        raise ValueError(f"Profile at {path} is not valid JSON/YAML: {exc}") from exc
    if "rosbag" not in data or "topics" not in data["rosbag"]:
        raise ValueError("Profile missing rosbag/topics section")
    return data


def ensure_dir(path: Path) -> None:
    """Ensure the output directory exists and is writable.

    Args:
        path: Path to the directory to create/verify.

    Raises:
        PermissionError: If the directory is not writable.
    """
    path.mkdir(parents=True, exist_ok=True)
    if not os.access(path, os.W_OK):
        raise PermissionError(f"Output directory {path} is not writable")


def compute_dir_size_mb(path: Path) -> float:
    """Compute the total size of all files in a directory in megabytes.

    Args:
        path: Root directory to scan.

    Returns:
        Total size in megabytes.
    """
    total = 0
    for file_path in path.rglob('*'):
        try:
            if file_path.is_file():
                total += file_path.stat().st_size
        except FileNotFoundError:
            continue
    return total / (1024 * 1024)


def build_rosbag_command(out_path: Path, profile: dict[str, Any], topics: list[str]) -> list[str]:
    """Build the rosbag2 record command with configured options.

    Args:
        out_path: Output path for the rosbag.
        profile: Recording profile dictionary with rosbag configuration.
        topics: List of topic names to record.

    Returns:
        List of command arguments for subprocess execution.
    """
    rosbag_cfg = profile.get("rosbag", {})
    cmd = ["ros2", "bag", "record", "-o", str(out_path)]
    storage = rosbag_cfg.get("storage_id")
    if storage:
        cmd += ["--storage", storage]
    compression_mode = rosbag_cfg.get("compression_mode")
    compression_format = rosbag_cfg.get("compression_format")
    if compression_mode and compression_format:
        cmd += ["--compression-mode", compression_mode, "--compression-format", compression_format]
    cmd += topics
    return cmd


def run_rosbag(cmd: list[str], duration: int | None) -> int:
    """Execute the rosbag recording process with signal handling.

    Args:
        cmd: Command list to execute.
        duration: Optional auto-stop duration in seconds.

    Returns:
        Process return code.
    """
    proc = subprocess.Popen(cmd)
    stop_event = threading.Event()

    def terminate(*_: Any) -> None:
        stop_event.set()
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)

    timer = None
    if duration:
        def auto_stop() -> None:
            time.sleep(duration)
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        timer = threading.Thread(target=auto_stop, daemon=True)
        timer.start()

    return_code = proc.wait()
    if timer and timer.is_alive():
        timer.join(timeout=1)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    return return_code


def snapshot_profile(profile_path: Path, session_dir: Path) -> None:
    """Copy the recording profile to the session directory for reference.

    Args:
        profile_path: Path to the original profile file.
        session_dir: Destination session directory.
    """
    shutil.copy(profile_path, session_dir / "profile_snapshot.yaml")


def write_metadata(session_dir: Path, metadata: dict[str, Any]) -> None:
    """Write session metadata to a JSON file.

    Args:
        session_dir: Directory to write the metadata file.
        metadata: Dictionary containing session information.
    """
    metadata_path = session_dir / "metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True))


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for the recording session.

    Returns:
        Parsed arguments namespace.
    """
    parser = argparse.ArgumentParser(description="Record Aeris simulation sessions with rosbag2")
    parser.add_argument(
        "--config",
        default="software/sim/config/recording_profile.yaml",
        help="Path to recording profile (JSON or YAML)",
    )
    parser.add_argument(
        "--duration",
        type=int,
        help="Automatically stop recording after N seconds",
    )
    parser.add_argument(
        "--topics",
        nargs='+',
        help="Override topics list (space-separated)",
    )
    parser.add_argument(
        "--output-dir",
        help="Override output directory (default from profile)",
    )
    parser.add_argument(
        "--prefix",
        help="Override bag name prefix (default from profile)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print command without executing",
    )
    return parser.parse_args()


def main() -> int:
    """Main entry point for the recording session.

    Returns:
        Exit code (0 for success, non-zero for errors).
    """
    args = parse_args()
    profile_path = Path(args.config).expanduser()
    if not profile_path.is_file():
        print(f"[record_sim_session] ERROR: Profile not found: {profile_path}", file=sys.stderr)
        return 1
    profile = load_profile(profile_path)

    topics = args.topics or profile["rosbag"].get("topics", [])
    if not topics:
        print("[record_sim_session] ERROR: No topics specified", file=sys.stderr)
        return 1

    output_dir = Path(args.output_dir or profile.get("output_dir", "log/recordings"))
    ensure_dir(output_dir)

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    prefix = args.prefix or profile.get("bag_name_prefix", "aeris_mission")
    session_dir = output_dir / f"{timestamp}_{prefix}"
    if session_dir.exists():
        raise FileExistsError(f"Session directory already exists: {session_dir}")
    ensure_dir(session_dir)
    bag_out = session_dir / prefix

    cmd = build_rosbag_command(bag_out, profile, topics)

    metadata = {
        "profile": str(profile_path),
        "session_dir": str(session_dir),
        "bag_output": str(bag_out),
        "topics": topics,
        "start_time": timestamp,
        "duration_limit_s": args.duration,
        "command": cmd,
    }

    if args.dry_run:
        print("[record_sim_session] Dry run: would execute")
        print(" ".join(cmd))
        write_metadata(session_dir, metadata | {"dry_run": True})
        snapshot_profile(profile_path, session_dir)
        return 0

    snapshot_profile(profile_path, session_dir)
    write_metadata(session_dir, metadata)

    print(f"[record_sim_session] Recording to {bag_out}")
    print(f"Topics: {', '.join(topics)}")
    if args.duration:
        print(f"Auto-stop after {args.duration} seconds")

    return_code = run_rosbag(cmd, args.duration)
    end_time = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    size_mb = compute_dir_size_mb(session_dir)

    metadata.update({"end_time": end_time, "size_mb": round(size_mb, 2), "return_code": return_code})
    write_metadata(session_dir, metadata)

    threshold = profile.get("max_size_mb")
    if threshold and size_mb > threshold:
        print(
            f"[record_sim_session] WARNING: recording is {size_mb:.1f} MB (> {threshold} MB target)",
            file=sys.stderr,
        )
    else:
        print(f"[record_sim_session] Recording complete ({size_mb:.1f} MB)")
    return return_code


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[record_sim_session] ERROR: {exc}", file=sys.stderr)
        sys.exit(1)

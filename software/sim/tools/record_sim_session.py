from __future__ import annotations

"""Helper to start MCAP-based rosbag recordings with storage safeguards."""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path


def load_profile(path: Path) -> dict:
    try:
        data = json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        raise ValueError(f"Profile at {path} is not valid JSON/YAML: {exc}") from exc
    if "rosbag" not in data or "topics" not in data["rosbag"]:
        raise ValueError("Profile missing rosbag/topics section")
    return data


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)
    if not os.access(path, os.W_OK):
        raise PermissionError(f"Output directory {path} is not writable")


def compute_dir_size_mb(path: Path) -> float:
    total = 0
    for file_path in path.rglob('*'):
        try:
            if file_path.is_file():
                total += file_path.stat().st_size
        except FileNotFoundError:
            continue
    return total / (1024 * 1024)


def build_rosbag_command(out_path: Path, profile: dict, topics: list[str]) -> list[str]:
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
    proc = subprocess.Popen(cmd)
    stop_event = threading.Event()

    def terminate(*_):
        stop_event.set()
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)

    timer = None
    if duration:
        def auto_stop():
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
    shutil.copy(profile_path, session_dir / "profile_snapshot.yaml")


def write_metadata(session_dir: Path, metadata: dict) -> None:
    metadata_path = session_dir / "metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True))


def parse_args() -> argparse.Namespace:
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

    timestamp = datetime.utcnow().strftime("%Y%m%dT%H%M%S")
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
    end_time = datetime.utcnow().strftime("%Y%m%dT%H%M%S")
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

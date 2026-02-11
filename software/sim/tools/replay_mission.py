"""Replay recorded Aeris missions and optionally launch the cinematic world.

This module provides functionality to replay rosbag recordings from
Aeris simulation missions, with optional world launching and camera
preset triggering for mission analysis and demonstration.

Typical usage example:
    # Replay a bag file
    python replay_mission.py --bag /path/to/bag

    # Launch world and replay with camera preset
    python replay_mission.py --bag /path/to/bag --launch-world --camera-preset wide
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

TOOLS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOLS_DIR.parent.parent
DEFAULT_WORLD = "software/sim/worlds/disaster_scene.sdf"
SEND_CAMERA = TOOLS_DIR / "send_camera_view.py"
RUN_SIM = TOOLS_DIR / "run_basic_sim.sh"


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for mission replay.

    Returns:
        Parsed arguments namespace.
    """
    parser = argparse.ArgumentParser(description="Replay Aeris mission recordings")
    parser.add_argument("--bag", required=True, help="Path to rosbag directory or metadata file")
    parser.add_argument(
        "--world",
        default=DEFAULT_WORLD,
        help=f"World SDF to launch headless before replay (default: {DEFAULT_WORLD})",
    )
    parser.add_argument("--launch-world", action="store_true", help="Automatically run run_basic_sim.sh")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    parser.add_argument("--loop", action="store_true", help="Loop playback (ros2 bag -l)")
    parser.add_argument("--pause", action="store_true", help="Start playback paused")
    parser.add_argument("--camera-preset", help="Preset to trigger at playback start")
    parser.add_argument("--camera-target", default="scout1", help="Vehicle name for presets")
    parser.add_argument("--camera-track", action="store_true", help="Enable tracking flag when sending preset")
    parser.add_argument("--dry-run", action="store_true", help="Print actions without executing")
    return parser.parse_args()


def resolve_bag_path(arg: str) -> Path:
    """Resolve the bag path from user input.

    Handles various input formats including directories, metadata files,
    and database files.

    Args:
        arg: User-provided path string.

    Returns:
        Resolved Path object pointing to the bag directory.

    Raises:
        FileNotFoundError: If the path does not exist.
    """
    bag_path = Path(arg).expanduser()
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag path not found: {bag_path}")
    if bag_path.is_dir():
        return bag_path
    if bag_path.suffix in {".mcap", ".db3"}:
        return bag_path.parent
    if bag_path.name == "metadata.yaml":
        return bag_path.parent
    return bag_path


def launch_world(world_path: str | None) -> subprocess.Popen[Any] | None:
    """Launch the simulation world using the run_basic_sim.sh script.

    Args:
        world_path: Path to the world SDF file, or None to skip.

    Returns:
        Subprocess handle if launched, None otherwise.

    Raises:
        FileNotFoundError: If the simulation script or world file is not found.
    """
    if not world_path:
        return None
    if not RUN_SIM.exists():
        raise FileNotFoundError(f"run_basic_sim.sh not found at {RUN_SIM}")
    world_file = Path(world_path)
    if not world_file.is_absolute():
        world_file = (REPO_ROOT / world_file).resolve()
    if not world_file.is_file():
        raise FileNotFoundError(f"World file not found: {world_file}")
    env = os.environ.copy()
    env.setdefault("HEADLESS", "1")
    env["WORLD_PATH"] = str(world_file)
    print(f"[replay_mission] Launching world via run_basic_sim.sh (WORLD_PATH={world_file})")
    return subprocess.Popen([str(RUN_SIM)], env=env)


def send_camera_preset(preset: str, target: str, track: bool) -> int:
    """Send a camera preset command to the simulation.

    Args:
        preset: Name of the camera preset to activate.
        target: Target vehicle name.
        track: Whether to enable tracking mode.

    Returns:
        Return code from the camera command (0 for success).
    """
    if not SEND_CAMERA.exists():
        print("[replay_mission] Camera sender not found; skipping preset trigger", file=sys.stderr)
        return 1
    cmd = ["python3", str(SEND_CAMERA), "--preset", preset, "--target", target]
    if track:
        cmd.append("--track")
    print(f"[replay_mission] Triggering camera preset: {' '.join(cmd)}")
    return subprocess.call(cmd)


def build_play_command(bag_path: Path, speed: float, loop: bool, pause: bool) -> list[str]:
    """Build the ros2 bag play command with options.

    Args:
        bag_path: Path to the bag directory.
        speed: Playback speed multiplier.
        loop: Whether to loop playback.
        pause: Whether to start paused.

    Returns:
        Command list for subprocess execution.
    """
    cmd = ["ros2", "bag", "play", str(bag_path)]
    if speed and speed != 1.0:
        cmd += ["-r", str(speed)]
    if loop:
        cmd.append("-l")
    if pause:
        cmd.append("--pause")
    return cmd


def main() -> int:
    """Main entry point for mission replay.

    Returns:
        Exit code (0 for success, non-zero for errors).
    """
    args = parse_args()
    try:
        bag_path = resolve_bag_path(args.bag)
    except FileNotFoundError as exc:
        print(f"[replay_mission] ERROR: {exc}", file=sys.stderr)
        return 1

    play_cmd = build_play_command(bag_path, args.speed, args.loop, args.pause)

    if args.dry_run:
        if args.launch_world:
            print(f"[replay_mission] Dry run: would launch world (WORLD_PATH={args.world})")
        print("[replay_mission] Dry run: ros2 bag command ->")
        print(" ".join(play_cmd))
        if args.camera_preset:
            print(
                f"[replay_mission] Dry run: would trigger preset {args.camera_preset} targeting {args.camera_target}"
            )
        return 0

    world_proc = None
    if args.launch_world:
        world_proc = launch_world(args.world)
        time.sleep(5)  # Allow Gazebo to initialize

    print(f"[replay_mission] Playing bag from {bag_path}")
    print("Command: " + " ".join(play_cmd))

    play_proc = subprocess.Popen(play_cmd)

    def terminate(*_: Any) -> None:
        if play_proc.poll() is None:
            play_proc.send_signal(signal.SIGINT)
        if world_proc and world_proc.poll() is None:
            world_proc.terminate()

    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)

    if args.camera_preset:
        # Wait briefly so the service exists
        time.sleep(2)
        send_camera_preset(args.camera_preset, args.camera_target, args.camera_track)

    return_code = play_proc.wait()
    if world_proc:
        world_proc.terminate()
        world_proc.wait(timeout=5)

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    print(f"[replay_mission] Playback finished with exit code {return_code}")
    return return_code


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[replay_mission] ERROR: {exc}", file=sys.stderr)
        sys.exit(1)

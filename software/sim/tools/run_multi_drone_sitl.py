#!/usr/bin/env python3
"""Helper for spawning multiple PX4 SITL instances with unique MAVLink ports."""
import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
from pathlib import Path


def load_config(path: Path):
    data = json.loads(path.read_text())
    if "vehicles" not in data or not isinstance(data["vehicles"], list):
        raise ValueError("Config must contain a 'vehicles' list")
    return data["vehicles"]


def validate(vehicles):
    names = set()
    ports = set()
    poses = []

    for vehicle in vehicles:
        name = vehicle.get("name")
        port = vehicle.get("mavlink_udp_port")
        pose = vehicle.get("pose")
        if not name or name in names:
            raise ValueError(f"Vehicle names must be unique; problem with {name!r}")
        if port in ports:
            raise ValueError(f"Duplicate MAVLink UDP port detected: {port}")
        if not isinstance(port, int):
            raise ValueError(f"Port must be int for {name}")
        if not pose:
            raise ValueError(f"Vehicle {name} missing pose string")
        pos_vals = [float(x) for x in pose.split()[0:3]]
        poses.append((name, pos_vals))
        names.add(name)
        ports.add(port)

    # Ensure ≥50 m separation between all vehicles
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            name_i, xyz_i = poses[i]
            name_j, xyz_j = poses[j]
            dist = math.dist(xyz_i, xyz_j)
            if dist < 50.0:
                raise ValueError(
                    f"Vehicles {name_i} and {name_j} are only {dist:.2f}m apart (<50m)"
                )


def summarize(vehicles, *, gps_denied: bool = False, external_vision: bool = False):
    lines = ["Multi-drone configuration:"]
    lines.append(
        f"- profile: gps_denied={'on' if gps_denied else 'off'}, "
        f"external_vision={'on' if external_vision else 'off'}"
    )
    for v in vehicles:
        lines.append(
            f"- {v['name']} model={v.get('model')} port={v['mavlink_udp_port']} pose={v['pose']}"
        )
    return "\n".join(lines)


def execute(vehicles, *, gps_denied: bool = False, external_vision: bool = False):
    px4_bin = os.environ.get("PX4_BIN") or shutil.which("px4")
    if not px4_bin:
        raise RuntimeError(
            "PX4 binary not found. Install PX4 SITL and/or set PX4_BIN to the px4 executable"
        )

    processes = []
    for idx, vehicle in enumerate(vehicles):
        env = os.environ.copy()
        env.setdefault("PX4_SIM_MODEL", vehicle.get("px4_sim_model", "gz_x500"))
        env.setdefault("PX4_SYS_AUTOSTART", str(vehicle.get("px4_sys_autostart", 4001)))
        env.setdefault("PX4_GZ_MODEL_POSE", vehicle["pose"])
        env.setdefault("PX4_INSTANCE", str(vehicle.get("px4_instance", idx + 1)))
        env.setdefault("PX4_MAVLINK_UDP_PRT", str(vehicle["mavlink_udp_port"]))

        if gps_denied:
            env.setdefault("AERIS_GPS_DENIED_MODE", "1")
            env.setdefault("PX4_SIM_GPS_DISABLED", "1")
            env.setdefault("PX4_PARAM_EKF2_GPS_CTRL", "0")
            env.setdefault("PX4_PARAM_EKF2_HGT_REF", "Vision")
        if external_vision:
            env.setdefault("AERIS_EXTERNAL_VISION_MODE", "1")
            env.setdefault("PX4_SIM_EXTERNAL_VISION", "1")
            env.setdefault("PX4_PARAM_EKF2_EV_CTRL", "3")

        raw_px4_env = vehicle.get("px4_env", {})
        if raw_px4_env:
            if not isinstance(raw_px4_env, dict):
                print(
                    f"Warning: {vehicle['name']} px4_env is not a dict; ignoring override",
                    file=sys.stderr,
                )
            else:
                for key, value in raw_px4_env.items():
                    key_name = str(key).strip()
                    if not key_name:
                        continue
                    env[key_name] = str(value)

        cmd = [px4_bin, "-i", env["PX4_INSTANCE"], "-d"]
        print(f"Launching {vehicle['name']}: {' '.join(cmd)}")
        processes.append((vehicle["name"], subprocess.Popen(cmd, env=env)))

    user_terminated = False

    def shutdown(*_):
        nonlocal user_terminated
        user_terminated = True
        for _, proc in processes:
            proc.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    failures = []
    for name, proc in processes:
        code = proc.wait()
        if user_terminated and code in (-signal.SIGINT, -signal.SIGTERM):
            continue
        if code != 0:
            failures.append((name, code))

    if failures:
        details = ", ".join(f"{name}={code}" for name, code in failures)
        raise RuntimeError(f"PX4 SITL process failed: {details}")


def main():
    parser = argparse.ArgumentParser(description="PX4 multi-drone SITL launcher")
    parser.add_argument(
        "--config",
        default="software/sim/config/multi_drone.yaml",
        help="Path to vehicle config (JSON/YAML syntax)",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually launch px4 processes (requires px4 binary)",
    )
    parser.add_argument(
        "--vehicles",
        help="Comma-separated vehicle names to include (default: all)",
    )
    parser.add_argument(
        "--gps-denied",
        action="store_true",
        help="Enable GPS-denied profile defaults (PX4_PARAM_EKF2_GPS_CTRL=0, vision altitude ref).",
    )
    parser.add_argument(
        "--external-vision",
        action="store_true",
        help="Enable external vision profile defaults (PX4_PARAM_EKF2_EV_CTRL=3).",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    vehicles = load_config(config_path)
    if args.vehicles:
        requested = {name.strip() for name in args.vehicles.split(',') if name.strip()}
        vehicles = [v for v in vehicles if v['name'] in requested]
        missing = requested - {v['name'] for v in vehicles}
        if missing:
            raise ValueError(f"Unknown vehicles requested: {', '.join(sorted(missing))}")
    if not vehicles:
        raise ValueError("No vehicles selected after filtering")
    validate(vehicles)
    print(
        summarize(
            vehicles,
            gps_denied=args.gps_denied,
            external_vision=args.external_vision,
        )
    )

    if args.execute:
        execute(
            vehicles,
            gps_denied=args.gps_denied,
            external_vision=args.external_vision,
        )
    else:
        print("Validation complete. Re-run with --execute to start SITL once PX4 is installed.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[run_multi_drone_sitl] ERROR: {exc}", file=sys.stderr)
        sys.exit(1)

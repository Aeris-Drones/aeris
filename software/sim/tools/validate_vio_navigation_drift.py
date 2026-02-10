#!/usr/bin/env python3
"""Validate VIO drift against simulation ground truth over a fixed mission window."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from bisect import bisect_left
from pathlib import Path

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
except ModuleNotFoundError as error:
    rclpy = None
    Odometry = object  # type: ignore[assignment]

    class Node:  # type: ignore[no-redef]
        pass

    _ROS_IMPORT_ERROR = error
else:
    _ROS_IMPORT_ERROR = None


def _load_profile(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("profile root must be a JSON object")
    return payload


def _stamp_to_seconds(message: Odometry) -> float:
    stamp = message.header.stamp
    seconds = float(stamp.sec) + (float(stamp.nanosec) / 1e9)
    if seconds <= 0.0:
        return time.time()
    return seconds


class DriftCollector(Node):
    def __init__(self, *, vio_topic: str, truth_topic: str) -> None:
        super().__init__("vio_navigation_drift_validator")
        self.vio_samples: list[tuple[float, float, float]] = []
        self.truth_samples: list[tuple[float, float, float]] = []
        self.create_subscription(Odometry, vio_topic, self._handle_vio, 10)
        self.create_subscription(Odometry, truth_topic, self._handle_truth, 10)
        self.get_logger().info(
            f"Collecting drift samples: vio_topic={vio_topic}, truth_topic={truth_topic}"
        )

    def _handle_vio(self, message: Odometry) -> None:
        stamp_sec = _stamp_to_seconds(message)
        self.vio_samples.append(
            (
                stamp_sec,
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
            )
        )

    def _handle_truth(self, message: Odometry) -> None:
        stamp_sec = _stamp_to_seconds(message)
        self.truth_samples.append(
            (
                stamp_sec,
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
            )
        )


def _nearest_truth_sample(
    truth_samples: list[tuple[float, float, float]], target_stamp: float
) -> tuple[float, float, float] | None:
    if not truth_samples:
        return None
    truth_times = [sample[0] for sample in truth_samples]
    index = bisect_left(truth_times, target_stamp)
    if index <= 0:
        return truth_samples[0]
    if index >= len(truth_samples):
        return truth_samples[-1]
    before = truth_samples[index - 1]
    after = truth_samples[index]
    if abs(before[0] - target_stamp) <= abs(after[0] - target_stamp):
        return before
    return after


def _compute_drift_metrics(
    vio_samples: list[tuple[float, float, float]],
    truth_samples: list[tuple[float, float, float]],
) -> tuple[dict[str, float], list[dict[str, float]]]:
    matched_rows: list[dict[str, float]] = []
    errors: list[float] = []
    sorted_truth = sorted(truth_samples, key=lambda sample: sample[0])
    for vio_stamp, vio_x, vio_z in sorted(vio_samples, key=lambda sample: sample[0]):
        truth_sample = _nearest_truth_sample(sorted_truth, vio_stamp)
        if truth_sample is None:
            continue
        truth_stamp, truth_x, truth_z = truth_sample
        error_m = math.hypot(vio_x - truth_x, vio_z - truth_z)
        matched_rows.append(
            {
                "stamp_sec": vio_stamp,
                "truth_stamp_sec": truth_stamp,
                "vio_x_m": vio_x,
                "vio_z_m": vio_z,
                "truth_x_m": truth_x,
                "truth_z_m": truth_z,
                "error_m": error_m,
            }
        )
        errors.append(error_m)

    if not errors:
        return (
            {
                "sample_count": 0,
                "rms_m": math.inf,
                "max_error_m": math.inf,
                "mean_error_m": math.inf,
            },
            matched_rows,
        )
    mean_error = sum(errors) / len(errors)
    rms_error = math.sqrt(sum(error * error for error in errors) / len(errors))
    return (
        {
            "sample_count": len(errors),
            "rms_m": rms_error,
            "max_error_m": max(errors),
            "mean_error_m": mean_error,
        },
        matched_rows,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare VIO odometry against simulation ground truth drift metrics."
    )
    parser.add_argument(
        "--profile",
        default="software/sim/config/vio_navigation_profile.yaml",
        help="JSON-compatible profile file with drift_validation defaults.",
    )
    parser.add_argument("--vio-topic", help="VIO odometry topic override")
    parser.add_argument("--ground-truth-topic", help="Ground-truth odometry topic override")
    parser.add_argument("--duration-sec", type=float, help="Collection duration in seconds")
    parser.add_argument("--max-rms-m", type=float, help="Maximum acceptable RMS drift (meters)")
    parser.add_argument("--max-error-m", type=float, help="Maximum acceptable max drift (meters)")
    parser.add_argument("--output-dir", help="Output directory root")
    parser.add_argument(
        "--run-id",
        help="Deterministic run folder name under output-dir (default: profile value or 'latest')",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if _ROS_IMPORT_ERROR is not None:
        raise RuntimeError(
            "ROS 2 Python dependencies are unavailable. Source ROS 2 (e.g., "
            "'source /opt/ros/humble/setup.bash') and workspace setup before running."
        ) from _ROS_IMPORT_ERROR

    profile = _load_profile(Path(args.profile))
    drift_config = profile.get("drift_validation", {})
    if not isinstance(drift_config, dict):
        raise ValueError("drift_validation profile entry must be an object")

    vio_topic = str(args.vio_topic or drift_config.get("vio_topic", "/scout1/openvins/odom"))
    truth_topic = str(
        args.ground_truth_topic
        or drift_config.get("ground_truth_topic", "/model/scout1/odometry")
    )
    duration_sec = float(args.duration_sec or drift_config.get("duration_sec", 600))
    max_rms_m = float(args.max_rms_m or drift_config.get("max_rms_m", 3.0))
    max_error_m = float(args.max_error_m or drift_config.get("max_error_m", 8.0))
    output_root = Path(args.output_dir or drift_config.get("output_dir", "output/vio_navigation_drift"))
    run_id = str(args.run_id or drift_config.get("run_id", "latest"))

    output_dir = output_root / run_id
    output_dir.mkdir(parents=True, exist_ok=True)
    report_json_path = output_dir / "drift_report.json"
    report_csv_path = output_dir / "drift_samples.csv"
    summary_path = output_dir / "drift_summary.txt"

    rclpy.init()
    collector = DriftCollector(vio_topic=vio_topic, truth_topic=truth_topic)
    start_monotonic = time.monotonic()

    try:
        while time.monotonic() - start_monotonic < duration_sec:
            rclpy.spin_once(collector, timeout_sec=0.1)
    finally:
        collector.destroy_node()
        rclpy.shutdown()

    metrics, matched_rows = _compute_drift_metrics(
        collector.vio_samples,
        collector.truth_samples,
    )
    sample_count = int(metrics["sample_count"])
    pass_rms = metrics["rms_m"] <= max_rms_m
    pass_max = metrics["max_error_m"] <= max_error_m
    passed = sample_count > 0 and pass_rms and pass_max

    report = {
        "vio_topic": vio_topic,
        "ground_truth_topic": truth_topic,
        "duration_sec": duration_sec,
        "thresholds": {"max_rms_m": max_rms_m, "max_error_m": max_error_m},
        "metrics": metrics,
        "passed": passed,
        "artifacts": {
            "report_json": str(report_json_path),
            "samples_csv": str(report_csv_path),
            "summary_txt": str(summary_path),
        },
        "generated_unix_sec": time.time(),
    }
    report_json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    with report_csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "stamp_sec",
                "truth_stamp_sec",
                "vio_x_m",
                "vio_z_m",
                "truth_x_m",
                "truth_z_m",
                "error_m",
            ],
        )
        writer.writeheader()
        writer.writerows(matched_rows)

    summary_lines = [
        f"VIO drift validation: {'PASS' if passed else 'FAIL'}",
        f"sample_count={sample_count}",
        f"rms_m={metrics['rms_m']:.4f} (limit {max_rms_m:.4f})",
        f"max_error_m={metrics['max_error_m']:.4f} (limit {max_error_m:.4f})",
        f"output_dir={output_dir}",
    ]
    summary_path.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
    print("\n".join(summary_lines))

    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())

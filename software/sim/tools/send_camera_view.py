"""Send preset or scripted camera commands to the simulation service.

This module provides a ROS 2 client for sending camera view commands
to the Aeris simulation, supporting both preset modes and custom
waypoint paths for cinematic camera control.

Typical usage example:
    # Activate a preset camera view
    python send_camera_view.py --preset wide --target scout1

    # Send a custom camera path
    python send_camera_view.py --path-json camera_path.json --target scout1
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import TYPE_CHECKING

import rclpy
from aeris_msgs.msg import CameraWaypoint
from aeris_msgs.srv import SetCameraView
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from rclpy.node import Node

if TYPE_CHECKING:
    from argparse import Namespace


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert roll-pitch-yaw angles to a quaternion.

    Uses the ZYX rotation convention (intrinsic rotations about Z, Y, X).

    Args:
        roll: Roll angle in radians.
        pitch: Pitch angle in radians.
        yaw: Yaw angle in radians.

    Returns:
        Tuple of (x, y, z, w) quaternion components.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def load_waypoints(json_path: Path) -> list[CameraWaypoint]:
    """Load camera waypoints from a JSON file.

    Args:
        json_path: Path to the waypoint JSON file.

    Returns:
        List of CameraWaypoint messages.

    Raises:
        ValueError: If no waypoints are found in the file.
    """
    data = json.loads(json_path.read_text())
    waypoints: list[CameraWaypoint] = []
    for entry in data.get('waypoints', []):
        waypoint = CameraWaypoint()
        time_value = float(entry.get('time_from_start', 0.0))
        secs = int(math.floor(time_value))
        nanos = int((time_value - secs) * 1e9)
        waypoint.time_from_start = Duration(sec=secs, nanosec=nanos)

        pose = Pose()
        position = entry.get('position', {})
        pose.position.x = float(position.get('x', 0.0))
        pose.position.y = float(position.get('y', 0.0))
        pose.position.z = float(position.get('z', 0.0))
        orientation = entry.get('orientation_rpy', {})
        quat = rpy_to_quat(
            float(orientation.get('roll', 0.0)),
            float(orientation.get('pitch', 0.0)),
            float(orientation.get('yaw', 0.0)),
        )
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        waypoint.pose = pose
        waypoint.field_of_view_deg = float(entry.get('fov_deg', 50.0))
        waypoint.focal_distance_m = float(entry.get('focal_distance_m', 40.0))
        waypoints.append(waypoint)

    if not waypoints:
        raise ValueError(f"No waypoints found in {json_path}")
    return waypoints


class CameraViewClient(Node):
    """ROS 2 client node for the camera view service."""

    def __init__(self) -> None:
        """Initialize the client and wait for the service."""
        super().__init__('camera_view_client')
        self._client = self.create_client(SetCameraView, '/simulation/set_camera_view')
        if not self._client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Service /simulation/set_camera_view not available')

    def send(self, args: Namespace) -> None:
        """Send a camera view request to the service.

        Args:
            args: Parsed command-line arguments containing target, preset,
                path_json, transition duration, and tracking mode.

        Raises:
            RuntimeError: If the service call fails or is rejected.
        """
        request = SetCameraView.Request()
        request.target_vehicle = args.target
        request.transition_duration = args.transition
        request.tracking_mode = args.track

        if args.preset:
            request.command_type = 'preset'
            request.preset_name = args.preset
        elif args.path_json:
            request.command_type = 'path'
            request.path = load_waypoints(Path(args.path_json))
        else:
            raise RuntimeError('Provide --preset or --path-json')

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is None:
            raise RuntimeError('Failed to call camera view service')
        response = future.result()
        if response.accepted:
            self.get_logger().info('Camera command accepted: %s', response.message)
        else:
            raise RuntimeError(f"Camera command rejected: {response.message}")


def main() -> None:
    """Parse arguments and send camera commands."""
    parser = argparse.ArgumentParser(description='Camera view service helper')
    parser.add_argument(
        '--preset',
        choices=['wide', 'tracking', 'pov', 'orbit'],
        help='Preset to activate'
    )
    parser.add_argument(
        '--path-json',
        help='JSON file generated by camera_path_builder.py'
    )
    parser.add_argument(
        '--target',
        default='scout1',
        help='Target vehicle for tracking/pov/orbit presets'
    )
    parser.add_argument(
        '--transition',
        type=float,
        default=3.0,
        help='Transition duration seconds'
    )
    parser.add_argument(
        '--track',
        action='store_true',
        help='Keep tracking target after first transition'
    )
    args = parser.parse_args()

    if not args.preset and not args.path_json:
        parser.error('Provide either --preset or --path-json')

    rclpy.init()
    try:
        client = CameraViewClient()
        client.send(args)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

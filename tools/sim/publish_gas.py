#!/usr/bin/env python3
"""Publish gas isopleth polygons over rosbridge for viewer testing.

This module simulates gas plume detection with realistic features including
concentric concentration zones, wind drift, and organic edge noise. It generates
nested polygons representing low/medium/high gas concentrations in local XY
meters, matching the runtime `GasIsopleth` contract used by orchestrator and
perception nodes.

ROS Topics:
    Published:
        /gas/isopleth (aeris_msgs/GasIsopleth): Gas concentration polygons

Usage:
    python publish_gas.py

The script connects to a rosbridge server at localhost:9090 and publishes
gas isopleth data at 1 Hz. The plume center drifts over time to simulate
wind, and polygon edges have randomized noise for organic appearance.

Features:
    - 3 concentric polygons (low/med/high concentration)
    - Wind drift simulation (plume moves over time)
    - Noise on polygon edges for organic appearance
    - Centerline data for wind vector visualization
"""

import math
import random
import sys
import time

try:
    import roslibpy
except ImportError:
    print("Error: roslibpy not installed. Please install it.")
    sys.exit(1)

# ROS bridge connection settings
HOST = 'localhost'
PORT = 9090
TOPIC = '/gas/isopleth'
MSG_TYPE = 'aeris_msgs/GasIsopleth'

# Leak origin in local XY meters.
LEAK_X_M = 50.0
LEAK_Y_M = 50.0


def make_ring(center_x, center_y, radius_m, segments=24, noise_factor=0.1):
    """Generate a polygon ring with optional noise for organic shape.

    Args:
        center_x: Center x coordinate in meters.
        center_y: Center y coordinate in meters.
        radius_m: Radius in meters.
        segments: Number of polygon vertices.
        noise_factor: Random variation as fraction of radius (0.1 = 10%).

    Returns:
        list: List of point dictionaries with 'x', 'y', 'z' keys representing
            polygon vertices in local XY meter coordinates.

    Note:
        The returned list includes a duplicate of the first point at the end
        to close the polygon loop for proper rendering.
    """
    points = []
    for i in range(segments):
        theta = (2 * math.pi * i) / segments

        # Add random noise for organic, non-perfect circular appearance
        r = radius_m + random.uniform(-1, 1) * noise_factor * radius_m

        x = center_x + (math.sin(theta) * r)
        y = center_y + (math.cos(theta) * r)
        points.append({'x': x, 'y': y, 'z': 0.0})

    # Close the polygon loop by duplicating the first point
    points.append(points[0].copy())
    return points


def make_centerline(center_x, center_y, wind_dir_rad, length_m=30):
    """Create centerline points showing wind direction.

    Args:
        center_x: Plume center x in meters.
        center_y: Plume center y in meters.
        wind_dir_rad: Wind direction in radians (0 = North, pi/2 = East).
        length_m: Length of centerline in meters.

    Returns:
        list: List of point dictionaries showing the plume centerline,
            with increasing altitude to visualize wind direction in 3D.
    """
    points = []
    for i in range(4):
        dist = (i / 3) * length_m
        x = center_x + (math.sin(wind_dir_rad) * dist)
        y = center_y + (math.cos(wind_dir_rad) * dist)
        # Rising centerline for 3D visualization
        points.append({'x': x, 'y': y, 'z': float(i * 3)})
    return points


def run():
    """Main publisher loop for gas isopleth data.

    Connects to rosbridge and publishes gas plume data at 1 Hz. The plume
    center drifts in a sinusoidal pattern to simulate wind effects, and
    the wind direction rotates slowly over time.
    """
    client = roslibpy.Ros(host=HOST, port=PORT)
    client.run()

    if not client.is_connected:
        print(f"Failed to connect to ROS Bridge at {HOST}:{PORT}")
        return

    print(f"Connected to ROS Bridge at {HOST}:{PORT}")
    print(f"Publishing gas plume near x={LEAK_X_M:.1f}m, y={LEAK_Y_M:.1f}m")
    print("Features: Wind drift, organic noise, centerline")

    # Advertise gas isopleth publisher
    pub = roslibpy.Topic(client, TOPIC, MSG_TYPE)
    pub.advertise()

    t = 0.0
    try:
        while client.is_connected:
            t += 0.1
            now = time.time()
            sec = int(now)
            nanosec = int((now - sec) * 1e9)

            # Simulate wind drift: plume center moves in Lissajous pattern
            drift_n = math.sin(t * 0.2) * 8.0   # +/- 8m North
            drift_e = math.cos(t * 0.15) * 5.0  # +/- 5m East

            current_x = LEAK_X_M + drift_e
            current_y = LEAK_Y_M + drift_n

            # Slowly rotating wind direction
            wind_dir = t * 0.1

            # Three concentric polygons: outer=low, mid=med, inner=high
            # Outer ring has more noise (turbulent edges), inner is smoother
            polygons = [
                {'points': make_ring(current_x, current_y, 50.0, segments=32, noise_factor=0.15)},
                {'points': make_ring(current_x, current_y, 30.0, segments=28, noise_factor=0.12)},
                {'points': make_ring(current_x, current_y, 12.0, segments=24, noise_factor=0.08)},
            ]

            # Construct GasIsopleth message
            msg = {
                'stamp': {'sec': sec, 'nanosec': nanosec},
                'species': 'VOC',
                'units': 'ppm',
                'polygons': polygons,
                'centerline': make_centerline(current_x, current_y, wind_dir),
            }

            pub.publish(roslibpy.Message(msg))
            print(f"Published gas plume (drift: N={drift_n:.1f}m, E={drift_e:.1f}m)")

            # 1 Hz update rate
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopping gas publisher...")
    finally:
        pub.unadvertise()
        client.terminate()


if __name__ == '__main__':
    run()

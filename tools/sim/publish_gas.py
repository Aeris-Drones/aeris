#!/usr/bin/env python3
"""Publish gas isopleth polygons over rosbridge for viewer testing.

This module simulates gas plume detection with realistic features including
concentric concentration zones, wind drift, and organic edge noise. It generates
nested polygons representing low/medium/high gas concentrations that move over
time to simulate wind effects.

ROS Topics:
    Published:
        /perception/gas (aeris_msgs/GasIsopleth): Gas concentration polygons

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
TOPIC = '/perception/gas'
MSG_TYPE = 'aeris_msgs/GasIsopleth'

# Base coordinate for simulation
BASE_LAT = 37.7749
BASE_LON = -122.4194

# Leak origin: 50m East, 50m North of base
OFFSET_E_M = 50.0
OFFSET_N_M = 50.0

# Approximate conversion factor for latitude degrees to meters
METERS_PER_DEG_LAT = 111_111.0


def offset_lat_lon(lat, lon, north_m, east_m):
    """Convert meter offsets to lat/lon offsets.

    Args:
        lat: Base latitude in degrees.
        lon: Base longitude in degrees.
        north_m: North offset in meters (positive = north).
        east_m: East offset in meters (positive = east).

    Returns:
        tuple: (new_lat, new_lon) after applying offsets.

    Note:
        Uses equirectangular approximation suitable for small distances
        (< 1km) at moderate latitudes.
    """
    d_lat = north_m / METERS_PER_DEG_LAT
    d_lon = east_m / (METERS_PER_DEG_LAT * math.cos(math.radians(lat)))
    return lat + d_lat, lon + d_lon


# Calculate absolute leak location
LEAK_LAT, LEAK_LON = offset_lat_lon(BASE_LAT, BASE_LON, OFFSET_N_M, OFFSET_E_M)


def make_ring(center_lat, center_lon, radius_m, segments=24, noise_factor=0.1):
    """Generate a polygon ring with optional noise for organic shape.

    Args:
        center_lat: Center latitude in degrees.
        center_lon: Center longitude in degrees.
        radius_m: Radius in meters.
        segments: Number of polygon vertices.
        noise_factor: Random variation as fraction of radius (0.1 = 10%).

    Returns:
        list: List of point dictionaries with 'x', 'y', 'z' keys representing
            the polygon vertices in lat/lon/altitude format.

    Note:
        The returned list includes a duplicate of the first point at the end
        to close the polygon loop for proper rendering.
    """
    points = []
    for i in range(segments):
        theta = (2 * math.pi * i) / segments

        # Add random noise for organic, non-perfect circular appearance
        r = radius_m + random.uniform(-1, 1) * noise_factor * radius_m

        north = math.cos(theta) * r
        east = math.sin(theta) * r
        lat, lon = offset_lat_lon(center_lat, center_lon, north, east)
        points.append({'x': lat, 'y': lon, 'z': 0.0})

    # Close the polygon loop by duplicating the first point
    points.append(points[0].copy())
    return points


def make_centerline(center_lat, center_lon, wind_dir_rad, length_m=30):
    """Create centerline points showing wind direction.

    Args:
        center_lat: Plume center latitude.
        center_lon: Plume center longitude.
        wind_dir_rad: Wind direction in radians (0 = North, pi/2 = East).
        length_m: Length of centerline in meters.

    Returns:
        list: List of point dictionaries showing the plume centerline,
            with increasing altitude to visualize wind direction in 3D.
    """
    points = []
    for i in range(4):
        dist = (i / 3) * length_m
        north = math.cos(wind_dir_rad) * dist
        east = math.sin(wind_dir_rad) * dist
        lat, lon = offset_lat_lon(center_lat, center_lon, north, east)
        # Rising centerline for 3D visualization
        points.append({'x': lat, 'y': lon, 'z': float(i * 3)})
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
    print(f"Publishing gas plume near lat={LEAK_LAT:.6f}, lon={LEAK_LON:.6f}")
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

            current_lat, current_lon = offset_lat_lon(
                LEAK_LAT, LEAK_LON, drift_n, drift_e
            )

            # Slowly rotating wind direction
            wind_dir = t * 0.1

            # Three concentric polygons: outer=low, mid=med, inner=high
            # Outer ring has more noise (turbulent edges), inner is smoother
            polygons = [
                {'points': make_ring(current_lat, current_lon, 50.0, segments=32, noise_factor=0.15)},
                {'points': make_ring(current_lat, current_lon, 30.0, segments=28, noise_factor=0.12)},
                {'points': make_ring(current_lat, current_lon, 12.0, segments=24, noise_factor=0.08)},
            ]

            # Construct GasIsopleth message
            msg = {
                'stamp': {'sec': sec, 'nanosec': nanosec},
                'species': 'VOC',
                'units': 'ppm',
                'polygons': polygons,
                'centerline': make_centerline(current_lat, current_lon, wind_dir),
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

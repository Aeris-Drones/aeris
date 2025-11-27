#!/usr/bin/env python3
"""Publish nested gas isopleth polygons over rosbridge for viewer testing.

Features:
- 3 concentric polygons (low/med/high concentration)
- Wind drift simulation (plume moves over time)
- Noise on polygon edges for organic appearance
- Centerline data for future wind vector visualization
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

HOST = 'localhost'
PORT = 9090
TOPIC = '/perception/gas'
MSG_TYPE = 'aeris_msgs/GasIsopleth'

BASE_LAT = 37.7749
BASE_LON = -122.4194

# Leak origin: 50m East, 50m North of base
OFFSET_E_M = 50.0
OFFSET_N_M = 50.0

# Simple equirectangular offsets
METERS_PER_DEG_LAT = 111_111.0


def offset_lat_lon(lat, lon, north_m, east_m):
    """Convert meter offsets to lat/lon offsets."""
    d_lat = north_m / METERS_PER_DEG_LAT
    d_lon = east_m / (METERS_PER_DEG_LAT * math.cos(math.radians(lat)))
    return lat + d_lat, lon + d_lon


LEAK_LAT, LEAK_LON = offset_lat_lon(BASE_LAT, BASE_LON, OFFSET_N_M, OFFSET_E_M)


def make_ring(center_lat, center_lon, radius_m, segments=24, noise_factor=0.1):
    """Generate a polygon ring with optional noise for organic shape.

    Args:
        center_lat, center_lon: Center of the ring
        radius_m: Radius in meters
        segments: Number of polygon vertices
        noise_factor: Amount of random variation (0.1 = 10% of radius)
    """
    points = []
    for i in range(segments):
        theta = (2 * math.pi * i) / segments

        # Add noise for organic appearance
        r = radius_m + random.uniform(-1, 1) * noise_factor * radius_m

        north = math.cos(theta) * r
        east = math.sin(theta) * r
        lat, lon = offset_lat_lon(center_lat, center_lon, north, east)
        points.append({'x': lat, 'y': lon, 'z': 0.0})

    # Close the polygon loop
    points.append(points[0].copy())
    return points


def make_centerline(center_lat, center_lon, wind_dir_rad, length_m=30):
    """Create centerline points showing wind direction.

    Args:
        center_lat, center_lon: Plume center
        wind_dir_rad: Wind direction in radians (0 = North)
        length_m: Length of centerline
    """
    points = []
    for i in range(4):
        dist = (i / 3) * length_m
        north = math.cos(wind_dir_rad) * dist
        east = math.sin(wind_dir_rad) * dist
        lat, lon = offset_lat_lon(center_lat, center_lon, north, east)
        points.append({'x': lat, 'y': lon, 'z': float(i * 3)})  # Rising centerline
    return points


def run():
    client = roslibpy.Ros(host=HOST, port=PORT)
    client.run()

    if not client.is_connected:
        print(f"Failed to connect to ROS Bridge at {HOST}:{PORT}")
        return

    print(f"Connected to ROS Bridge at {HOST}:{PORT}")
    print(f"Publishing gas plume near lat={LEAK_LAT:.6f}, lon={LEAK_LON:.6f}")
    print("Features: Wind drift, organic noise, centerline")

    pub = roslibpy.Topic(client, TOPIC, MSG_TYPE)
    pub.advertise()

    t = 0.0
    try:
        while client.is_connected:
            t += 0.1
            now = time.time()
            sec = int(now)
            nanosec = int((now - sec) * 1e9)

            # Simulate wind drift (plume center moves over time)
            drift_n = math.sin(t * 0.2) * 8.0   # +/- 8m North
            drift_e = math.cos(t * 0.15) * 5.0  # +/- 5m East

            current_lat, current_lon = offset_lat_lon(
                LEAK_LAT, LEAK_LON, drift_n, drift_e
            )

            # Simulate varying wind direction
            wind_dir = t * 0.1  # Slowly rotating wind

            # Three concentric polygons: outer=low, mid=med, inner=high
            # Different noise levels for each (outer = more turbulent)
            polygons = [
                {'points': make_ring(current_lat, current_lon, 50.0, segments=32, noise_factor=0.15)},
                {'points': make_ring(current_lat, current_lon, 30.0, segments=28, noise_factor=0.12)},
                {'points': make_ring(current_lat, current_lon, 12.0, segments=24, noise_factor=0.08)},
            ]

            msg = {
                'stamp': {'sec': sec, 'nanosec': nanosec},
                'species': 'VOC',
                'units': 'ppm',
                'polygons': polygons,
                'centerline': make_centerline(current_lat, current_lon, wind_dir),
            }

            pub.publish(roslibpy.Message(msg))
            print(f"Published gas plume (drift: N={drift_n:.1f}m, E={drift_e:.1f}m)")

            time.sleep(1.0)  # 1 Hz update rate

    except KeyboardInterrupt:
        print("\nStopping gas publisher...")
    finally:
        pub.unadvertise()
        client.terminate()


if __name__ == '__main__':
    run()

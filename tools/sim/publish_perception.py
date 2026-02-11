#!/usr/bin/env python3
"""Publish thermal hotspot detections over rosbridge for testing.

This module simulates thermal perception data by generating persistent hotspots
with slight movement and temperature variation over time. Useful for testing
the perception visualization pipeline without actual thermal cameras.

ROS Topics:
    Published:
        /perception/thermal (aeris_msgs/ThermalHotspot): Thermal detections

Usage:
    python publish_perception.py

The script connects to a rosbridge server at localhost:9090 and publishes
the thermal hotspot data at 2 Hz. Five persistent hotspots are generated
with randomized initial positions and temperatures.

Features:
    - 5 persistent hotspots with unique IDs
    - Slight orbital movement to simulate measurement variation
    - Temperature fluctuation over time
    - Randomized confidence values
"""

import time
import math
import random
import sys

try:
    import roslibpy
except ImportError:
    print("Error: roslibpy not installed. Please install it.")
    sys.exit(1)

# ROS bridge connection settings
HOST = 'localhost'
PORT = 9090
TOPIC = '/perception/thermal'
MSG_TYPE = 'aeris_msgs/ThermalHotspot'

# Base location for simulation (San Francisco area)
BASE_LAT = 37.7749
BASE_LON = -122.4194


def meters_to_lat(meters):
    """Convert meters to latitude degrees.

    Args:
        meters: Distance in meters.

    Returns:
        float: Equivalent latitude degrees.
    """
    return meters / 111111.0


def meters_to_lon(meters, lat):
    """Convert meters to longitude degrees.

    Args:
        meters: Distance in meters.
        lat: Latitude for cosine correction.

    Returns:
        float: Equivalent longitude degrees.
    """
    return meters / (111111.0 * math.cos(math.radians(lat)))


def run_publisher():
    """Main publisher loop for thermal hotspot detections.

    Connects to rosbridge, creates 5 persistent hotspots with randomized
    positions and temperatures, and publishes updates at 2 Hz. Each hotspot
    has slight orbital movement and temperature variation to simulate
    realistic sensor behavior.
    """
    client = roslibpy.Ros(host=HOST, port=PORT)
    client.run()

    if not client.is_connected:
        print(f"Failed to connect to ROS Bridge at {HOST}:{PORT}")
        return

    print(f"Connected to ROS Bridge at {HOST}:{PORT}")

    # Advertise thermal hotspot publisher
    publisher = roslibpy.Topic(client, TOPIC, MSG_TYPE)
    publisher.advertise()

    start_time = time.time()

    # Initialize 5 persistent hotspots with randomized parameters
    hotspots = []
    for i in range(5):
        hotspots.append({
            "id": f"hotspot_{i}",
            "offset_x": random.uniform(-100, 100),  # East offset in meters
            "offset_y": random.uniform(-100, 100),  # North offset in meters
            "base_temp": random.uniform(25, 70),    # Base temperature in Celsius
            "altitude": random.uniform(0, 50)       # Some elevated (e.g., buildings)
        })

    print(f"Publishing thermal hotspots to {TOPIC}...")

    try:
        while client.is_connected:
            now = time.time()
            elapsed = now - start_time

            # Construct ROS timestamp
            sec = int(now)
            nanosec = int((now - sec) * 1e9)

            for idx, h in enumerate(hotspots):
                # Simulate slight orbital movement for measurement variation
                x = h["offset_x"] + math.sin(elapsed * 0.5 + idx) * 5
                y = h["offset_y"] + math.cos(elapsed * 0.5 + idx) * 5

                # Vary temperature with slow sinusoidal fluctuation
                current_temp = h["base_temp"] + math.sin(elapsed) * 5

                # Convert local offsets to absolute coordinates
                lat = BASE_LAT + meters_to_lat(y)
                lon = BASE_LON + meters_to_lon(x, BASE_LAT)

                # Construct ThermalHotspot message
                msg = {
                    "stamp": {"sec": sec, "nanosec": nanosec},
                    "id": h["id"],
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": h["altitude"],
                    "temp_c": current_temp,
                    "confidence": random.uniform(0.7, 0.99),
                    "frame_id": "map",
                    "bbox_px": [0, 0, 0, 0]  # Dummy bounding box (not used in sim)
                }

                publisher.publish(roslibpy.Message(msg))

            # 2 Hz publish rate
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Stopping publisher...")
    finally:
        publisher.unadvertise()
        client.terminate()


if __name__ == '__main__':
    run_publisher()

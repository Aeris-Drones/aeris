#!/usr/bin/env python3
"""Publish perception detections over rosbridge for testing.

This module simulates thermal perception data by generating persistent hotspots
with slight movement and temperature variation over time. Useful for testing
the perception visualization pipeline without actual thermal cameras.

ROS Topics:
    Published (default):
        /perception/thermal (aeris_msgs/ThermalHotspot): Thermal detections
    Published (--fused):
        /detections/fused (aeris_msgs/FusedDetection): Fused detections

Usage:
    python publish_perception.py
    python publish_perception.py --fused

The script connects to a rosbridge server at localhost:9090 and publishes
the thermal hotspot data at 2 Hz. Five persistent hotspots are generated
with randomized initial positions and temperatures.

Features:
    - 5 persistent hotspots with unique IDs
    - Slight orbital movement to simulate measurement variation
    - Temperature fluctuation over time
    - Randomized confidence values
    - Optional fused detection stream for viewer end-to-end smoke tests
"""

import argparse
import json
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
THERMAL_TOPIC = '/perception/thermal'
THERMAL_MSG_TYPE = 'aeris_msgs/ThermalHotspot'
FUSED_TOPIC = '/detections/fused'
FUSED_MSG_TYPE = 'aeris_msgs/FusedDetection'

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


def confidence_to_level(confidence):
    """Convert numeric confidence to the string level used by FusedDetection."""
    if confidence >= 0.85:
        return 'HIGH'
    if confidence >= 0.6:
        return 'MEDIUM'
    return 'LOW'


def run_publisher(*, fused=False):
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

    topic = FUSED_TOPIC if fused else THERMAL_TOPIC
    msg_type = FUSED_MSG_TYPE if fused else THERMAL_MSG_TYPE

    # Advertise perception publisher
    publisher = roslibpy.Topic(client, topic, msg_type)
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

    mode = 'fused detections' if fused else 'thermal hotspots'
    print(f"Publishing {mode} to {topic}...")

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

                confidence = random.uniform(0.7, 0.99)

                if fused:
                    half_extent = 4.0
                    local_geometry = [
                        {"x": x - half_extent, "y": y - half_extent, "z": 0.0},
                        {"x": x + half_extent, "y": y - half_extent, "z": 0.0},
                        {"x": x + half_extent, "y": y + half_extent, "z": 0.0},
                        {"x": x - half_extent, "y": y + half_extent, "z": 0.0},
                    ]
                    modalities = ['thermal']
                    if idx % 3 == 1:
                        modalities = ['acoustic']
                    elif idx % 3 == 2:
                        modalities = ['gas']

                    hazard_payload = ""
                    if 'gas' in modalities:
                        hazard_payload = json.dumps({
                            "polygons": [[
                                {"x": point["x"], "z": point["y"]}
                                for point in local_geometry
                            ]]
                        })

                    # Construct FusedDetection message
                    msg = {
                        "stamp": {"sec": sec, "nanosec": nanosec},
                        "candidate_id": h["id"],
                        "mission_id": "sim-mission",
                        "confidence_level": confidence_to_level(confidence),
                        "confidence": confidence,
                        "source_modalities": modalities,
                        "local_target": {"x": x, "y": y, "z": h["altitude"]},
                        "local_geometry": local_geometry,
                        "frame_id": "map",
                        "hazard_payload_json": hazard_payload,
                    }
                else:
                    # Construct ThermalHotspot message
                    msg = {
                        "stamp": {"sec": sec, "nanosec": nanosec},
                        "id": h["id"],
                        "latitude": lat,
                        "longitude": lon,
                        "altitude": h["altitude"],
                        "temp_c": current_temp,
                        "confidence": confidence,
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
    parser = argparse.ArgumentParser(description='Publish simulated perception data over rosbridge.')
    parser.add_argument(
        '--fused',
        action='store_true',
        help='Publish aeris_msgs/FusedDetection data to /detections/fused for viewer smoke tests.',
    )
    args = parser.parse_args()
    run_publisher(fused=args.fused)

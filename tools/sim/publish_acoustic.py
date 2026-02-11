#!/usr/bin/env python3
"""Publish acoustic bearing detections over rosbridge for testing.

This module simulates acoustic detection by calculating bearings from vehicle
positions to a fixed sound source. It subscribes to vehicle telemetry to track
positions and publishes bearing measurements with SNR-based confidence.

ROS Topics:
    Subscribed:
        /vehicle/telemetry (aeris_msgs/Telemetry): Vehicle position updates
    Published:
        /perception/acoustic (aeris_msgs/AcousticBearing): Bearing detections

Usage:
    python publish_acoustic.py

The script connects to a rosbridge server at localhost:9090, tracks vehicle
positions from telemetry messages, and publishes bearing measurements to the
sound source at a 2 Hz rate.

Example Output:
    Published bearing 45.2 for scout_1 (dist 142.3m)
"""

import time
import math
import sys
import threading

try:
    import roslibpy
except ImportError:
    print("Error: roslibpy not installed. Please install it.")
    sys.exit(1)

# ROS bridge connection settings
HOST = 'localhost'
PORT = 9090
TELEM_TOPIC = '/vehicle/telemetry'
ACOUSTIC_TOPIC = '/perception/acoustic'
ACOUSTIC_MSG_TYPE = 'aeris_msgs/AcousticBearing'

# Sound source location: 100m North-East of base coordinate
BASE_LAT = 37.7749
BASE_LON = -122.4194
SOURCE_OFFSET_N = 100  # meters
SOURCE_OFFSET_E = 100  # meters

# Convert source offset to absolute coordinates
# 111111 meters per degree latitude (approximate)
SOURCE_LAT = BASE_LAT + (SOURCE_OFFSET_N / 111111.0)
# Longitude degrees vary by latitude (cosine correction)
SOURCE_LON = BASE_LON + (SOURCE_OFFSET_E / (111111.0 * math.cos(math.radians(BASE_LAT))))

print(f"Sound Source at: {SOURCE_LAT}, {SOURCE_LON}")

# Thread-safe storage for latest vehicle positions
latest_vehicle_positions = {}
lock = threading.Lock()


def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculate the forward azimuth (bearing) from point 1 to point 2.

    Args:
        lat1: Latitude of starting point in degrees.
        lon1: Longitude of starting point in degrees.
        lat2: Latitude of destination point in degrees.
        lon2: Longitude of destination point in degrees.

    Returns:
        float: Bearing in degrees [0, 360).

    Note:
        Uses the haversine formula for accurate bearing calculation
        over short distances.
    """
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    diff_lon_rad = math.radians(lon2 - lon1)

    y = math.sin(diff_lon_rad) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(diff_lon_rad)

    bearing_rad = math.atan2(y, x)
    bearing_deg = math.degrees(bearing_rad)

    return (bearing_deg + 360) % 360


def telemetry_callback(msg):
    """Process incoming telemetry messages and update vehicle positions.

    Args:
        msg: Dictionary containing the parsed Telemetry message.

    Note:
        Thread-safe update of the global latest_vehicle_positions dictionary.
        Silently ignores malformed messages missing required fields.
    """
    try:
        vid = msg['vehicle_id']
        pos = msg['position']
        with lock:
            latest_vehicle_positions[vid] = {
                'lat': pos['latitude'],
                'lon': pos['longitude'],
                'alt': pos['altitude']
            }
    except KeyError:
        # Silently skip malformed telemetry messages
        pass


def run_publisher():
    """Main publisher loop for acoustic bearing detections.

    Connects to rosbridge, subscribes to telemetry, and publishes bearing
    measurements at 2 Hz. Calculates bearings from each vehicle to the
    sound source and includes SNR-based confidence values.

    The detection range is limited to 200m. SNR decreases linearly with
    distance (20dB at 0m, 0dB at 200m). Classification varies by vehicle
    ID for testing purposes.
    """
    client = roslibpy.Ros(host=HOST, port=PORT)
    client.run()

    if not client.is_connected:
        print(f"Failed to connect to ROS Bridge at {HOST}:{PORT}")
        return

    print(f"Connected to ROS Bridge at {HOST}:{PORT}")

    # Subscribe to telemetry for position updates
    telem_sub = roslibpy.Topic(client, TELEM_TOPIC, 'aeris_msgs/Telemetry')
    telem_sub.subscribe(telemetry_callback)

    # Advertise acoustic bearing publisher
    acoustic_pub = roslibpy.Topic(client, ACOUSTIC_TOPIC, ACOUSTIC_MSG_TYPE)
    acoustic_pub.advertise()

    try:
        while client.is_connected:
            # Get thread-safe copy of current positions
            current_positions = {}
            with lock:
                current_positions = latest_vehicle_positions.copy()

            # Construct ROS timestamp (sec + nanosec)
            now = time.time()
            sec = int(now)
            nanosec = int((now - sec) * 1e9)

            for vid, pos in current_positions.items():
                # Calculate bearing to sound source
                bearing = calculate_bearing(pos['lat'], pos['lon'], SOURCE_LAT, SOURCE_LON)

                # Calculate distance using equirectangular approximation
                d_lat = (SOURCE_LAT - pos['lat']) * 111111.0
                d_lon = (SOURCE_LON - pos['lon']) * 111111.0 * math.cos(math.radians(BASE_LAT))
                dist = math.sqrt(d_lat * d_lat + d_lon * d_lon)

                # SNR: max 20dB at 0m, decreases with distance
                snr = max(0, 20 - (dist / 10.0))

                # Vary classification by vehicle for testing
                classification = "mechanical"
                if vid == "scout_1":
                    classification = "vocal"

                # Only publish if within detection range
                if dist < 200:
                    # Construct AcousticBearing message
                    msg = {
                        "stamp": {"sec": sec, "nanosec": nanosec},
                        "bearing_deg": bearing,
                        "confidence": 0.8 if snr > 10 else 0.4,
                        "snr_db": snr,
                        "mic_array": "array_v1",
                        "vehicle_id": vid,
                        "classification": classification
                    }
                    acoustic_pub.publish(roslibpy.Message(msg))
                    print(f"Published bearing {bearing:.1f} for {vid} (dist {dist:.1f}m)")

            # 2 Hz publish rate
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        telem_sub.unsubscribe()
        acoustic_pub.unadvertise()
        client.terminate()


if __name__ == '__main__':
    run_publisher()

#!/usr/bin/env python3
"""Publish simulated vehicle telemetry and mission state over rosbridge.

This module generates realistic vehicle telemetry data including position,
orientation, and velocity for multiple simulated vehicles. Vehicles follow
circular paths around a base coordinate with varying altitudes.

ROS Topics:
    Published:
        /vehicle/telemetry (aeris_msgs/Telemetry): Vehicle state updates
        /mission/state (std_msgs/String): Current mission state

Usage:
    python publish_telemetry.py

The script connects to a rosbridge server at localhost:9090 and publishes:
    - Vehicle telemetry at 10 Hz
    - Mission state changes every 10 seconds

Two vehicles are simulated:
    - scout_1: Smaller radius (50m), lower altitude (30m), faster speed
    - ranger_1: Larger radius (80m), higher altitude (60m), slower speed

Coordinate Frames:
    All positions use geodetic coordinates (WGS84).
    Velocities are in ENU frame (East-North-Up) per REP-103.
"""

import time
import math
import sys

try:
    import roslibpy
except ImportError:
    print("Error: roslibpy not installed. Please install it.")
    sys.exit(1)

# ROS bridge connection settings
HOST = 'localhost'
PORT = 9090
VEHICLE_TOPIC = '/vehicle/telemetry'
VEHICLE_MSG_TYPE = 'aeris_msgs/Telemetry'
MISSION_TOPIC = '/mission/state'
MISSION_MSG_TYPE = 'std_msgs/String'

# Vehicle definitions with orbital parameters
VEHICLES = [
    {
        "id": "scout_1",
        "type": "scout",
        "radius": 50.0,          # Orbit radius in meters
        "speed": 0.5,            # Angular velocity in rad/s
        "altitude_base": 30.0,   # Base altitude in meters
        "altitude_var": 5.0,     # Altitude variation amplitude
        "phase": 0.0             # Initial orbital phase
    },
    {
        "id": "ranger_1",
        "type": "ranger",
        "radius": 80.0,
        "speed": 0.3,
        "altitude_base": 60.0,
        "altitude_var": 2.0,
        "phase": 3.14            # Start opposite to scout_1
    }
]

# Mission state cycling configuration
MISSION_STATES = ["IDLE", "SEARCHING", "TRACKING", "COMPLETE"]
MISSION_STATE_CYCLE_TIME = 10.0  # seconds per state

# Base location (San Francisco area)
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
    """Main publisher loop for vehicle telemetry and mission state.

    Connects to rosbridge and publishes:
        - Vehicle telemetry at 10 Hz for all defined vehicles
        - Mission state transitions every 10 seconds

    Vehicle positions are calculated using circular orbital mechanics with
    ENU coordinate frame conventions. Velocities are computed as derivatives
    of the position equations.
    """
    client = roslibpy.Ros(host=HOST, port=PORT)

    try:
        client.run()
    except Exception as e:
        print(f"Failed to connect to ROS Bridge at {HOST}:{PORT}")
        print(e)
        return

    print(f"Connected to ROS Bridge at {HOST}:{PORT}")

    # Advertise publishers
    vehicle_publisher = roslibpy.Topic(client, VEHICLE_TOPIC, VEHICLE_MSG_TYPE)
    vehicle_publisher.advertise()

    mission_publisher = roslibpy.Topic(client, MISSION_TOPIC, MISSION_MSG_TYPE)
    mission_publisher.advertise()

    start_time = time.time()
    last_mission_update = 0.0
    current_state_idx = 0

    try:
        while client.is_connected:
            now = time.time()
            elapsed = now - start_time

            # --- Mission State Publishing ---
            if now - last_mission_update > MISSION_STATE_CYCLE_TIME:
                current_state = MISSION_STATES[current_state_idx]
                mission_msg = {"data": current_state}
                mission_publisher.publish(roslibpy.Message(mission_msg))
                print(f"Published Mission State: {current_state}")

                current_state_idx = (current_state_idx + 1) % len(MISSION_STATES)
                last_mission_update = now

            # --- Vehicle Telemetry Publishing ---
            for v in VEHICLES:
                # Calculate orbital position
                angle = v['phase'] + elapsed * v['speed']

                # ENU coordinate frame (REP 103): X=East, Y=North, Z=Up
                x = v['radius'] * math.cos(angle)  # East
                y = v['radius'] * math.sin(angle)  # North

                # Altitude with sinusoidal variation
                alt = v['altitude_base'] + v['altitude_var'] * math.sin(elapsed * 0.5)

                # Convert ENU offsets to geodetic coordinates
                lat = BASE_LAT + meters_to_lat(y)
                lon = BASE_LON + meters_to_lon(x, BASE_LAT)

                # Calculate yaw (tangent to circular path, +90 deg from angle)
                yaw = angle + math.pi / 2
                # Normalize yaw to [-pi, pi] range
                yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

                # Compute velocity components (derivatives of position)
                # vx = dx/dt = -radius * speed * sin(angle)
                # vy = dy/dt = radius * speed * cos(angle)
                # vz = dz/dt = altitude_var * 0.5 * cos(elapsed * 0.5)
                vx = -v['radius'] * v['speed'] * math.sin(angle)
                vy = v['radius'] * v['speed'] * math.cos(angle)
                vz = v['altitude_var'] * 0.5 * math.cos(elapsed * 0.5)

                # Construct ROS timestamp
                sec = int(now)
                nanosec = int((now - sec) * 1e9)

                # Construct Telemetry message
                msg = {
                    "vehicle_id": v['id'],
                    "vehicle_type": v['type'],
                    "timestamp": {
                        "sec": sec,
                        "nanosec": nanosec
                    },
                    "position": {
                        "latitude": lat,
                        "longitude": lon,
                        "altitude": alt
                    },
                    "orientation": {
                        "roll": 0.1 * math.sin(elapsed),   # Small roll oscillation
                        "pitch": 0.05 * math.cos(elapsed),  # Small pitch oscillation
                        "yaw": yaw
                    },
                    "velocity": {
                        "x": vx,  # East velocity (m/s)
                        "y": vy,  # North velocity (m/s)
                        "z": vz   # Up velocity (m/s)
                    }
                }

                vehicle_publisher.publish(roslibpy.Message(msg))

            # 10 Hz publish rate
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping publisher...")
    finally:
        vehicle_publisher.unadvertise()
        mission_publisher.unadvertise()
        client.terminate()


if __name__ == '__main__':
    run_publisher()

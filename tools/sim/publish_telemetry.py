#!/usr/bin/env python3
import time
import math
import json
import sys
import threading

# Try importing roslibpy, if not available, print error (it should be in the environment per memory)
try:
    import roslibpy
except ImportError:
    print("Error: roslibpy not installed. Please install it.")
    sys.exit(1)

# Configuration
HOST = 'localhost'
PORT = 9090
TOPIC = '/vehicle/telemetry'
MSG_TYPE = 'aeris_msgs/Telemetry'

# Vehicle definitions
VEHICLES = [
    {
        "id": "scout_1",
        "type": "scout",
        "radius": 50.0,
        "speed": 0.5, # rad/s
        "altitude_base": 30.0,
        "altitude_var": 5.0,
        "phase": 0.0
    },
    {
        "id": "ranger_1",
        "type": "ranger",
        "radius": 80.0,
        "speed": 0.3,
        "altitude_base": 60.0,
        "altitude_var": 2.0,
        "phase": 3.14 # Start opposite
    }
]

# Base location (San Francisco roughly, matching some map tiles if available, or just arbitrary)
# MapTileManager logic sets origin to first tile.
# If we send vehicle first, it sets origin to vehicle start.
# Let's use a fixed lat/lon.
BASE_LAT = 37.7749
BASE_LON = -122.4194

# 1 degree lat ~ 111km. 1 deg lon ~ 111km * cos(lat) ~ 88km.
# We want meters movement.
# lat_change = meters / 111111
# lon_change = meters / (111111 * cos(lat))

def meters_to_lat(meters):
    return meters / 111111.0

def meters_to_lon(meters, lat):
    return meters / (111111.0 * math.cos(math.radians(lat)))

def run_publisher():
    client = roslibpy.Ros(host=HOST, port=PORT)

    try:
        client.run()
    except Exception as e:
        print(f"Failed to connect to ROS Bridge at {HOST}:{PORT}")
        print(e)
        return

    print(f"Connected to ROS Bridge at {HOST}:{PORT}")

    publisher = roslibpy.Topic(client, TOPIC, MSG_TYPE)
    # Advertise is crucial
    publisher.advertise()

    start_time = time.time()

    try:
        while client.is_connected:
            now = time.time()
            elapsed = now - start_time

            for v in VEHICLES:
                # Circular path
                angle = v['phase'] + elapsed * v['speed']

                # Local offsets
                x = v['radius'] * math.cos(angle) # East
                z = v['radius'] * math.sin(angle) # North (math), but we usually map differently.
                # Let's just move them in a circle.

                # Convert to Geodetic
                lat = BASE_LAT + meters_to_lat(z)
                lon = BASE_LON + meters_to_lon(x, BASE_LAT)

                # Altitude sine wave
                alt = v['altitude_base'] + v['altitude_var'] * math.sin(elapsed * 0.5)

                # Orientation (Yaw matches movement tangent)
                # Tangent of circle: angle + 90 deg (pi/2)
                yaw = angle + math.pi / 2
                # Normalize yaw -pi to pi
                yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

                # Construct message
                # Timestamp
                sec = int(now)
                nanosec = int((now - sec) * 1e9)

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
                        "roll": 0.1 * math.sin(elapsed), # Add some roll
                        "pitch": 0.05 * math.cos(elapsed), # Add some pitch
                        "yaw": yaw
                    },
                    "velocity": {
                        "x": -v['radius'] * v['speed'] * math.sin(angle),
                        "y": 0.0, # Vertical velocity ignored for now
                        "z": v['radius'] * v['speed'] * math.cos(angle)
                    }
                }

                publisher.publish(roslibpy.Message(msg))

            # 10 Hz = 0.1s sleep
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping publisher...")
    finally:
        publisher.unadvertise()
        client.terminate()

if __name__ == '__main__':
    run_publisher()

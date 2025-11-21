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

                # ENU coordinate frame (REP 103): X=East, Y=North, Z=Up
                x = v['radius'] * math.cos(angle)  # East
                y = v['radius'] * math.sin(angle)  # North
                
                # Altitude sine wave
                alt = v['altitude_base'] + v['altitude_var'] * math.sin(elapsed * 0.5)

                # Convert to Geodetic (latitude = North, longitude = East)
                lat = BASE_LAT + meters_to_lat(y)
                lon = BASE_LON + meters_to_lon(x, BASE_LAT)

                # Orientation (Yaw matches movement tangent in ENU frame)
                # Tangent of circle: angle + 90 deg (pi/2)
                yaw = angle + math.pi / 2
                # Normalize yaw -pi to pi
                yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
                
                # Compute velocity derivatives (ENU frame)
                # d/dt of x = -radius * speed * sin(angle)
                # d/dt of y = radius * speed * cos(angle)
                # d/dt of z (altitude) = altitude_var * 0.5 * cos(elapsed * 0.5)
                vx = -v['radius'] * v['speed'] * math.sin(angle)
                vy = v['radius'] * v['speed'] * math.cos(angle)
                vz = v['altitude_var'] * 0.5 * math.cos(elapsed * 0.5)

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
                        "x": vx,  # East velocity
                        "y": vy,  # North velocity
                        "z": vz   # Up velocity
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

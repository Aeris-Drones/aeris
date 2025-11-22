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
VEHICLE_TOPIC = '/vehicle/telemetry'
VEHICLE_MSG_TYPE = 'aeris_msgs/Telemetry'
MISSION_TOPIC = '/mission/state'
MISSION_MSG_TYPE = 'std_msgs/String'

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

# Mission States
MISSION_STATES = ["IDLE", "SEARCHING", "TRACKING", "COMPLETE"]
MISSION_STATE_CYCLE_TIME = 10.0 # seconds per state

# Base location (San Francisco roughly, matching some map tiles if available, or just arbitrary)
BASE_LAT = 37.7749
BASE_LON = -122.4194

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

                vehicle_publisher.publish(roslibpy.Message(msg))

            # 10 Hz = 0.1s sleep
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping publisher...")
    finally:
        vehicle_publisher.unadvertise()
        mission_publisher.unadvertise()
        client.terminate()

if __name__ == '__main__':
    run_publisher()

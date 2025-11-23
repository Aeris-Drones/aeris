#!/usr/bin/env python3
import time
import math
import random
import sys

try:
    import roslibpy
except ImportError:
    print("Error: roslibpy not installed. Please install it.")
    sys.exit(1)

# Configuration
HOST = 'localhost'
PORT = 9090
TOPIC = '/perception/thermal'
MSG_TYPE = 'aeris_msgs/ThermalHotspot'

# Base location (using same base as telemetry for visibility)
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

    publisher = roslibpy.Topic(client, TOPIC, MSG_TYPE)
    publisher.advertise()

    start_time = time.time()

    # Create a few persistent hotspots
    hotspots = []
    for i in range(5):
        hotspots.append({
            "id": f"hotspot_{i}",
            "offset_x": random.uniform(-100, 100),
            "offset_y": random.uniform(-100, 100),
            "base_temp": random.uniform(25, 70),
            "altitude": random.uniform(0, 50) # Some might be elevated (e.g. on a building)
        })

    print(f"Publishing thermal hotspots to {TOPIC}...")

    try:
        while client.is_connected:
            now = time.time()
            elapsed = now - start_time

            sec = int(now)
            nanosec = int((now - sec) * 1e9)

            for h in hotspots:
                # Simulate slight movement or measurement noise
                x = h["offset_x"] + math.sin(elapsed * 0.5 + int(h["id"][-1])) * 5
                y = h["offset_y"] + math.cos(elapsed * 0.5 + int(h["id"][-1])) * 5

                # Vary temperature slightly
                current_temp = h["base_temp"] + math.sin(elapsed) * 5

                lat = BASE_LAT + meters_to_lat(y)
                lon = BASE_LON + meters_to_lon(x, BASE_LAT)

                msg = {
                    "stamp": { "sec": sec, "nanosec": nanosec },
                    "id": h["id"],
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": h["altitude"],
                    "temp_c": current_temp,
                    "confidence": random.uniform(0.7, 0.99),
                    "frame_id": "map",
                    "bbox_px": [0, 0, 0, 0] # Dummy
                }

                publisher.publish(roslibpy.Message(msg))

            # Publish at 2Hz as requested
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Stopping publisher...")
    finally:
        publisher.unadvertise()
        client.terminate()

if __name__ == '__main__':
    run_publisher()

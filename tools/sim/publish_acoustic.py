#!/usr/bin/env python3
import time
import math
import sys
import threading

try:
    import roslibpy
except ImportError:
    print("Error: roslibpy not installed. Please install it.")
    sys.exit(1)

# Configuration
HOST = 'localhost'
PORT = 9090
TELEM_TOPIC = '/vehicle/telemetry'
ACOUSTIC_TOPIC = '/perception/acoustic'
ACOUSTIC_MSG_TYPE = 'aeris_msgs/AcousticBearing'

# Sound Source Location (Lat/Lon)
# 100 meters North-East of Base
BASE_LAT = 37.7749
BASE_LON = -122.4194

# 1 degree lat approx 111111 meters
# 1 degree lon approx 111111 * cos(lat)
SOURCE_OFFSET_N = 100 # meters
SOURCE_OFFSET_E = 100 # meters

SOURCE_LAT = BASE_LAT + (SOURCE_OFFSET_N / 111111.0)
SOURCE_LON = BASE_LON + (SOURCE_OFFSET_E / (111111.0 * math.cos(math.radians(BASE_LAT))))

print(f"Sound Source at: {SOURCE_LAT}, {SOURCE_LON}")

# State
latest_vehicle_positions = {} # vehicle_id -> {lat, lon, alt}
lock = threading.Lock()

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculates the forward azimuth (bearing) from point 1 to point 2.
    Returns degrees [0, 360).
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
    # Parse telemetry to update vehicle positions
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
        pass

def run_publisher():
    client = roslibpy.Ros(host=HOST, port=PORT)
    client.run()

    if not client.is_connected:
        print(f"Failed to connect to ROS Bridge at {HOST}:{PORT}")
        return

    print(f"Connected to ROS Bridge at {HOST}:{PORT}")

    # Subscribe to telemetry
    telem_sub = roslibpy.Topic(client, TELEM_TOPIC, 'aeris_msgs/Telemetry')
    telem_sub.subscribe(telemetry_callback)

    # Publisher for acoustic
    acoustic_pub = roslibpy.Topic(client, ACOUSTIC_TOPIC, ACOUSTIC_MSG_TYPE)
    acoustic_pub.advertise()

    try:
        while client.is_connected:
            current_positions = {}
            with lock:
                current_positions = latest_vehicle_positions.copy()

            now = time.time()
            sec = int(now)
            nanosec = int((now - sec) * 1e9)

            for vid, pos in current_positions.items():
                # Calculate true bearing
                bearing = calculate_bearing(pos['lat'], pos['lon'], SOURCE_LAT, SOURCE_LON)

                # Distance for SNR simulation
                # Simple euclidean approx for short distances
                d_lat = (SOURCE_LAT - pos['lat']) * 111111.0
                d_lon = (SOURCE_LON - pos['lon']) * 111111.0 * math.cos(math.radians(BASE_LAT))
                dist = math.sqrt(d_lat*d_lat + d_lon*d_lon)

                # Simulate SNR: drops with distance
                # Max 20dB at 0m, drops 6dB per doubling distance?
                # Let's just do linear for simplicity in this range
                snr = max(0, 20 - (dist / 10.0))

                classification = "mechanical"
                # Randomly switch to vocal occasionally or based on vehicle?
                # Let's make scout_1 detect vocal
                if vid == "scout_1":
                    classification = "vocal"

                # Only publish if reasonably close (< 200m)
                if dist < 200:
                    msg = {
                        "stamp": { "sec": sec, "nanosec": nanosec },
                        "bearing_deg": bearing,
                        "confidence": 0.8 if snr > 10 else 0.4,
                        "snr_db": snr,
                        "mic_array": "array_v1",
                        "vehicle_id": vid,
                        "classification": classification
                    }
                    acoustic_pub.publish(roslibpy.Message(msg))
                    print(f"Published bearing {bearing:.1f} for {vid} (dist {dist:.1f}m)")

            time.sleep(0.5) # 2 Hz

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        telem_sub.unsubscribe()
        acoustic_pub.unadvertise()
        client.terminate()

if __name__ == '__main__':
    run_publisher()

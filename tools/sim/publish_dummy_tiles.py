import argparse
import math
import time
try:
    import roslibpy
except ImportError:
    print("Error: roslibpy is not installed. Please run 'pip install roslibpy pillow'")
    exit(1)

def get_tile_indices(lat, lon, zoom):
    n = 2.0 ** zoom
    xtile = int((lon + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n)
    return xtile, ytile

def main():
    parser = argparse.ArgumentParser(description='Publish dummy MapTile messages.')
    parser.add_argument('--host', default='localhost', help='ROS bridge host')
    parser.add_argument('--port', type=int, default=9090, help='ROS bridge port')
    parser.add_argument('--rate', type=float, default=5.0, help='Publish rate (Hz)')
    parser.add_argument('--zoom', type=int, default=18, help='Zoom level')
    parser.add_argument('--center-lat', type=float, default=37.7749, help='Center Latitude')
    parser.add_argument('--center-lon', type=float, default=-122.4194, help='Center Longitude')
    parser.add_argument('--radius', type=int, default=10, help='Radius in tiles to spiral out')

    args = parser.parse_args()

    client = roslibpy.Ros(host=args.host, port=args.port)
    client.run()

    publisher = roslibpy.Topic(client, '/map/tiles', 'aeris_msgs/MapTile')
    publisher.advertise()

    print(f"Connected to {args.host}:{args.port}")
    print(f"Publishing to /map/tiles at {args.rate} Hz")

    center_x, center_y = get_tile_indices(args.center_lat, args.center_lon, args.zoom)

    # Spiral generation
    x, y = 0, 0
    dx, dy = 0, -1

    # Pre-generate coordinates for spiral
    coords = []
    for i in range((args.radius * 2 + 1)**2):
        coords.append((center_x + x, center_y + y))
        if x == y or (x < 0 and x == -y) or (x > 0 and x == 1-y):
            dx, dy = -dy, dx
        x, y = x + dx, y + dy

    idx = 0
    try:
        while client.is_connected:
            if idx >= len(coords):
                print("Finished spiral, restarting...")
                idx = 0
                # optional: time.sleep(2)

            tx, ty = coords[idx]
            tile_id = f"{args.zoom}/{tx}/{ty}"

            print(f"Publishing tile: {tile_id}")

            msg = {
                'tile_id': tile_id,
                'format': 'mbtiles-1.3',
                'layer_ids': ['base'],
                'hash_sha256': 'dummy',
                'byte_size': 0,
            }

            publisher.publish(roslibpy.Message(msg))

            idx += 1
            time.sleep(1.0 / args.rate)

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        publisher.unadvertise()
        client.terminate()

if __name__ == '__main__':
    main()

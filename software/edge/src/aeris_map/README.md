# Aeris Map Tile Service

This package publishes map tile descriptors on `/map/tiles` and serves tile bytes through `/map/get_tile_bytes`.

## Source Selection

- `map_source=occupancy` uses `/map` (`nav_msgs/OccupancyGrid`)
- `map_source=point_cloud` uses `/rtabmap/cloud_map` (`sensor_msgs/PointCloud2`)
- `map_source=hybrid` enables both subscriptions

Use config/launch parameters to pick the source-of-truth per environment without changing core code.

## Tile Retrieval Contract

- Descriptor topic: `/map/tiles` (`aeris_msgs/MapTile`)
- Byte service: `/map/get_tile_bytes` (`aeris_msgs/GetMapTileBytes`)
- Message flow:
  1. Edge publishes `MapTile` with `tile_id` (`z/x/y`), `format=mbtiles-1.3`, hash, and byte size.
  2. Consumer requests bytes by `tile_id` via `GetMapTileBytes`.
  3. Edge returns `content_type`, bytes, publish timestamp, hash, and size.

## MBTiles 1.3 Storage

Generated tile payloads are persisted into MBTiles schema:

- `metadata(name, value)`
- `tiles(zoom_level, tile_column, tile_row, tile_data)`

`tile_row` is stored in TMS row convention per MBTiles 1.3.

## Smoke Validation (ROS 2 Humble)

1. Build packages:

```bash
colcon build --packages-select aeris_msgs aeris_map --symlink-install
```

2. Launch simulation stack plus RTAB-Map.
3. Run first tile latency check:

```bash
python3 software/edge/tools/first_tile_timer.py --timeout-sec 120 --start-mode liftoff
```

4. Run stream latency distribution probe:

```bash
python3 software/edge/tools/tile_latency_probe.py --samples 40 --timeout-sec 180
```

Success criteria:

- First tile appears within 90 seconds of mission liftoff.
- p95 descriptor-to-consumer latency is at or below 2000 ms.

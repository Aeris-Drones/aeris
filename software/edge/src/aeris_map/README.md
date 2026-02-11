# Aeris Map Tile Service

ROS 2 package for real-time map tile streaming from edge SLAM systems to ground station operators.

## Overview

The map tile service bridges RTAB-Map SLAM output and ground station visualization by:
- Converting occupancy grids and point clouds to standard MBTiles format
- Publishing tile descriptors via ROS 2 topics for low-latency notification
- Serving tile bytes via ROS 2 services for on-demand retrieval

## Architecture

```
┌─────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  RTAB-Map   │────▶│  Map Tile       │────▶│  Ground Station │
│  (SLAM)     │     │  Publisher      │     │  (UI/Operator)  │
└─────────────┘     └─────────────────┘     └─────────────────┘
       │                    │                         │
       ▼                    ▼                         ▼
  /map (Occupancy)    /map/tiles              Tile Request
  /rtabmap/cloud_map  (descriptor)            (service call)
```

## Source Selection

The map tile publisher supports multiple input sources configured via the `map_source` parameter:

| Source | Topic | Message Type | Use Case |
|--------|-------|--------------|----------|
| `occupancy` | `/map` | `nav_msgs/OccupancyGrid` | 2D navigation, path planning |
| `point_cloud` | `/rtabmap/cloud_map` | `sensor_msgs/PointCloud2` | 3D reconstruction, elevation |
| `hybrid` | Both | Both | Combined visualization |

Select the source via launch parameters without modifying core code.

## Tile Retrieval Contract

### Message Flow

1. **Descriptor Publication**: Edge publishes `MapTile` message containing:
   - `tile_id`: Slippy map coordinates (`{zoom}/{x}/{y}`)
   - `format`: MBTiles version identifier (`mbtiles-1.3`)
   - `hash_sha256`: Content hash for cache validation
   - `byte_size`: Payload size for bandwidth estimation
   - `layer_ids`: Composition hints for multi-layer rendering

2. **Byte Request**: Consumer calls `GetMapTileBytes` service with `tile_id`

3. **Byte Response**: Edge returns:
   - `content_type`: MIME type (`image/png`)
   - `data`: Raw tile bytes
   - `published_at`: Original publication timestamp
   - `hash_sha256`: Verified content hash
   - `byte_size`: Confirmed payload size

### ROS 2 Interfaces

- **Topic**: `/map/tiles` (`aeris_msgs/MapTile`)
- **Service**: `/map/get_tile_bytes` (`aeris_msgs/GetMapTileBytes`)

## MBTiles 1.3 Storage

Tile payloads are persisted in SQLite following the MBTiles 1.3 specification:

```sql
CREATE TABLE metadata (name TEXT PRIMARY KEY, value TEXT);
CREATE TABLE tiles (
    zoom_level INTEGER,
    tile_column INTEGER,
    tile_row INTEGER,
    tile_data BLOB,
    PRIMARY KEY (zoom_level, tile_column, tile_row)
);
```

**Note**: `tile_row` uses TMS convention (inverted Y) per MBTiles 1.3 spec.

## Configuration

Key parameters in `config/map_tile_stream.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_source` | `occupancy` | Input source selection |
| `zoom` | 18 | Tile zoom level (0-22) |
| `tile_size_px` | 256 | Tile dimensions in pixels |
| `publish_on_change_only` | true | Reduce bandwidth by skipping unchanged tiles |
| `republish_interval_sec` | 5.0 | Force republish interval for consistency |
| `max_cached_tiles` | 500 | LRU cache size for tile storage |
| `mbtiles_path` | `/tmp/aeris/map_tiles/live_map.mbtiles` | SQLite database location |

## Quick Start

### Build

```bash
colcon build --packages-select aeris_msgs aeris_map --symlink-install
```

### Launch

```bash
ros2 launch aeris_map rtabmap_vio_sim.launch.py
```

### Validation

First tile latency (mission start to first tile publication):

```bash
python3 software/edge/tools/first_tile_timer.py --timeout-sec 120 --start-mode liftoff
```

Stream latency distribution (descriptor to byte retrieval):

```bash
python3 software/edge/tools/tile_latency_probe.py --samples 40 --timeout-sec 180
```

### Success Criteria

- First tile published within 90 seconds of mission liftoff
- p95 descriptor-to-consumer latency at or below 2000 ms

## Package Structure

```
aeris_map/
├── config/              # YAML parameter configurations
│   ├── map_tile_stream.yaml
│   ├── openvins_sim.yaml
│   └── rtabmap_vio_sim.yaml
├── include/aeris_map/   # C++ headers
│   └── tile_contract.hpp
├── launch/              # ROS 2 launch files
│   └── rtabmap_vio_sim.launch.py
├── src/                 # C++ implementations
│   ├── map_tile_publisher.cpp
│   └── tile_contract.cpp
└── test/                # Unit tests
    └── test_tile_contract.cpp
```

## Dependencies

- ROS 2 Humble
- RTAB-Map SLAM
- OpenVINS (for VIO pipeline)
- libpng (PNG encoding)
- SQLite3 (MBTiles storage)
- OpenSSL (SHA-256 hashing)

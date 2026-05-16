# Rosbridge (Dev)

This folder builds a small self-contained rosbridge image that also includes the project's custom ROS 2 message types (`aeris_msgs`) so the viewer can subscribe/publish via WebSocket without runtime `apt-get`/`colcon` steps.

Rebuild this image whenever `software/edge/src/aeris_msgs/msg/*.msg` changes, including `MapTile.msg` provenance fields used by the viewer.

## Run

From repo root:

```bash
docker compose -f docker-compose.rosbridge.yml up -d --build
docker compose -f docker-compose.rosbridge.yml logs -f
```

Verify the port is exposed:

```bash
docker port aeris-rosbridge
```

## Stop

```bash
docker compose -f docker-compose.rosbridge.yml down
```

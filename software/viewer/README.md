# Aeris GCS Viewer

A Next.js-based Ground Control Station (GCS) viewer for the Aeris autonomous drone swarm system.
This viewer provides real-time 3D visualization and telemetry monitoring using ROS 2 via WebSockets.

## Prerequisites

- Node.js 18+
- ROS 2 (Humble/Jazzy)
- `rosbridge_suite` package

## Installation

1. Install dependencies:
   ```bash
   npm install
   ```

## Running the Viewer

1. Start the development server:
   ```bash
    npm run dev
   ```
   The viewer will be available at `http://localhost:3000`.

2. If you need webpack mode (recommended for reproducible shader imports / offline-friendly dev):
   ```bash
   npm run dev:webpack
   ```

## Performance Validation (Gas Particles)

To capture a repeatable FPS + particle instance count log for Story 3.4:

```bash
# In one terminal (from software/viewer/)
NEXT_PUBLIC_GAS_PERF=1 npm run dev:webpack

# In another terminal (from repo root)
python3 tools/sim/publish_gas.py
```

This prints 1 line/second in the browser console: `fps=… instances=…`. Optional adaptive scaling can be enabled with `NEXT_PUBLIC_GAS_PARTICLE_ADAPTIVE=1`.

## ROS 2 Connection

To feed data to the viewer, you must run the `rosbridge_websocket` node on your ROS 2 system.

### Option 1: Docker (Recommended for macOS/Windows)

The easiest way to run rosbridge_server is using Docker:

```bash
# Using docker-compose (from project root)
docker-compose -f docker-compose.rosbridge.yml up

# Or using the helper script
./scripts/run_rosbridge.sh
```

The WebSocket server will be available at `ws://localhost:9090`.

### Option 2: Native ROS 2 Installation

If you have ROS 2 Humble installed natively:

```bash
# Install rosbridge_suite if not already installed
sudo apt install ros-humble-rosbridge-suite  # or ros-jazzy-rosbridge-suite

# Launch the websocket server (defaults to ws://localhost:9090)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Features

- **3D Visualization**: Interactive Three.js scene with OrbitControls.
- **Telemetry**: Real-time display of Altitude, Speed, Heading, and more.
- **Status Monitoring**: Aviation-standard indicators for GPS, Battery, Link, and Flight Mode.
- **Responsive Layout**: Optimized for Tablet and Desktop use.

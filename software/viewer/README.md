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

## ROS 2 Connection

To feed data to the viewer, you must run the `rosbridge_websocket` node on your ROS 2 system:

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

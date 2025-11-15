# Aeris 3D Viewer

A real-time 3D visualization interface for the Aeris multi-drone disaster response system. Built with Next.js, Three.js, and ROS 2 integration via rosbridge.

## Features

- **Real-time 3D Rendering**: High-performance Three.js scene with 60 FPS target
- **ROS 2 Integration**: WebSocket connection to ROS 2 via rosbridge_suite
- **Live Data Visualization**: Subscribe to ROS 2 topics and display data in real-time
- **Interactive Controls**: Orbit, pan, and zoom camera controls
- **Connection Management**: Automatic reconnection with exponential backoff
- **Modern UI**: Built with Next.js 16, React 19, and Tailwind CSS

## Tech Stack

- **Framework**: Next.js 16 with App Router
- **Language**: TypeScript
- **3D Engine**: Three.js with @react-three/fiber
- **ROS Bridge**: roslib
- **Styling**: Tailwind CSS 4
- **UI Components**: shadcn/ui (initialized)

## Prerequisites

### ROS 2 Requirements

You need a running ROS 2 environment with rosbridge_suite installed:

```bash
# Install rosbridge_suite (if not already installed)
sudo apt install ros-humble-rosbridge-suite

# Or build from source in your workspace
cd ~/ros2_ws/src
git clone https://github.com/RobotWebTools/rosbridge_suite.git
cd ~/ros2_ws
colcon build --packages-select rosbridge_suite
```

### Node.js Requirements

- Node.js 20.x or later
- npm 9.x or later

## Setup

### 1. Install Dependencies

```bash
cd software/viewer
npm install
```

### 2. Launch ROS Bridge

In a separate terminal, launch the rosbridge WebSocket server:

```bash
# Source your ROS 2 environment
source /opt/ros/humble/setup.bash
# Or if built from source:
# source ~/ros2_ws/install/setup.bash

# Launch rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

The rosbridge server will start on `ws://localhost:9090` by default.

### 3. Run the Viewer

```bash
# Development mode (with hot reload)
npm run dev

# Production build
npm run build
npm start
```

Open [http://localhost:3000](http://localhost:3000) in your browser.

## Development Workflow

### Development Server

```bash
npm run dev
```

The development server runs on port 3000 with hot module replacement.

### Building for Production

```bash
npm run build
```

### Linting

```bash
npm run lint
```

## Usage

### Testing the ROS Connection

1. Ensure rosbridge is running
2. Launch the viewer (`npm run dev`)
3. Check the connection status indicator in the top-left corner:
   - **Green**: Connected successfully
   - **Yellow**: Connecting...
   - **Red**: Connection error (click "Retry" to reconnect)
   - **Gray**: Disconnected

### Subscribing to Topics

The viewer automatically subscribes to the `/clock` topic as a test. You can see:

- Message count in the topic subscriber panel
- Live message data in the browser console
- Latest message payload in the UI

To subscribe to different topics, modify `src/app/page.tsx`.

### 3D Scene Controls

- **Orbit**: Left mouse button + drag
- **Pan**: Right mouse button + drag
- **Zoom**: Mouse scroll wheel

### Performance Monitoring

The FPS counter (Stats panel) is visible in the top-left corner of the 3D scene. Target is 60 FPS minimum.

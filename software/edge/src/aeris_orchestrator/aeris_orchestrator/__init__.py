"""Aeris Orchestrator ROS 2 package.

Coordinates multi-vehicle search and tracking missions by integrating
sensor detections (thermal, acoustic, gas) with MAVLink-compatible
autopilot command streams. Supports both GPS-based and GPS-denied
(VIO-based) navigation modes for scout and ranger vehicle roles.

Modules:
    heartbeat_node: Minimal liveness signal publisher for system health monitoring.
    mavlink_adapter: MAVLink protocol abstraction for waypoint streaming and commands.
    mission_node: Core mission lifecycle orchestrator with state machine management.
    search_patterns: Geometric waypoint generation for lawnmower and spiral patterns.
    vehicle_ids: Normalization utilities for consistent vehicle identifier handling.
"""

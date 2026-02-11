"""AERIS perception package providing synthetic sensor data publishers.

This package contains ROS 2 nodes that generate synthetic sensor data for
testing and validation of perception pipelines without requiring physical
hardware. Supported sensor types include:

- Thermal: ThermalHotspotNode generates synthetic thermal camera detections
- Acoustic: AcousticBearingNode generates synthetic microphone array bearings
- Gas: GasIsoplethNode generates synthetic gas plume contours

Example:
    To run the thermal hotspot node::

        $ ros2 run aeris_perception thermal_hotspot

    To run with custom parameters::

        $ ros2 run aeris_perception thermal_hotspot --ros-args -p rate_hz:=10.0
"""

__version__ = "0.1.0"

"""AERIS perception package providing simulation-ready perception nodes.

This package contains ROS 2 nodes for thermal, acoustic, and gas perception.
Thermal detections are derived from image processing, while acoustic and gas
nodes provide simulation-friendly data feeds.

Example:
    To run the thermal hotspot node::

        $ ros2 run aeris_perception thermal_hotspot

    To run with custom parameters::

        $ ros2 run aeris_perception thermal_hotspot --ros-args -p target_publish_rate_hz:=10.0
"""

__version__ = "0.1.0"

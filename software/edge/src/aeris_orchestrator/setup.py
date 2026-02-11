"""Aeris Orchestrator ROS 2 package setup configuration.

Defines package metadata, entry points, and installation requirements
for the Aeris mission orchestration system. The orchestrator coordinates
multi-vehicle search and tracking missions through ROS 2 services and
topics, integrating sensor detections with MAVLink autopilot commands.
"""

from setuptools import setup


package_name = "aeris_orchestrator"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "pymavlink"],
    zip_safe=True,
    maintainer="Aeris Developers",
    maintainer_email="dev@aeris.local",
    description="Aeris orchestrator package for multi-vehicle mission coordination.",
    license="proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "heartbeat = aeris_orchestrator.heartbeat_node:main",
            "mission = aeris_orchestrator.mission_node:main",
        ],
    },
)

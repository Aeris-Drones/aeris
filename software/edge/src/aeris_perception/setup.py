"""Setup configuration for the aeris_perception package.

This package provides synthetic perception publishers for thermal, acoustic,
and gas sensor simulation in ROS 2 environments.
"""

from setuptools import setup

package_name = "aeris_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Aeris Developers",
    maintainer_email="dev@aeris.local",
    description="Synthetic perception publishers for thermal, acoustic, and gas cues.",
    license="proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "thermal_hotspot = aeris_perception.thermal_hotspot_node:main",
            "acoustic_bearing = aeris_perception.acoustic_bearing_node:main",
            "gas_isopleth = aeris_perception.gas_isopleth_node:main",
        ],
    },
)

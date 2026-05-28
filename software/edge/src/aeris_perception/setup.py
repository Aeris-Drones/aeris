"""Setup configuration for the aeris_perception package."""

from setuptools import setup

package_name = "aeris_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/config",
            [
                "config/halo_rgb_ingest.example.yaml",
                "config/perception_demo_rgb_ingest.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Aeris Developers",
    maintainer_email="dev@aeris.local",
    description="Perception nodes for Halo RGB ingest and hazard cues.",
    license="proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rgb_ingest = aeris_perception.rgb_ingest_node:main",
            "thermal_hotspot = aeris_perception.thermal_hotspot_node:main",
            "acoustic_bearing = aeris_perception.acoustic_bearing_node:main",
            "acoustic_audio_sim = aeris_perception.acoustic_audio_sim_node:main",
            "gas_isopleth = aeris_perception.gas_isopleth_node:main",
            "gas_input_sim = aeris_perception.gas_input_sim_node:main",
            "fusion = aeris_perception.fusion_node:main",
        ],
    },
)

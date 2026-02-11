"""Setup configuration for aeris_mesh_agent package.

Provides network impairment simulation tools for testing Aeris systems
under degraded communication conditions.
"""
from setuptools import setup

package_name = "aeris_mesh_agent"

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
    description="Mesh impairment helpers for Aeris simulation.",
    license="proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "impairment_relay = aeris_mesh_agent.impairment_relay:main",
            "store_forward_tiles = aeris_mesh_agent.store_forward_tiles:main",
        ],
    },
)

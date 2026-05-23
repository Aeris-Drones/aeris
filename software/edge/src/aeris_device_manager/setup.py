"""Setup configuration for the aeris_device_manager package."""

from setuptools import setup


package_name = "aeris_device_manager"


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
    description="Device manager package for Aeris modular pod enumeration.",
    license="proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "device_manager = aeris_device_manager.device_manager_node:main",
        ],
    },
)

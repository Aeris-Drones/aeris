"""Setup configuration for the aeris_update_manager package."""

from setuptools import setup


package_name = "aeris_update_manager"


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
    description="Firmware update coordination package for Aeris edge nodes.",
    license="proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "firmware_update_manager = aeris_update_manager.node:main",
        ],
    },
)

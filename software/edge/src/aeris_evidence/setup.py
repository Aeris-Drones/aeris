from setuptools import setup

package_name = "aeris_evidence"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Aeris Developers",
    maintainer_email="dev@aeris.local",
    description="Evidence bundling CLI for bench runs",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "evidence_bundle = aeris_evidence.evidence_bundle:main",
        ],
    },
)

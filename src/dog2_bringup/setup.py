from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


package_name = "dog2_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/worlds", glob("worlds/*")),
        (f"share/{package_name}/doc", glob("doc/*.md")),
        (f"share/{package_name}", ["README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Developer",
    maintainer_email="dev@example.com",
    description="Unified bringup and integration package for Dog2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cmd_vel_teleop = dog2_bringup.cmd_vel_teleop:main",
            "crossing_check = dog2_bringup.crossing_check:main",
            "crossing_trigger = dog2_bringup.crossing_trigger:main",
            "mpc_debug_adapter = dog2_bringup.mpc_debug_adapter:main",
            "smoke_check = dog2_bringup.smoke_check:main",
            "wbc_effort_mux = dog2_bringup.wbc_effort_mux:main",
            "wbc_debug_adapter = dog2_bringup.wbc_debug_adapter:main",
        ],
    },
)

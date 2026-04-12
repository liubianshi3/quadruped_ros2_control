from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


PACKAGE_NAME = "dog2_motion_control"
PACKAGE_VERSION = "0.1.0"

MAINTAINER_NAME = "Developer"
MAINTAINER_EMAIL = "dev@example.com"
PACKAGE_DESCRIPTION = (
    "dog2 quadruped motion-control stack for narrow frame crossing tasks"
)
PACKAGE_LICENSE = "Apache-2.0"

LAUNCH_GLOB = "launch/*.py"
CONFIG_GLOB = "config/*.yaml"

AMENT_INDEX_PATH = "share/ament_index/resource_index/packages"
PACKAGE_SHARE_PATH = str(Path("share") / PACKAGE_NAME)
PACKAGE_LAUNCH_SHARE_PATH = str(Path("share") / PACKAGE_NAME / "launch")
PACKAGE_CONFIG_SHARE_PATH = str(Path("share") / PACKAGE_NAME / "config")

CONSOLE_SCRIPTS = [
    "spider_controller = dog2_motion_control.spider_robot_controller:main",
    "mpc_controller = dog2_motion_control.mpc_robot_controller:main",
    "gz_pose_to_odom = dog2_motion_control.gz_pose_to_odom:main",
    "obstacle_analysis = dog2_motion_control.obstacle_analysis:main",
    "gz_gain_setter = dog2_motion_control.gz_gain_setter:main",
    "gz_startup_gate = dog2_motion_control.gz_startup_gate:main",
]

INSTALL_REQUIRES = [
    "setuptools",
    "numpy>=1.20.0",
    "scipy>=1.7.0",
]

setup(
    name=PACKAGE_NAME,
    version=PACKAGE_VERSION,
    packages=find_packages(exclude=["test"]),
    data_files=[
        (AMENT_INDEX_PATH, [f"resource/{PACKAGE_NAME}"]),
        (PACKAGE_SHARE_PATH, ["package.xml"]),
        (PACKAGE_LAUNCH_SHARE_PATH, glob(LAUNCH_GLOB)),
        (PACKAGE_CONFIG_SHARE_PATH, glob(CONFIG_GLOB)),
    ],
    install_requires=INSTALL_REQUIRES,
    zip_safe=True,
    maintainer=MAINTAINER_NAME,
    maintainer_email=MAINTAINER_EMAIL,
    description=PACKAGE_DESCRIPTION,
    license=PACKAGE_LICENSE,
    tests_require=["pytest"],
    entry_points={"console_scripts": CONSOLE_SCRIPTS},
)

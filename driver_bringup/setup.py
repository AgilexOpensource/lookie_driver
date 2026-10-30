import os
from glob import glob

from setuptools import setup

package_name = "driver_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Agilex Robotics",
    maintainer_email="contact@agilex.ai",
    description="Lookie driver bringup: motor YAML, launch files, and joint name bridge",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "joint_bridge_node = driver_bringup.joint_bridge_node:main",
        ],
    },
)

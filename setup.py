import os
from glob import glob

from setuptools import find_packages, setup

package_name = "camera_calibration"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=open('requirements.txt').read().splitlines(),
    zip_safe=True,
    maintainer="anyone",
    maintainer_email="henryamwilliams@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "calibration_node = camera_calibration.calibration_node:main"
        ],
    },
)

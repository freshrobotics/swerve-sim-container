import os
from glob import glob

from setuptools import setup

# enter package values here
package_name = "swerve_controller"
version = "0.1.0"
description = "ros controller for swerve drive vehicles"
maintainer = "Robotics @ Fresh Consulting"
maintainer_email = "robotics@freshconsulting.com"
license = "TODO"
scripts = [
    "wheel_cmd_publisher = swerve_controller.swerve_commander:main",
    "wheel_odometry = swerve_controller.swerve_odometer:main",
]

# only touch this for special cases
setup(
    name=package_name,
    version=version,
    packages=[package_name],
    data_files=[
        # install marker file in the package index
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        # include our package.xml file
        (os.path.join("share", package_name), ["package.xml"]),
        # include all launch files.
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        # install yaml parameter files
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer=maintainer,
    maintainer_email=maintainer_email,
    description=description,
    license=license,
    tests_require=[],
    entry_points={
        "console_scripts": scripts,
    },
)

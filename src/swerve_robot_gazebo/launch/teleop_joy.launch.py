import os
from typing import Final, List, Optional

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Packages
TELEOP_JOY_PKG: Final[str] = "teleop_twist_joy"
SWERVE_ROBOT_GAZEBO_PKG: Final[str] = "swerve_robot_gazebo"

# Directories
LAUNCH_DIR: Final[str] = "launch"
CONFIG_DIR: Final[str] = "config"

# Files
TELEOP_JOY_LAUNCH: Final[str] = "teleop-launch.py"
TELEOP_JOY_CONFIG: Final[str] = "xbox_holonomic.config.yaml"

# File Paths
teleop_joy_launch_path = os.path.join(
    get_package_share_directory(TELEOP_JOY_PKG), LAUNCH_DIR, TELEOP_JOY_LAUNCH
)
print(teleop_joy_launch_path)
teleop_joy_config_path = os.path.join(
    get_package_share_directory(SWERVE_ROBOT_GAZEBO_PKG), CONFIG_DIR, TELEOP_JOY_CONFIG
)
print(teleop_joy_config_path)

# Launch Arguments
ARGUMENTS: Optional[List[DeclareLaunchArgument]] = []


def generate_launch_description() -> LaunchDescription:
    """
    Include teleop joy launch with appropriate launch args
    """

    print("Prior")
    teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([teleop_joy_launch_path]),
        launch_arguments={
            "config_filepath": teleop_joy_config_path,
        }.items(),
    )
    print("Post")

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(teleop_joy)

    return ld

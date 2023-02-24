import os
from typing import Final, List, Optional

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# ROS Pkg Share
WORLD_PKG: Final[str] = "aws_robomaker_small_warehouse_world"
SWERVE_ROBOT_GAZEBO_PKG: Final[str] = "swerve_robot_gazebo"

# File Names
WORLD_LAUNCH_FILE: Final[str] = "no_roof_small_warehouse.launch.py"
SPAWN_LAUNCH_FILE: Final[str] = "spawn_swerve_robot.launch.py"

# Directory Names
LAUNCH_DIR: Final[str] = "launch"

# File Paths
world_launch_path = os.path.join(
    get_package_share_directory(WORLD_PKG), LAUNCH_DIR, WORLD_LAUNCH_FILE
)
spawn_launch_path = os.path.join(
    get_package_share_directory(SWERVE_ROBOT_GAZEBO_PKG), LAUNCH_DIR, SPAWN_LAUNCH_FILE
)

# Launch Arguments
ARGUMENTS: Optional[List[DeclareLaunchArgument]] = []


def generate_launch_description() -> LaunchDescription:
    """
    Generate Launch Description for Harmony Sim.

    Returns
    -------
        LaunchDescription: Description of the launch-able ROS system

    """
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([world_launch_path]),
    )

    swerve_robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([spawn_launch_path]),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(world_launch)
    ld.add_action(swerve_robot_spawn)

    return ld

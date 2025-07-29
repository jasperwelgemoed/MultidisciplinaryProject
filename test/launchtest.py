#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get shared package directories
    nav_share = get_package_share_directory("mirte_navigation")
    slam_share = get_package_share_directory("slam_toolbox")

    # Path to the YAML file
    params_file = os.path.join(nav_share, "params", "mirte_nav2_params.yaml")

    # Launch Mirte Navigation with params file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "launch", "robot_navigation.launch.py")
        ),
        launch_arguments={"params_file": params_file}.items(),
    )

    # Launch SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "online_async_launch.py")
        )
    )

    return LaunchDescription([nav2_launch, slam_launch])

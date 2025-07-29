#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # 1) Bring up your robot + Nav2 from mirte_navigation
    nav_share = get_package_share_directory('mirte_navigation')
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'robot_navigation.launch.py')
            )
        )
    )

    # 2) Start SLAM Toolbox in online mode
    slam_share = get_package_share_directory('slam_toolbox')
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, 'launch', 'online_async_launch.py')
            )
        )
    )

    return ld

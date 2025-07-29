import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package share directory
    pkg_group03 = get_package_share_directory("group03")

    # Define relative paths for the map and params file
    map_file = os.path.join(pkg_group03, "worlds", "test_gazebo_world_map", "map.yaml")
    params_file = os.path.join(pkg_group03, "params", "nav2_params.yaml")

    # # # Absolute paths for map and Nav2 params
    # map_file = '/home/jverhoog/mdp_sim/src/group03/worlds/test_gazebo_world_map/map.yaml'
    # params_file = '/home/jverhoog/mdp_sim/src/group03/params/nav2_params.yaml'

    # Include the Nav2 localization launch (nav2_bringup)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"]
            )
        ),
        launch_arguments={
            "map": map_file,
            "params_file": params_file,
            "use_sim_time": "false",
        }.items(),
    )

    # Publish initial pose once after a short delay (sets robot at x=0,y=0,theta=0)
    initial_pose_pub = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/initialpose",
                    "geometry_msgs/PoseWithCovarianceStamped",
                    '{header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" }, '
                    "pose: {pose: {position: { x: 0.0, y: 0.0, z: 0.0 }, "
                    "orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }}, "
                    "covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, "
                    "0.0, 0.25, 0.0, 0.0, 0.0, 0.0, "
                    "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
                    "0.0, 0.0, 0.0, 0.0685, 0.0, 0.0, "
                    "0.0, 0.0, 0.0, 0.0, 0.0685, 0.0, "
                    "0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([localization_launch, initial_pose_pub])

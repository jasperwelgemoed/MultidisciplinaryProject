import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    spin_node = TimerAction(
        period=12.0,  # small delay so ROS graph has time to come up
        actions=[
            Node(
                package="group03",  # your package name
                executable="spin_node",  # the name you used in setup.py
                name="spin_node",
                output="screen",
            )
        ],
    )

    late_nodes = TimerAction(
        period=15.0,  # Delay in seconds
        actions=[
            Node(
                package="group03",
                executable="fsm_node",
                name="fsm_node",
                output="screen",
            ),
            Node(
                package="group03",
                executable="farmer_command_node",
                name="farmer_command_node",
                output="screen",
            ),
            Node(
                package="group03",
                executable="navigation_node",
                name="navigation_node",
                output="screen",
            ),
            Node(
                package="group03",
                executable="ik_solver_node",
                name="ik_solver_node",
                output="screen",
            ),
        ],
    )

    sonar_obstacles = Node(
        package="group03",
        executable="sonar_obstacles",
        name="sonar_obstacles",
        output="screen",
    )

    map_file = os.path.abspath(
        "/home/ishita/mdp/src/group03/worlds/test_gazebo_world_map/map.yaml"
    )
    params_file = os.path.abspath(
        "/home/ishita/mdp/src/group03/params/nav2_params.yaml"
    )

    # Localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"]
            )
        ),
        launch_arguments={"map": map_file, "use_sim_time": "false"}.items(),
    )

    # Navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
            )
        ),
        launch_arguments={
            "map": map_file,
            "params_file": params_file,
            "use_sim_time": "false",
        }.items(),
    )

    relay_topic_odom = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/mirte_base_controller/odom", "/odom"],
        output="screen",
    )
    # tf_base_footprint =
    tf_base_frame = TimerAction(
        period=1.0,  # Delay in seconds before starting the initial pose node
        actions=[
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_frame"],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
                output="screen",
            ),
        ],
    )
    # Publish initial pose once after a short delay (sets robot at x=0,y=0,theta=0) AIM MIRTE IN RED AXIS DIRECTION IN GAZEBO
    initial_pose_pub = TimerAction(
        period=10.0,
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
    initial_pose_node = TimerAction(
        period=1.0,  # Delay in seconds before starting the initial pose node
        actions=[
            Node(
                package="mirte_navigation",
                executable="set_initial_pose",
                name="set_initial_pose",
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            spin_node,
            sonar_obstacles,
            late_nodes,
            localization_launch,
            navigation_launch,
            relay_topic_odom,
            tf_base_frame,
            initial_pose_pub,
            initial_pose_node,
        ]
    )

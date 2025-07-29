import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare  

def generate_launch_description():
    # Get the package share directory
    pkg_mirte_navigation = get_package_share_directory('mirte_navigation')
    pkg_mirte_group03 = get_package_share_directory('group03')

    # Define relative paths for the map and params file
    map_file = os.path.join(
        pkg_mirte_group03,
        'worlds',
        'hallway_map',
        'map.yaml')

    # Define relative paths for the map and params file
    map_file = os.path.join(
        pkg_mirte_navigation,
        'maps',
        'map.yaml')
    
    # params_file = os.path.join(
    #     pkg_mirte_navigation,
    #     'params',
    #     'mirte_nav2_params.yaml')

    # Localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"
        ])),
        launch_arguments={'map': map_file}.items()
    )

    # # Navigation launch
    # navigation_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([
    #         FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"
    #     ])),
    #     launch_arguments={
    #         "map": map_file,
    #         "params_file": params_file
    #     }.items()
    # )

    # RViz execution
    rviz_command = ExecuteProcess(
        cmd=["rviz2", "-d", PathJoinSubstitution([
            FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"
        ])],
        output="screen"
    )
    relay_topic_cmd = Node(
        package = "topic_tools",
        executable = "relay",
        arguments=["/cmd_vel", "/mirte_base_controller/cmd_vel"],
        output="screen",
    )
    relay_topic_odom = Node(
        package = "topic_tools",
        executable = "relay",
        arguments=["/mirte_base_controller/odom", "/odom"],
        output="screen",
    )
    # tf_base_footprint = 
    tf_base_frame = TimerAction(
        period=1.0,  # Delay in seconds before starting the initial pose node
        actions=[Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_frame"],
        output="screen",
    ),Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0",  "base_link", "base_footprint"],
        output="screen",
    ),
    ])
    initial_pose_node = TimerAction(
        period=1.0,  # Delay in seconds before starting the initial pose node
        actions=[
            Node(
                package='mirte_navigation',
                executable='set_initial_pose',
                name='set_initial_pose',
                output='screen'
            )
        ]
    )
    return LaunchDescription([
        localization_launch,
        initial_pose_node,
        rviz_command,
        relay_topic_cmd,
        relay_topic_odom,
        tf_base_frame,
    ])
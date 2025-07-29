#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # === 1) Declare exactly the same arguments ===
    arm_enable = DeclareLaunchArgument("arm_enable", default_value="True")
    sonar_enable = DeclareLaunchArgument("sonar_enable", default_value="True")
    lidar_enable = DeclareLaunchArgument("lidar_enable", default_value="True")
    depth_camera_enable = DeclareLaunchArgument(
        "depth_camera_enable", default_value="True"
    )
    visualize_sensors = DeclareLaunchArgument("visualize_sensors", default_value="True")
    gui = DeclareLaunchArgument("gui", default_value="True")
    world = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution(
            [FindPackageShare("group03"), "worlds", "test_gazebo_world.world"]
        ),
    )

    # === 2) Include Gazebo with your saved world ===
    gazebo_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("mirte_gazebo"),
                    "launch",
                    "gazebo_mirte_world_generated.launch.xml",
                ]
            )
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "generated_world": LaunchConfiguration("world"),
        }.items(),
    )

    # === 3) Spawn the MIRTE master ===
    spawn_mirte = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("mirte_gazebo"),
                    "launch",
                    "spawn_mirte_master.launch.xml",
                ]
            )
        ),
        launch_arguments={
            "x": "0.0",
            "y": "0.0",
            "z": "0.0",
            "visualize_sensors": LaunchConfiguration("visualize_sensors"),
        }.items(),
    )

    # === 4) Controller spawners ===
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mirte_arm_controller", "mirte_gripper_controller"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    base_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "mirte_base_controller"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # === 5) twist_mux node ===
    twist_mux_config = PathJoinSubstitution(
        [FindPackageShare("mirte_gazebo"), "config", "twist_mux.yaml"]
    )
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        remappings=[("/cmd_vel_out", "/cmd_vel")],
        parameters=[twist_mux_config, {"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            arm_enable,
            sonar_enable,
            lidar_enable,
            depth_camera_enable,
            visualize_sensors,
            gui,
            world,
            gazebo_launch,
            spawn_mirte,
            arm_spawner,
            base_spawner,
            twist_mux_node,
        ]
    )

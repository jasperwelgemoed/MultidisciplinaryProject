# group03 ROS 2 Package

This package contains ROS 2 nodes, drivers, planners and perception modules for planning and picking apples in an orchard environment, using the Mirte robot in Gazebo. It also includes tools for mapping, localization, and a YOLO-based apple detector training pipeline.

---

## Table of Contents

* [Overview](#overview)
* [Prerequisites](#prerequisites)
* [Installation](#installation)
* [Usage Full Solution](#usage-full-solution)
   * [Running the Full Solution](#running-the-full-solution)
* [Usage for Subsections or the Repository as a Whole](#usage-for-subsections-or-the-repository-as-a-whole)
   * [Running Apple Detection](#running-apple-detection)
   * [Transforms & Relays for Mirte](#transforms--relays-for-mirte)
   * [Mapping](#mapping)
   * [Self-Localization](#self-localization)
   * [Gripper-manager](#gripper-manager)
   * [Adding a Node](#adding-a-node)
   * [Adding a Launch File](#adding-a-launch-file)
* [Detection Model Training](#detection-model-training)
* [Testing](#testing)
* [Repository Structure](#repository-structure)
* [License & Maintainer](#license--maintainer)

---

## Overview

The **group03** ROS 2 package provides:

* A finite state machine (`fsm_node`) for sequencing orchard tasks
* A navigation stack (`navigation_node`) using Nav2
* Obstacle avoidance (`emergency_brake_check`) and emergency brake
* A gripper manager for apple picking (`gripper_manager_node`)
* A apple‐detection publisher setup (`apple_detection_node`)
* Launch files for consistent simulation, navigation, mapping & localization
* Support files: world definitions, parameters, and sound alerts

---

## Prerequisites

* **ROS 2** (tested on **Humble Hawksbill**)
* **Colcon** build tool
* **Python 3** (≥ 3.8)
* ROS 2 Python client libraries and message packages:

  * `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`
  * `cv_bridge`, OpenCV (`cv2`)
  * `launch`, `launch_ros`, `ament_index_python`
* Gazebo plugins and Mirte simulation:

  * `gazebo_ros`, `mirte_gazebo`, `controller_manager`, `twist_mux`
* **Nav2** and **SLAM Toolbox** for mapping & localization
* `topic_tools` and `tf2_ros` for relaying & static transforms

---

## Installation

1. **Clone** into your ROS 2 workspace:

   ```bash
   cd <your_ros2_ws>/src
   git clone <repository_url> group03
   ```

2. **Build** the package:

   ```bash
   cd <your_ros2_ws>
   colcon build --packages-select group03 --merge-install
   ```

3. **Source** the install space:

   ```bash
   source install/setup.bash
   ```

---

## Usage full solution

### Running the full solution

1. Start Mirte up and get all topics locally.

2. Create a static map (see below). 

3. `ros2 launch group03 full_start.launch.py`

#### Overrides for the fsm:

1. Publish a String to “farmer_cmd” (e.g. 'stop', 'return_to_start', 'order_request')

`ros2 topic pub /farmer_cmd std_msgs/msg/String "{data: 'stop'}" --once`

2. Publish a Bool to “obstacle_detected” (true or false)

`ros2 topic pub /obstacle_detected std_msgs/msg/Bool "{data: true}" --once`

3. Publish a Bool to “nav_goal_reached” (true or false)

`ros2 topic pub /nav_goal_reached std_msgs/msg/Bool "{data: false}" --once`

4. Publish an Int32 to “nr_of_apples_detected” (e.g. 0, 1, 2, 3, 4)

`ros2 topic pub /nr_of_apples_detected std_msgs/msg/Int32 "{data: 2}" --once`

5. Publish a Bool to “gripper_above_base” (true or false)

`ros2 topic pub /gripper_above_base std_msgs/msg/Bool "{data: true}" --once`


## Usage for subsections or the repository as a whole

### Gripper manager

This ROS 2 node controls the Mirte robot arm, performing inverse kinematics (IK) for picking and placing apples. It reacts to FSM state commands and vision data, also signaling arm safety.

#### Functionality

* **Arm Control:** Handles IK, joint trajectory execution, and gripper operations.
* **Reactive Behavior:** Responds to FSM states (picking, placing) and apple coordinates.
* **Safety Status:** Publishes `gripper_above_base` for mobile base coordination.

#### Dependencies

`rclpy`, `geometry_msgs`, `std_msgs`, `numpy`, `scipy.optimize`, `control_msgs`, `trajectory_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `visualization_msgs`. Requires `mirte_master_arm_controller` and `mirte_master_gripper_controller` action servers.

#### ROS 2 Interface

##### Subscribed Topics

* `/detections/gripper/apples/coords` (`geometry_msgs/msg/PointStamped`): Apple 3D position.
* `/fsm_state` (`std_msgs/msg/String`): Current FSM state (e.g., "PICKING_APPLE", "PLACING_APPLE").
* `/arm_goal_pose` (`geometry_msgs/msg/Pose`): Manual arm control (optional).

##### Published Topics

* `/gripper_above_base` (`std_msgs/msg/Bool`): Arm safe for base movement (`True`/`False`).
* `/apple_marker` (`visualization_msgs/msg/Marker`): RViz visualization of target.

#### Internal States (`self.stage`)

The `ik_solver_node` manages its own internal sequence of actions using the `self.stage` variable to track its progress:

* `"idle"`: Default state, arm is waiting for an FSM command.
* `"initial_approach_to_apple"`: Moving to a first position near the apple.
* `"pre_grasp"`: Moving to a closer position, ready to grasp.
* `"grasping"`: Closing the gripper.
* `"unplucking_up"`, `"unplucking_down"`: Stages for the unplucking motion.
* `"stow_to_base"`: Moving the arm to a safe, stowed position after picking.
* `"waiting_for_basket"`: Internal state after picking and stowing, waiting for the FSM to signal navigation to the basket.
* `"dropping_apple"`: Moving to the basket drop-off position.
* `"releasing_apple"`: Opening the gripper to release the apple.
* `"return_to_stow"`: Returning to the general stow position after dropping the apple.
* `"manual_control"`: Engaged when receiving a command on `/arm_goal_pose`.

The `self.is_busy` flag tracks if the arm is currently executing a complex multi-step task, preventing new commands from interrupting ongoing operations.

#### Usage / Testing

1.  **Ensure ROS 2 is Sourced:**
    ```bash
    source /opt/ros/humble/setup.bash
    source <your_workspace>/install/setup.bash
    ```
2.  **Run Mirte Controllers (e.g., in simulation):**
    Ensure the `mirte_master_arm_controller` and `mirte_master_gripper_controller` action servers are running. 
3.  **Run the `ik_solver_node`:**
    ```bash
    ros2 run group03 ik_solver_node
    ```
4.  **Simulate Inputs (in separate terminals):**

    * **Simulate Apple Detection:**
        ```bash
        ros2 topic pub /detections/gripper/apples/coords geometry_msgs/msg/PointStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, point: {x: 0.4, y: 0.05, z: 0.20}}' --once
        ```

    * **Simulate FSM for Picking:**
        ```bash
        ros2 topic pub /fsm_state std_msgs/msg/String '{data: "PICKING_APPLE"}' --once
        ```

    * **Simulate FSM for Placing (after a successful pick):**
        ```bash
        ros2 topic pub /fsm_state std_msgs/msg/String '{data: "PLACING_APPLE"}' --once
        ```

    * **Simulate FSM for Idle/Safety (e.g., during navigation):**
        ```bash
        ros2 topic pub /fsm_state std_msgs/msg/String '{data: "NAV_TO_TREE"}' --once
        ```

#### Parameters

You can configure the arm's behavior via ROS parameters:

* `l1`, `l2`, `l3`, `l4`: Link lengths (floats).
* `initial_q`: Default joint angles (list, radians), `[0.0, 0.0, -1.56, 0.0]`.
* `approach_offset`: Pre-grasp distance (float, meters).
* `unpluck_distance`: Unplucking oscillation distance (float, meters).
* `unpluck_cycles`: Number of unplucking oscillations (int).
* `gripper_open_delay_sec`: Delay after gripper open (float, seconds).




---

### Running Apple Detection

1. **Publisher** (example image stream, only if not connected to real Mirte):

   ```bash
   ros2 run group03 talker
   ```

2. **Listener** (processes the images):

   ```bash
   ros2 run group03 listener
   ```

---

### Transforms & Relays for Mirte

When running Gazebo with Mirte, start these in separate terminals:

```bash
ros2 run topic_tools relay /cmd_vel /mirte_base_controller/cmd_vel --ros-args --log-level info
ros2 run topic_tools relay /mirte_base_controller/odom /odom --ros-args --log-level info
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_frame --ros-args --log-level info &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint --ros-args --log-level info &
```

---

### Mapping

*First*, ensure the relays & transforms above are running.

1. **Start SLAM**:

   ```bash
   ros2 launch slam_toolbox online_async_launch.py \
     slam_params_file:=src/group03/params/mapper_params_online_async.yaml
   ```

2. Drive the robot until LiDAR self‐detections disappear.
   The map is stored at `worlds/empty_map/map.yaml`.

3. **Save** your static map:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f map
   ```

---

### Self-Localization

1. **Launch** localization:

   ```bash
   ros2 launch nav2_bringup localization_launch.py \
     map:=<path_to>/map.yaml \
     params_file:=src/group03/params/nav2_params.yaml \
     use_sim_time:=false
   ```

2. **Publish** an initial pose guess:

   ```bash
   ros2 topic pub --once /initialpose \
     geometry_msgs/PoseWithCovarianceStamped '{
       header: { frame_id: "map" },
       pose: {
         pose: {
           position: { x: 0.0, y: 0.0, z: 0.0 },
           orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
         },
         covariance: [
           0.25, 0, 0, 0, 0, 0,
           0, 0.25, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0.0685, 0, 0,
           0, 0, 0, 0.0685, 0, 0,
           0, 0, 0, 0, 0, 0.0685
         ]
       }
     }'
   ```

---

## Detection Model Training

Training notebooks and best‐performing weights are under
`resource/yolo_training/`. The main notebook is:

```bash
resource/yolo_training/yolo_train.ipynb
```

* **Dataset**: 44 training images, 19 validation images (30 % contain trees with/without test apples).
* Additional apple‐only images under various occlusions for robustness.

---


### Adding a Node

1. Create a new Python script in
   `src/group03/group03/`.
   Each script should define one ROS 2 node (via a `main()`).

2. Register it in **setup.py** under `entry_points`:

   ```python
   entry_points={
     'console_scripts': [
       # existing...
       'my_node = group03.my_node:main',
     ],
   },
   ```

3. Rebuild & source:

   ```bash
   colcon build --packages-select group03 --merge-install
   source install/setup.bash
   ```

4. Run your node:

   ```bash
   ros2 run group03 my_node
   ```

---

### Adding a Launch File

1. Create a `.py` in `src/group03/launch/`, e.g. `my_launch.py`.
   Define nodes, remappings, parameters, etc.

2. Make it executable:

   ```bash
   chmod +x src/group03/launch/my_launch.py
   ```

3. Launch:

   ```bash
   ros2 launch group03 my_launch.py
   ```

---

## Testing

Run unit, style, and documentation tests:

```bash
pytest src/group03
flake8 src/group03
pydocstyle src/group03
```

---

## Repository Structure

```
├── launch/                     # ROS 2 launch scripts
├── params/                     # YAML parameter files
├── resource/
│   ├── listen_to_fsm_example.py
│   └── yolo_training/          # YOLO training artifacts & notebooks
├── sounds/                     # Alert sound files
├── worlds/                     # Gazebo world & map files
├── group03/                    # Python package code
├── setup.py
├── package.xml
└── test/                       # Unit & style tests
```

---

## License & Maintainer

* **License**: Apache-2.0
* **Maintainer**: Group03

---

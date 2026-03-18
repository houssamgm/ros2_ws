# diffbot_sim

This package provides a launch file to simulate a differential drive robot (`diffbot`) in a Gazebo environment, perform mapping or localization using a pre-existing map, and run navigation using Nav2.

## Prerequisites

Before using this package, make sure you have the following ROS 2 packages installed:

* `gazebo_ros`: For Gazebo integration.
* `robot_state_publisher`: To publish the robot's joint states.
* `slam_sim`: For SLAM functionalities.
* `navigation2` and `nav2_bringup`: For the Navigation2 stack.

You can install these dependencies using the following command:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros ros-humble-robot-state-publisher ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-navigation2
```

**Note:** Replace `humble` with your ROS 2 distribution name if you are not using Humble Hawksbill.

## Package Structure

The `diffbot_sim` package contains the following relevant directories:

* `launch`: Contains the main launch file (`sim.launch.py`).
* `worlds`: Contains the Gazebo world file (`house.world`).
* `urdf`: Contains the robot description in XACRO format (`robot.xacro.urdf`).
* `config`: Contains configuration files, including the Nav2 parameters (`nav2_params.yaml`).

## Launch File Description

The `sim.launch.py` file performs the following actions:

1.  **Launches Gazebo:** Starts the Gazebo simulator with the `house.world` environment.
2.  **Publishes Robot State:** Loads the robot description from the `robot.xacro.urdf` file, processes it, and starts the `robot_state_publisher` to publish the robot's joint states to the `/robot_description` topic.
3.  **Spawns the Robot:** Uses the `gazebo_ros` tool to spawn the robot model (defined in the URDF) into the Gazebo simulation. The robot will be named `diffbot`.
4.  **Starts Localization:** Includes the `slam.launch.py` file from the `slam_sim` package with specific arguments for mapping or localization:
    * `mode`: Set to `localization` or `mapping`.
    * `use_sim_time`: Set to `true` to use the Gazebo simulation time.
    * `map_file_name`: Specifies the path to the pre-existing map file (`/home/ana/ros2_ws/map/map`). **You will need to replace this with the actual path to your map file.**
    * `map_start_at_dock`: Set to `true` to indicate the robot's initial pose is at a dock.
5.  **Starts Navigation:** Includes a `bringup_navigation.py` launch file (assumed to be in the `diffbot_sim` package) to start the Navigation2 stack:
    * `use_sim_time`: Set to `true`.
    * `log_level`: Set to `ERROR` to reduce the verbosity of the navigation nodes.
    * `params_file`: Specifies the path to the Nav2 parameter file (`config/nav2_params.yaml`).

## How to Use

1.  **Clone the repository:** Clone this package into your ROS 2 workspace.
2.  **Build the package:** Navigate to the root of your workspace and build the package:

    ```bash
    colcon build --packages-select diffbot_sim
    ```

3.  **Source the environment:** Source the setup file for your workspace:

    ```bash
    source install/setup.bash
    ```

4.  **Run the launch file:** Execute the main launch file:

    ```bash
    ros2 launch diffbot_sim diffbot_simulation.launch.py
    ```

This command will start Gazebo with the specified world, load the robot model, spawn it into the simulation, start the robot state publisher, initiate the localization process using the provided map, and launch the Navigation2 stack.

## Customization

* **World File:** You can change the Gazebo world by modifying the `world_file_path` variable in the `generate_launch_description()` function of the launch file. Make sure the specified world file exists in the `worlds` directory.
* **Robot Model:** The robot model is defined in `urdf/robot.xacro.urdf`. You can modify this file to change the robot's physical characteristics.
* **Map File:** Ensure that the `map_file_name` argument in the `slam` IncludeLaunchDescription points to the correct path of your pre-existing map file if you want to perform localizatoin.
* **Navigation Parameters:** The navigation behavior is configured through the `config/nav2_params.yaml` file. You can adjust the parameters in this file to fine-tune the navigation stack.
* **Localization Mode:** The launch file is currently configured for localization. If you want to perform mapping, you would need to modify the arguments passed to the `slam.launch.py` file.

## Notes

* The `bringup_navigation.py` launch file is assumed to be present in the `diffbot_sim/launch` directory. Ensure this file exists and is correctly configured for your robot.
* The path to the map file (`/home/ana/ros2_ws/map/map`) is hardcoded in the launch file. Remember to replace this with the actual path to your map.
* This launch file assumes that the `slam_sim` package provides a `slam.launch.py` file with the specified launch arguments. Refer to the documentation of the `slam_sim` package for more details.
# slam_sim

This package provides a launch file to run the `slam_toolbox` for Simultaneous Localization and Mapping (SLAM). It offers flexibility through various launch arguments to configure the SLAM process for different needs, such as mapping and localization, and to manage the lifecycle of the SLAM node.

## Prerequisites

Before using this package, ensure you have the `slam_toolbox` package installed. You can install it using the following command:

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
```

**Note:** Replace `humble` with your ROS 2 distribution name if you are not using Humble Hawksbill.

## Package Structure

The `slam_sim` package contains the following relevant directories:

* `launch`: Contains the main launch file (`slam.launch.py`).
* `config`: Contains the default parameter file for `slam_toolbox` (`slam_toolbox_params.yaml`). You can modify this file or provide your own.

## Launch File Description

The `slam.launch.py` file launches the `async_slam_toolbox_node` from the `slam_toolbox` package. It exposes several launch arguments to configure its behavior:

* **`autostart` (boolean, default: `true`):**
    * If `true`, the `slam_toolbox` node will automatically configure and activate upon launch (unless `use_lifecycle_manager` is `true`).
    * If `false`, the node will start in an inactive state, and you'll need to manually trigger the configuration and activation (unless `use_lifecycle_manager` is `true`).

* **`use_lifecycle_manager` (boolean, default: `false`):**
    * If `true`, the launch file will not automatically configure and activate the `slam_toolbox` node. You will need to use a lifecycle manager (like the `robot_state_publisher` in the `diffbot_sim` example) to manage the node's state transitions.

* **`use_sim_time` (boolean, default: `true`):**
    * Set to `true` if you are running the SLAM in a simulation environment (like Gazebo) that provides a `/clock` topic.
    * Set to `false` if you are running SLAM with data from a real robot.

* **`slam_params_file` (string, default: `config/slam_toolbox_params.yaml`):**
    * Specifies the full path to the YAML file containing the parameters for the `slam_toolbox` node. The default file is located in the `config` directory of this package. You can create your own parameter file and provide its path here.

* **`mode` (string, default: `mapping`):**
    * Overrides the `mode` parameter defined in the `slam_params_file`. Common modes include `mapping` for creating a new map and `localization` for localizing the robot within a pre-existing map.

* **`map_file_name` (string, default: `map.yaml`):**
    * Overrides the `map_file_name` parameter defined in the `slam_params_file`. This parameter specifies the name of the map file to be saved (in mapping mode) or loaded (in localization mode). The full path where the map is saved or loaded from is usually configured within the `slam_params_file`.

* **`map_start_at_dock` (boolean, default: `false`):**
    * Overrides the `map_start_at_dock` parameter defined in the `slam_params_file`. This parameter might be specific to certain configurations or robots and could indicate if the mapping or localization process should start from a designated "dock" position.

## How to Use

1.  **Clone the repository:** Clone this package into your ROS 2 workspace.
2.  **Build the package:** Navigate to the root of your workspace and build the package:

    ```bash
    colcon build --packages-select slam_sim
    ```

3.  **Source the environment:** Source the setup file for your workspace:

    ```bash
    source install/setup.bash
    ```

4.  **Run the launch file:** Execute the launch file with desired configurations using `ros2 launch`:

    ```bash
    ros2 launch slam_sim slam.launch.py [launch_arguments]
    ```

    Replace `[launch_arguments]` with the specific parameters you want to set. For example:

    * **To start mapping using default parameters:**
        ```bash
        ros2 launch slam_sim slam.launch.py
        ```

    * **To start localization using a specific map file and simulation time:**
        ```bash
        ros2 launch slam_sim slam.launch.py mode:=localization map_file_name:=/path/to/your/map.yaml use_sim_time:=true
        ```

    * **To use a custom parameter file:**
        ```bash
        ros2 launch slam_sim slam.launch.py slam_params_file:=/path/to/your/custom_params.yaml
        ```

## Customization

* **Parameter File:** The primary way to customize the behavior of `slam_toolbox` is by modifying the parameters in the YAML file specified by the `slam_params_file` argument. This file controls various aspects of the SLAM algorithm, such as sensor configuration, scan matching parameters, and loop closure settings.
* **Launch Arguments:** Use the provided launch arguments to quickly switch between different modes (mapping, localization), specify map file names, and control the lifecycle and timing of the SLAM node.
* **Integration with Other Packages:** This launch file is designed to be included by other launch files (as seen in the `diffbot_sim` example). This allows you to integrate the SLAM functionality seamlessly into your robot simulation or real-world setup.

## Notes

* Refer to the documentation of the `slam_toolbox` package for a detailed explanation of the available parameters in the parameter file.
* The `map_file_name` argument only specifies the base name of the map file. The full path where the map is saved or loaded from is typically configured within the `slam_params_file`.
* Ensure that the topics and sensor configurations in your robot setup (either simulated or real) match the configuration specified in the `slam_params_file`.
# diffbot_sim — Working Nav2 + SLAM Run Commands 

## Terminal 1 — Simulation (Gazebo + Robot)

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch diffbot_sim sim.launch.py


##Terminal 2 — Navigation Stack (Nav2)

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch diffbot_sim bringup_navigation.py use_sim_time:=true use_composition:=False

##Terminal 3 — RViz (Nav2 View)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch diffbot_sim rviz.launch.py

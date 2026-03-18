# Installation
## Clone the repository
## 1. Clone the repository

 git clone https://github.com/houssamgm/ros2_ws.git
 
 cd ros2_ws
 
## 2. Install dependencies

rosdep update

rosdep install --from-paths src --ignore-src -r -y

## 3. Build the workspace

colcon build

## 4. Source the workspace

source install/setup.bash


# how to run it 

## Terminal 1 — Simulation (Gazebo + Robot-follower)

### Nav2:

ros2 launch diffbot_sim sim.launch.py use_nav2:=true

### no Nav2:

ros2 launch diffbot_sim sim.launch.py use_nav2:=false


## Terminal 2 leaderbot

ros2 launch diffbot_sim leader_spawn.launch.py


## Terminal 3 leader control

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/leader/cmd_vel


## Terminal 4 (optional) — RViz

ros2 launch diffbot_sim rviz.launch.py


## Terminal 5 — Start the follow node using nav2

ros2 run follow_target_nav2 moving_goal


## Terminal 6 — pid follow node + DWB

ros2 run follow_target_nav2 pid_follow


## Terminal 7  — guide node

ros2 run diffbot_sim guide_node


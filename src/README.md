# diffbot_sim — Working Nav2 + SLAM Run Commands 

## Terminal 1 — Simulation (Gazebo + Robot)

#Nav2 (old behavior):

ros2 launch diffbot_sim sim.launch.py use_nav2:=true

#no Nav2 (PID mode):

ros2 launch diffbot_sim sim.launch.py use_nav2:=false

#leaderbot

ros2 launch diffbot_sim leader_spawn.launch.py

#leader control
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/leader/cmd_vel



##Terminal 3 — RViz (Nav2 View)

ros2 launch diffbot_sim rviz.launch.py

#Terminal 4 — Start the follow node

ros2 run follow_target_nav2 moving_goal

#Terminal5  — pid follow node

ros2 run follow_target_nav2 pid_follow

#Terminal6  — guide node

ros2 run diffbot_sim guide_node


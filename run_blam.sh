#!/bin/bash
SESSION=BLAM
WORKSPACE=~/opshack_ws
BAGFILE=/home/costar/Data/full_tunnel_hand-carry_no_rs_2_2019-01-19-laser_imu.bag

## Start up
tmux -2 new-session -d -s $SESSION

tmux split-window -v -p 75
tmux select-pane -t 1
tmux split-window -v -p 75
tmux select-pane -t 0
tmux split-window -h
tmux select-pane -t 2
tmux split-window -h

# Start ROSCORE
tmux send-keys -t 0 "roscore" C-m

# Start ORBSLAM
tmux send-keys -t 2 "sleep 3;rosparam set /use_sim_time true; source $WORKSPACE/devel/setup.bash;roslaunch blam_example exec_online.launch robot_namespace:=husky" C-m

# Place rosbag
tmux send-keys -t 1 "rosparam set /use_sim_time true; rosbag play -r 1 -s 2 $BAGFILE --clock --prefix=husky" C-m
# tmux send-keys -t 1 "rosbag play -r 1 -s 2 $BAGFILE --prefix=husky"


# Prep tf record script
tmux send-keys -t 3 "sleep 2; rosparam set /use_sim_time true; rviz -d $WORKSPACE/src/husky_core/localizer_blam/internal/src/blam_example/rviz/lidar_slam_husky.rviz" C-m
# tmux send-keys -t 3 "sleep 2; rviz -d $WORKSPACE/src/localizer_blam/internal/src/blam_example/rviz/lidar_slam_husky.rviz" C-m

# Prep close script
tmux send-keys -t 4 "rosparam set /use_sim_time true;source $WORKSPACE/devel/setup.bash; roscd blam_slam/../loop_closure_tools" C-m
tmux send-keys -t 4 "python add_factor.py "

# Static transform publisher
tmux select-pane -t 4
tmux split-window -h
#tmux send-keys -t 5 "rosparam set /use_sim_time true;sleep 10s;rosrun tf static_transform_publisher 0 0 0 0 0 0 /husky/base_link /husky/velodyne 10" C-m 
tmux send-keys -t 5 "rosparam set /use_sim_time true;sleep 10s;rosrun tf static_transform_publisher 0 0 0 0 0 0 /husky/base_link /velodyne 10" C-m 
# tmux send-keys -t 5 "rosrun tf static_transform_publisher 0 0 0 0 0 0 /husky/base_link /velodyne" C-m 
tmux select-pane -t 5
tmux split-window -v
tmux send-keys -t 6 "rosparam set /use_sim_time true;sleep 10s;rosrun tf static_transform_publisher 0 0 0 0 0 0 /world /husky/blam 10" C-m 

tmux select-pane -t 1

tmux -2 attach-session -t $SESSION

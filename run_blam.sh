#!/bin/bash
SESSION=BLAM
WORKSPACE=~/postdoc/SubT/subt_ws
BAGFILE=/home/bjm/postdoc/data/Eagle_Mine/full_tunnel_hand-carry_no_rs_2_2019-01-19-laser_imu.bag

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
tmux send-keys -t 2 "sleep 1; source $WORKSPACE/devel/setup.bash;roslaunch blam_example exec_online_remote.launch robot_namespace:=husky" C-m

# Place rosbag
tmux send-keys -t 1 "rosbag play -r 1 -s 2 $BAGFILE --prefix=husky" 


# Prep tf record script
tmux send-keys -t 3 "sleep 2; rviz -d $WORKSPACE/src/localizer_blam/internal/src/blam_example/rviz/lidar_slam_husky.rviz" C-m

# Prep close script
tmux send-keys -t 4 "source $WORKSPACE/devel/setup.bash; cd $WORKSPACE/src/localizer/localizer_blam/internal/src/human_loop_closure" C-m
tmux send-keys -t 4 "python manual_graph_edge.py "

# Static transform publisher
tmux select-pane -t 4
tmux split-window -h
tmux send-keys -t 5 "static_transform_publisher 0 0 0 0 0 0 1 /husky/base_link /velodyne" C-m 


# place cursor in image pub
tmux select-pane -t 1

tmux -2 attach-session -t $SESSION

#!/bin/bash
SESSION=lion_and_lofrontend

ROBOT_NAMESPACE=husky
IMU_TOPIC=/imu/data
POSE_TOPIC=/Robot_7/pose  # check type

LION_PATH=$(rospack find localizer_lion)
LO_FRONTEND_PATH=$(rospack find localizer_lo_frontend)

#### Session settings ####
tmux -2 new-session -d -s $SESSION
tmux set -wg pane-border-status top
tmux display-pane;
tmux set -g  mouse on 

#### Panel structure ####
## Create three rows in the whole panel
tmux select-pane -t 0 # Sensor raw
tmux split-window -h  -p 80 # split horizontally (the newly created row will be 33% of screen)
tmux select-pane -t 0
tmux split-window -h  -p 80 # split horizontally

# Split first column in four rows
tmux select-pane -t 0
tmux split-window -v   -p 33 # split horiz
tmux select-pane -t 0
tmux split-window -v   -p 50 # split horiz
tmux select-pane -t 0
tmux split-window -v   -p 50 # split horiz


# Split second column in two
tmux select-pane -t 4 
tmux split-window -v  -p 10 # split vertically


# Split second cell in two 
#tmux select-pane -t 3 
#tmux split-window -v -p 23

# Split last column in four
tmux select-pane -t 6
tmux split-window -v   -p 33 # split horiz
tmux select-pane -t 6
tmux split-window -v   -p 50 # split horiz
tmux select-pane -t 6
tmux split-window -v   -p 50 # split horiz

##### Roscore and set sim time true is unconventionally placed in the third panel
tmux send-keys -t 0 "roscore" C-m
tmux send-keys -t 6 "sleep 2; rosparam set /use_sim_time true" C-m

##### Play rosbag
tmux send-keys -t 1 "sleep 3; rosbag play -r 1.0 ~/bags/lo_frontend/slow1.bag --clock --pause" C-m 

tmux send-keys -t 2 "sleep 3; rostopic hz -w 3 $POSE_TOPIC" C-m # LION pose input rate
tmux send-keys -t 3 "sleep 3; rostopic hz -w 3 $IMU_TOPIC" C-m # LION imu input rate

##### Launch lion on terminal waiting for enter command
tmux send-keys -t 4 "sleep 2; roslaunch /home/matteo/andrea_ws/src/localizer_lion/launch/lo_frontend_lion.launch \
 robot_namespace:=$ROBOT_NAMESPACE imu_topic:=$IMU_TOPIC pose_topic:=$POSE_TOPIC" C-m # This will launch the node to convert vicon as well
tmux send-keys -t 5 "sleep 2; rostopic hz -w 3 /$ROBOT_NAMESPACE/lion/odom" C-m # LION output odom rate

##### RQT and RVIZ
tmux send-keys -t 7 "sleep 4; rosrun rqt_multiplot rqt_multiplot --multiplot-run-all --clear-config --multiplot-config /home/matteo/andrea_ws/src/localizer_lo_frontend/lo_frontend/testing/rqt_multiplot.xml" C-m
tmux send-keys -t 8 "sleep 4; rosrun rqt_multiplot rqt_multiplot --multiplot-run-all --clear-config --multiplot-config $LION_PATH/testing/rqt_multiplot_husky_covariance_and_calibration.xml" C-m
#tmux send-keys -t 8 "sleep 4; rosrun rviz rviz -d  $LION_PATH/rviz_cfg/lion.rviz" C-m
# Position 
# Velocity 
# Imu bias
# Imu gyro 

tmux -2 attach-session -t $SESSION


#rosbag play BLAM_IMU.bag -r 0.5 --clock
#roslaunch lion angel_dataset_blam.launch 
#rosrun rqt_multiplot rqt_multiplot rqt_multiplot_angel_dataset.xml

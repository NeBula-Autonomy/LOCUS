#!/bin/bash
SESSION=lo_frontend_evaluation
ROBOT_NAME=husky

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
tmux select-pane -t 6
tmux split-window -v   -p 50 # split horiz

##### Roscore and set sim time true is unconventionally placed in the third panel
tmux send-keys -t 0 "roscore" C-m
#tmux send-keys -t 6 "sleep 1; rosparam set /use_sim_time true" C-m

##### Play rosbag
tmux send-keys -t 1 "sleep 2; rosparam set /use_sim_time true && rosbag play -r 1.0 ~/bags/lo_frontend/slow1.bag --clock --pause /velodyne_points:=/$ROBOT_NAME/velodyne_points" C-m
tmux send-keys -t 2 "sleep 2; rostopic hz -w 3 /$ROBOT_NAME/velodyne_points" C-m # LIDAR RATE
tmux send-keys -t 3 "sleep 2; python $(rospack find lo_frontend)/testing/subtract_offset_from_groundtruth.py" C-m 


##### Launch lo_frontend
tmux send-keys -t 4 "sleep 2; roslaunch lo_frontend lo_frontend.launch robot_namespace:=$ROBOT_NAME" C-m
tmux send-keys -t 5 "sleep 2; rostopic hz -w 3 /$ROBOT_NAMESPACE/lo_frontend/odometry_integrated_estimate" C-m # front-end output odom rate

##### rqt_multiplot and conversion tools for visualization
tmux send-keys -t 7 "sleep 2; rosrun rqt_multiplot rqt_multiplot --multiplot-run-all --clear-config --multiplot-config $(rospack find lo_frontend)/testing/rqt_multiplot.xml" C-m
tmux send-keys -t 8 "sleep 2; rosrun tf2_ros static_transform_publisher 0 0 -0.08 0 0 0 1 /velodyne $ROBOT_NAME/base_link" C-m
tmux send-keys -t 9 "sleep 5; rosrun topic_tools transform /Robot_7/pose /Robot_7/pose/rpy  geometry_msgs/Vector3 'tf.transformations.euler_from_quaternion([m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w])' --import tf" C-m
tmux send-keys -t 6 "sleep 5; rosrun topic_tools transform /$ROBOT_NAME/lo_frontend/odometry_integrated_estimate /$ROBOT_NAME/lo_frontend/odometry_integrated_estimate/rpy geometry_msgs/Vector3 'tf.transformations.euler_from_quaternion([m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w])' --import tf" C-m
# Position 
# Velocity 
# Imu bias
# Imu gyro 

tmux -2 attach-session -t $SESSION




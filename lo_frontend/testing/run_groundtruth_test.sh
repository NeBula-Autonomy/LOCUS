#!/bin/bash
SESSION=matteo
ROBOT_NAME=husky

#### tmux ####
tmux -2 new-session -d -s $SESSION
tmux set -wg pane-border-status top
tmux display-pane;
tmux set -g  mouse on 
tmux select-pane -t 0 
tmux split-window -h  -p 80 
tmux select-pane -t 0
tmux split-window -h  -p 80 
tmux select-pane -t 0
tmux split-window -v   -p 33
tmux select-pane -t 0
tmux split-window -v   -p 50 
tmux select-pane -t 0
tmux split-window -v   -p 50 
tmux select-pane -t 4 
tmux split-window -v  -p 10 
tmux select-pane -t 6
tmux split-window -v   -p 33 
tmux select-pane -t 6
tmux split-window -v   -p 50
tmux select-pane -t 6
tmux split-window -v   -p 50 

#### Start ROSCORE 
tmux send-keys -t 0 "roscore" C-m

#### Play the BAGFILE 
tmux send-keys -t 1 "sleep 2; rosparam set /use_sim_time true && rosbag play -r 1.0 ~/bags/lo_frontend/slow1.bag --clock --pause /velodyne_points:=/$ROBOT_NAME/velodyne_points" C-m

#### Launch LO_FRONTEND 
tmux send-keys -t 2 "sleep 2; roslaunch lo_frontend lo_frontend_groundtruth.launch robot_namespace:=$ROBOT_NAME" C-m

#### Execute .py script to convert quaternion to rpy 
tmux send-keys -t 3 "sleep 2; roslaunch quat2eul quat2eul.launch" C-m

#### Execute .py script to remove offset from visualization of groundtruth 
tmux send-keys -t 4 "sleep 5; python $(rospack find lo_frontend)/testing/subtract_offset_from_groundtruth.py" C-m 

#### Run rqt for visualization 
tmux send-keys -t 5 "sleep 2; rosrun rqt_multiplot rqt_multiplot --multiplot-run-all --clear-config --multiplot-config $(rospack find lo_frontend)/testing/rqt_multiplot.xml" C-m
tmux send-keys -t 6 "sleep 2; rosrun tf2_ros static_transform_publisher 0 0 -0.08 0 0 0 1 /velodyne $ROBOT_NAME/base_link" C-m
tmux send-keys -t 7 "sleep 5; rosrun topic_tools transform /Robot_7/pose /Robot_7/pose/rpy  geometry_msgs/Vector3 'tf.transformations.euler_from_quaternion([m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w])' --import tf" C-m
tmux send-keys -t 8 "sleep 5; rosrun topic_tools transform /$ROBOT_NAME/lo_frontend/odometry_integrated_estimate /$ROBOT_NAME/lo_frontend/odometry_integrated_estimate/rpy geometry_msgs/Vector3 'tf.transformations.euler_from_quaternion([m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w])' --import tf" C-m

tmux -2 attach-session -t $SESSION
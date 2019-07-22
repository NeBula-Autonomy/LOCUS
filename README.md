# LO-frontend
LO-fronted computes relative poses from Lidar scans, using prior information provided by a cascaded state (pose, velocity, angular velocity) estimator

## Dependencies.
Please download and install LAMP in the same workspace, as LO-frontend depends on packages available in LAMP.

## Build Instructions
Build this package in a catkin workspace 
```bash
mkdir -p catkin_ws/src
cd catkin_ws
catkin config --extend /opt/ros/kinetic
```
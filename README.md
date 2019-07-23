# LO-frontend
LO-fronted computes relative poses from Lidar scans, using prior information provided by a cascaded state (pose, velocity, angular velocity) estimator.

## Dependencies
Please download and install LAMP in the same workspace, as LO-frontend depends on packages available in LAMP.

## Build Instructions
Build this package in a catkin workspace 
```bash
mkdir -p catkin_ws/src
cd catkin_ws
catkin config --extend /opt/ros/kinetic
```

## Run a tests with ground truth
Download test bags from [here](https://drive.google.com/drive/folders/1dPy667dAnJy9wgXmlnRgQZxQF_ESuve3) (credits to LIO-mapping).
Create a new directory in your user folder: 
```bash
mkdir -p ~/bags/lo_frontend
```
and unzip and move the downloaded files there. 
Launch the script from:
```bash
sh $(rospack find lo_frontend)/testing/run_test.sh
```

## Run the simulation to test integration with lamp
- install simulation following instructions in [core_workspace](https://gitlab.robotics.caltech.edu/rollocopter/core/core_workspace)
- from core_workspace package, checkout branch $feature/lo_split_architecture$
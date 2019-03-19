# B(erkeley) L(ocalization) A(nd) M(apping)!

![alt text](https://github.com/erik-nelson/blam/raw/master/blam_mosaic.png)

***BLAM!*** is an open-source software package for LiDAR-based real-time 3D localization and mapping. ***BLAM!*** is developed by Erik Nelson from the Berkeley AI Research Laboratory ([BAIR](http://bair.berkeley.edu)). See https://youtu.be/08GTGfNneCI for a video example.

## Build Instructions
Build this package in a catkin workspace 
```bash
mkdir -p catkin_ws/src
cd catkin_ws
catkin config --extend /opt/ros/kinetic
```

### Dependencies
This package requires the core_messages package to be build:
```bash
cd ~/catkin_ws/src
git clone https://gitlab.robotics.caltech.edu/rollocopter/core/core_messages.git
catkin build pose_graph_msgs
```

***BLAM!*** relies on system installations of the following packages:

* [ROS](http://wiki.ros.org/ROS/Installation)
* [GTSAM](https://collab.cc.gatech.edu/borg/gtsam)

We recommend:
* Clone GTSAM in your *home* folder and checkout the feature branch:   
        ```bash
        cd
        git clone -b feature/improvementsIncrementalFilter --single-branch https://bitbucket.org/gtborg/gtsam
        ```
* Checkout a specific commit to solve a build issue:
        ```bash
        cd gtsam
        git checkout c827d4cd6b11f78f3d2d9d52b335ac562a2757fc
        ```
* Build
        ```bash
        cd gtsam 
        mkdir build
        cd build
        cmake ..
        $ optional: sudo make check
        sudo make install
        ```

`OLD ADVICE ON BLAM`:
GTSAM in particular should be installed from source using the latest version of the develop branch from https://bitbucket.org/gtborg/gtsam. GTSAM relies on Boost, an incorrect version of which will interfere with some of ROS' packages if ROS is not upgraded to at least Indigo. ROS Indigo, in turn, relies on Ubuntu 14.04.


### Building BLAM
This repository contains the checked-out repositories that were installed via rosinstall in the original ***BLAM!*** repository.
With these changes, the following commands build the entire ***BLAM!*** stack:
```bash
cd ~/catkin_ws/src
git clone https://gitlab.robotics.caltech.edu/rollocopter/localizer/localizer_blam.git
catkin build blam_slam
catkin build point_cloud_visualizer
catkin build blam_example
```

The following significant changes were made to the build process:
* Projects using PCL are now including `${PCL_LIBRARIES}` in their respective `CMakeLists.txt`.
* All `CMakelists.text` are set to build in Release



## Run Instructions
***BLAM!*** is written in C++ with some Python interface elements, wrapped by
Robot Operating System ([ROS](http://ros.org)). Input LiDAR data should be
provided to the `/velodyne_points` topic using message type `sensor_msgs::PointCloud2`.

To run in for SubT operations, use (replacing "husky" with the robot name)

```bash
roslaunch blam_example exec_online.launch robot_namespace:=husky
```

When running with a bagfile, the lidar data should be on `/husky/velodyne_points` when the `robot_namespace` is `husky`.
Also, when running on a bagfile, a static transform publisher is needed, to take place of the robot description:

```bash
static_transform_publisher 0 0 0 0 0 0 1 /husky/base_link /velodyne
```

To run in online mode (e.g. by replaying a bag file from another terminal or
using a real-time sensor stream), use

```bash
roslaunch blam_example test_online.launch
```

To run in offline mode, i.e. by loading a bagfile and processing its data as
fast as possible, set the bagfile name and scan topic in
`blam_example/launch/test_offline.launch`, and use

```bash
roslaunch blam_example test_offline.launch
```

An example .rviz configuration file is provided under
`blam_example/rviz/lidar_slam.rviz`.

![LAMP-logo](https://gitlab.robotics.caltech.edu/rollocopter/localizer/localizer_blam/raw/master/LAMP-logo.png)

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

This package requires `minizip` to be available globally. It can be installed from the package manager via:
```bash
apt install libminizip-dev
```

***BLAM!*** relies on system installations of the following packages:

* [ROS](http://wiki.ros.org/ROS/Installation)
* [GTSAM](https://collab.cc.gatech.edu/borg/gtsam)

We recommend:
Clone GTSAM in your *home* folder and checkout the feature branch:   
```bash
cd
git clone -b feature/improvementsIncrementalFilter --single-branch https://bitbucket.org/gtborg/gtsam
```
Checkout a specific commit to solve a build issue:
```bash
cd gtsam
git checkout c827d4cd6b11f78f3d2d9d52b335ac562a2757fc
```
Build
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

To run, use (replacing "husky" with the robot name)

```bash
roslaunch blam_example exec_online.launch robot_namespace:=husky
```

When running with a bagfile, the lidar data should be on `/husky/velodyne_points` when the `robot_namespace` is `husky`.

To remap the bagfile, run

```bash
rosbag play bagfile.bag --prefix husky
```

Also, when running on a bagfile, a static transform publisher is needed, to take place of the robot description:

```bash
static_transform_publisher 0 0 0 0 0 0 1 /husky/base_link /velodyne
```

In addition, a static transform publisher is needed to take place of the tf from world to blam:

```bash
static_transform_publisher 0 0 0 0 0 0 1 /world /husky/blam
```
**Note:** 
If you are using the ``run_blam.sh`` script, there is no need to run the static transform publisher from `/world` to `/husky/blam` as this is already captured in the script.

To visualize in RViz, use the husky rviz file:
```bash
rviz -d {filepath}/localizer_blam/internal/src/blam_example/rviz/lidar_slam_husky.rviz
```

Alternatively, just run the tmux script (after modifying the parameters at the top of the file):
```bash
./run_blam.sh
```

## (Optional) Running TBB and MKL:
Follow these steps for downloading MKL package:

Downloading the GnuPG key first and add it to the keyring:
```
cd /tmp
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
sh -c 'echo deb https://apt.repos.intel.com/mkl all main > /etc/apt/sources.list.d/intel-mkl.list'
```

After this step for avoiding any error update your repository once more.
```
apt-get update
```

And then:
```
apt-get install intel-mkl-64bit-2018.2-046
```
This is the 64-bit of the MKL.

**Note:**
MKL package at least requires 500MB packages. If you are running out of space, it is not required to risk it.



For the purpose of enabling the TBB package follow these commands:
```
sudo apt-get install libtbb-dev
```

and then

```
cd ~/ws/gtsam/cmake
```

Add these two commands to the CMakeLists.txt of gtsam and then rebuild your gtsam.
```
FindMKL.cmake
FindTBB.cmake 
```

**Note:** By applying both the packages, there are still crashes you will be seeing. It is provided by the developer that these two packages are still under the development.

**Note:** There are not consistancy in TBB package. %70 cases used the MKL and TBB and perfectly working with enhancement in lowering the computation. There are cases of software crashing.


# OLD
To run in online mode (e.g. by replaying a bag file from another terminal or
using a real-time sensor stream), use

```bash
roslaunch blam_example test_online.launch
```

An existing pose graph zip-file generated by the `save_graph` service (as documented in the [loop_closure_tools](https://gitlab.robotics.caltech.edu/rollocopter/localizer/localizer_blam/tree/feature/save_graph/internal/src/loop_closure_tools) module) can be restored by providing the filename as command-line argument:

```bash
roslaunch blam_example test_online.launch load_pose_graph_file:=pose_graph.zip
```

To run in offline mode, i.e. by loading a bagfile and processing its data as
fast as possible, set the bagfile name and scan topic in
`blam_example/launch/test_offline.launch`, and use

```bash
roslaunch blam_example test_offline.launch
```

An example .rviz configuration file is provided under
`blam_example/rviz/lidar_slam.rviz`.

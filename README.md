# LOCUS

LOCUS (Lidar Odometry for Consistent operation in Uncertain Settings) is a Multi-Sensor Lidar-Centric Solution for High-Precision Odometry and 3D Mapping in Real-Time.

![alt text](readme.png)

# Build Instructions

## Native Build

Install [ROS](http://wiki.ros.org/ROS/Installation)

Install catkin tools
```
sudo apt-get install ros-kinetic-catkin python-catkin-tools python3-catkin-tools
```

Install PCL 
```
sudo apt-get install ros-melodic-pcl-ros # for the melodic distro - Ubuntu 18.04
sudo apt-get install ros-noetic-pcl-ros # for the noetc distro - Ubuntu 20.04
```

Install `tf2_sensor_msgs`  and `tf2_geometry_msgs`
```
# for the melodic distro - Ubuntu 18.04
sudo apt install ros-melodic-tf2-sensor-msgs 
sudo apt install ros-melodic-tf2-geometry-msgs 
# for the noetic distro - Ubuntu 20.04
sudo apt install ros-noetic-tf2-sensor-msgs 
sudo apt install ros-noetic-tf2-geometry-msgs 
```

Build this package in a catkin workspace, e.g. 

```bash
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release 
cd src
git clone git@github.com:NeBula-Autonomy/LOCUS.git
catkin build locus
```

## Dockerfile
The docker provides an alternative, contained method for installation

Go to the docker folder, and
```
./cmd locus build
./cmd locus run
./cmd locus bash
cd ~/locus_ws/
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
```



# Running Instructions

To do 


# TODOS

- [x] Rename all lo_frontend instances to locus
- [ ] Cleanup comments, the less the better, just keep the very relevant ones
- [ ] Cleanup unused parameters, class variables, publishers and subscribers 
- [ ] Bring in MDC in launch
- [ ] Remove filter_.Filter call in LidarCallback as nothing is being filtered internally anymore and remove point_cloud_filter package accordingly
- [ ] Simplify our robot-specific files for a generic robot (e.g. no husky/spot anymore) and define how you want to let the user load sensor extrinsic, either hardcoded or by TF
- [ ] Suggest parameters for open/closed space in yaml where relevant (e.g. PointCloudOdometry, PointCloudLocalization)
- [ ] Update README.md with instructions on how to setup/run the package


# Cite
```
@article{reinke2022iros,
  title={LOCUS 2.0: Robust and Computationally Efficient LiDAR Odometry for Real-Time Underground 3D Mapping},
  author={Andrzej Reinke, Matteo Palieri, Benjamin Morrell, Yun Chang, Kamak Ebadi, Luca Carlone, Ali-akbar Agha-mohammadi},
  journal={TODO},
  year={2022},
  publisher={TODO}
}
```

# Old 


# Prerequisites

`tf2_sensor_msgs` may not be installed by default, so install with:
```
sudo apt install ros-$(rosversion -d)-tf2-sensor-msgs
```

# Testing in Ubuntu 20.04 on Dell Precision 



### ROS Install summary
http://wiki.ros.org/noetic/Installation/Ubuntu
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop
```

Source ros to bashrc
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

Ros build tools 
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

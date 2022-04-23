# Testing in Ubuntu 20.04 on Dell Precision 


## Install steps

- install git (can ignore)
- 



### Install ROS
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


```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone git@github.com:NeBula-Autonomy/LOCUS.git
```
FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    wget \
    python3 \
    python3-pip \
    tmux \ 
    tmuxp \
    gdb \
    python-is-python3 \
    python3-catkin-tools \
    ros-noetic-tf2-sensor-msgs

RUN  pip3 install -U catkin_tools

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

CMD ["bash"]


FROM ros:humble-ros-core-jammy

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,display

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    wget \ 
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*
# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO
# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update
# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* && apt-get ros-humble-xacro && apt-get ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*
# install gazebo Garden
RUN apt-get install lsb-release wget gnupg
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y gz-garden 

# Build and install FLANN
RUN git clone https://github.com/mariusmuja/flann.git /opt/flann
RUN mkdir -p /opt/flann/build && \
    cd /opt/flann/build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

RUN source /opt/ros/humble/setup.bash
#sudo docker run -it  -e DISPLAY=:1  -v /tmp/.X11-unix:/tmp/.X11-unix --gpus all  test

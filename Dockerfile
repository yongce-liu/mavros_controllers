FROM osrf/ros:noetic-desktop-full

#####################################################################
# FOR UBUNTU MIRRORS AMD
RUN sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list && \
    apt-get update --fix-missing && apt-get upgrade -y
######################################################################

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git \
    lsb-release \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-mavros-msgs \
    protobuf-compiler \
    libeigen3-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \ 
    gstreamer1.0-libav \
    gstreamer1.0-plugins-* \
    python3-pip \
    python3-catkin-tools \
    python3-rosinstall-generator && \
    rm -rf /var/lib/apt/lists/*

RUN pip install -i https://mirrors.ustc.edu.cn/pypi/simple pip -U && \
    pip config set global.index-url https://mirrors.ustc.edu.cn/pypi/simple

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh

RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools

RUN cd PX4-Autopilot && \
    DONT_RUN=1 make px4_sitl_default gazebo-classic

SHELL ["/bin/bash", "-c"]
RUN mkdir -p catkin_ws && \
    mkdir -p ros_ws/src && \
    cd ros_ws/src && \
    git clone https://github.com/catkin/catkin_simple && \
    git clone https://github.com/ethz-asl/eigen_catkin && \
    git clone https://github.com/ethz-asl/mav_comm && \
    cd .. && \
    source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release

COPY ./src catkin_ws/src
RUN cd catkin_ws && \
    source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release

RUN cat <<EOF >> ~/.bashrc
source /PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash /PX4-Autopilot /PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export PATH=$PATH:/PX4-Autopilot/build/px4_sitl_default/bin
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
source /catkin_ws/devel/setup.bash
EOF

CMD ["/bin/bash"]

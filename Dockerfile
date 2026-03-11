###############################################################################
# mavros_controllers – ROS Noetic + PX4 SITL + Gazebo 11
# Ref: https://docs.px4.io/v1.14/en/simulation/ros_interface.html
###############################################################################
FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ── 1. Basic tooling ────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        wget \
        curl \
        lsb-release \
        gnupg2 \
        python3-pip \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-catkin-tools \
        python3-vcstool \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# ── 2. Gazebo 11 ────────────────────────────────────────────────────────────
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable focal main" \
    > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget -qO - https://packages.osrfoundation.org/gazebo.key | apt-key add - && \
    apt-get update && apt-get install -y --no-install-recommends \
        gazebo11 \
        libgazebo11-dev \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# ── 3. MAVROS + ROS packages ────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-noetic-mavros \
        ros-noetic-mavros-extras \
        ros-noetic-mavros-msgs \
        ros-noetic-mavlink \
        ros-noetic-tf \
        ros-noetic-tf2-ros \
        ros-noetic-dynamic-reconfigure \
        ros-noetic-rviz \
        ros-noetic-rqt \
        ros-noetic-rqt-common-plugins \
        ros-noetic-message-generation \
        ros-noetic-message-runtime \
        ros-noetic-std-msgs \
        ros-noetic-geometry-msgs \
        ros-noetic-sensor-msgs \
        ros-noetic-nav-msgs \
        ros-noetic-eigen-conversions \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets (required by MAVROS)
RUN wget -qO - https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash

# ── 4. PX4 Autopilot (v1.14) ────────────────────────────────────────────────
RUN git clone --recursive --branch v1.14.3 --depth 1 \
        https://github.com/PX4/PX4-Autopilot.git /opt/px4 && \
    cd /opt/px4 && \
    # Install PX4 build dependencies
    bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools && \
    # Build SITL with Gazebo
    DONT_RUN=1 make px4_sitl_default gazebo-classic && \
    rm -rf /var/lib/apt/lists/*

# ── 5. Catkin workspace ─────────────────────────────────────────────────────
RUN mkdir -p /root/catkin_ws/src

# Clone rosinstall dependencies (catkin_simple, mav_comm, eigen_catkin)
COPY dependencies.rosinstall /tmp/dependencies.rosinstall
RUN cd /root/catkin_ws/src && \
    wstool init . /tmp/dependencies.rosinstall

# Copy mavros_controllers source
COPY controller_msgs      /root/catkin_ws/src/mavros_controllers/controller_msgs
COPY geometric_controller  /root/catkin_ws/src/mavros_controllers/geometric_controller
COPY trajectory_publisher  /root/catkin_ws/src/mavros_controllers/trajectory_publisher
COPY mavros_controllers    /root/catkin_ws/src/mavros_controllers/mavros_controllers

# Build the workspace
RUN source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    catkin build --no-status --force-cmake

# ── 6. Entrypoint setup ─────────────────────────────────────────────────────
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/noetic/setup.bash\n\
source /root/catkin_ws/devel/setup.bash\n\
# PX4 environment\n\
source /opt/px4/Tools/simulation/gazebo-classic/setup_gazebo.bash /opt/px4 /opt/px4/build/px4_sitl_default\n\
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/opt/px4:/opt/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

FROM osrf/ros:noetic-desktop-full

#####################################################################
# FOR UBUNTU MIRRORS AMD
RUN sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list && \
    apt-get update --fix-missing && apt-get upgrade -y
######################################################################
# INSTALL ZSH
RUN apt-get update && apt-get install zsh git tmux -y && chsh -s /bin/zsh && \
    git clone https://github.com/robbyrussell/oh-my-zsh.git ${HOME}/.oh-my-zsh \
    && cp ${HOME}/.oh-my-zsh/templates/zshrc.zsh-template ${HOME}/.zshrc && \
    git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-${HOME}/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
    git clone https://github.com/zsh-users/zsh-autosuggestions.git ${ZSH_CUSTOM:-${HOME}/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    sed -i 's/plugins=(git)/plugins=(git zsh-syntax-highlighting zsh-autosuggestions)/g' ${HOME}/.zshrc
SHELL [ "/bin/bash", "-c" ]
######################################################################

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    lsb-release \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-mavros-msgs \
    protobuf-compiler \
    libeigen3-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    gstreamer1.0-plugins-* \
    python3-pip \
    python3-catkin-tools \
    python3-rosinstall-generator && \
    rm -rf /var/lib/apt/lists/*

RUN pip install -i https://mirrors.ustc.edu.cn/pypi/simple pip -U && \
    pip config set global.index-url https://mirrors.ustc.edu.cn/pypi/simple

# RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
COPY PX4-Autopilot /PX4-Autopilot
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh

RUN cd PX4-Autopilot && \
    DONT_RUN=1 make px4_sitl_default gazebo-classic

RUN mkdir -p catkin_ws/src
COPY ./controller_msgs catkin_ws/src/controller_msgs
COPY ./geometric_controller catkin_ws/src/geometric_controller
COPY ./mavros_controllers catkin_ws/src/mavros_controllers
COPY ./trajectory_publisher catkin_ws/src/trajectory_publisher

RUN cd catkin_ws/src && \
    git clone https://github.com/catkin/catkin_simple && \
    git clone https://github.com/ethz-asl/eigen_catkin && \
    git clone https://github.com/ethz-asl/mav_comm

RUN cd catkin_ws && \
    source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release

RUN cat <<EOF >> ~/.zshrc
source /opt/ros/noetic/setup.zsh
source /catkin_ws/devel/setup.zsh
source /PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.zsh /PX4-Autopilot /PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
EOF

CMD ["bash"]

FROM osrf/ros:noetic-desktop-full

#####################################################################
# FOR UBUNTU MIRRORS AMD
RUN sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list && \
    apt-get update --fix-missing && apt-get upgrade -y
# #####################################################################
# FOR ROS 1 MIRRORS
RUN apt-get update && apt-get install -y gpg lsb-release curl && \
    gpg --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    gpg --export C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 | tee /usr/share/keyrings/ros.gpg > /dev/null && \
    sh -c 'echo "deb [signed-by=/usr/share/keyrings/ros.gpg] https://mirrors.ustc.edu.cn/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
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

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    lsb-release \
    gnupg2 \
    protobuf-compiler \
    libeigen3-dev \
    libopencv-dev \
    sudo \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-mavros-msgs \ 
    python3 \
    python-catkin-tools \
    python-rosinstall-generator \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh

RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

RUN mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

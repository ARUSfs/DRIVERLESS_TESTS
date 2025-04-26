FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Create workspace early
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Install base dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    git \
    cmake \
    build-essential \
    locales \
    gnupg2 \
    python3 \
    python3-pip && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    add-apt-repository universe && \
    apt-get update

# Install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y ros-humble-desktop

# Install ROS-dependent packages
RUN apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-pcl-ros \
    ros-humble-rosbag2-storage-mcap

# Install remaining system dependencies
RUN apt-get install -y \
    nlohmann-json3-dev \
    libpcl-dev \
    libeigen3-dev \
    libspdlog-dev \
    libsuitesparse-dev \
    qtdeclarative5-dev \
    qt5-qmake \
    libqglviewer-dev-qt5

# Install Python deps
RUN pip install --no-cache-dir ruamel.yaml colcon-common-extensions

# Clone and build g2o
RUN git clone https://github.com/RainerKuemmerle/g2o.git /tmp/g2o && \
    cd /tmp/g2o && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && ldconfig && \
    rm -rf /tmp/g2o


# Source built workspace
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec bash"]

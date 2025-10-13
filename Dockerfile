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
    libqglviewer-dev-qt5 \
    g++ \
    wget \
    unzip \
    build-essential \
    libopencv-dev \
    libtclap-dev \
    libmagic-dev 


# Install Python deps
RUN pip install --no-cache-dir ruamel.yaml colcon-common-extensions

# Clone and build g2o
RUN git clone https://github.com/RainerKuemmerle/g2o.git /tmp/g2o && \
    cd /tmp/g2o && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && ldconfig && \
    rm -rf /tmp/g2o

# Install OpenCV
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip && \
    unzip opencv.zip && rm opencv.zip && \
    cd opencv-4.x && mkdir build && cd build && \
    cmake  ../opencv-4.x && cmake --build .

# Update cmake 
RUN wget https://github.com/Kitware/CMake/releases/download/v4.1.2/cmake-4.1.2.tar.gz && \
    tar -xvf cmake-4.1.2.tar.gz && cd cmake-4.1.2 && ./bootstrap --prefix=$HOME/cmake-install && \
    make -j$(nproc) && make install && export PATH=$HOME/cmake-install/bin:$PATH && \
    export CMAKE_PREFIX_PATH=$HOME/cmake-install:$CMAKE_PREFIX_PATH 

RUN git clone https://codeberg.org/CCodeRun/darknet.git /tmp/darknet && \
    cd /tmp/darknet && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && sudo dpkg -i darknet-*.deb && \
    rm -rf /tmp/darknet

RUN git clone https://github.com/stephanecharette/DarkHelp.git /tmp/DarkHelp && \
    cd /tmp/DarkHelp && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && make package && dpkg -i darkhelp*.deb \
    rm -rf /tmp/DarkHelp


# Source built workspace
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec bash"]

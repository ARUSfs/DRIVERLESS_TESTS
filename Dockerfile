#--------------------------------------------------
# @file   Dockerfile
# @author Arturo Redondo Naranjo
# @date   20-12-2025
# @brief  Dockerfile for ARUS Driverless Environment
#--------------------------------------------------

#--------------------------------------------------
# Previous installation steps
#--------------------------------------------------

# ROS2 Humble with Ubuntu 22.04
FROM osrf/ros:humble-desktop-full

# Avoids config interfaces 
SHELL ["/bin/bash", "-c"]

# Initial update and upgrade
RUN apt-get update

# Build tools and basic utilities
RUN apt-get update && apt-get install --install-recommends -y \
    # Basic utilities
    apt-utils bash-completion build-essential ccache curl file git nano net-tools tmux unzip wget \
    # Development tools
    cmake g++ g++-12 gcc-12 gdb gfortran pkg-config python3-dev python3-pip python3-venv \
    # ROS 2 and build tools
    python3-colcon-common-extensions python3-empy ros-dev-tools ros-humble-ament-cmake ros-humble-rmw-cyclonedds-cpp ros-$ROS_DISTRO-rosbag2-storage-mcap \
    # Image / video libraries
    libavcodec-dev libavformat-dev libjpeg-dev libpng-dev libswscale-dev libtiff-dev libv4l-dev libx264-dev libxvidcore-dev \
    # Linear algebra / math libraries
    libeigen3-dev liblapack-dev libopenblas-dev libopenblas64-0 libopenblas64-0-openmp libopenblas64-openmp-dev libsuitesparse-dev \
    # Other required libraries
    libgtk-3-dev libmagic-dev libopencv-dev libpcl-dev libpcap-dev libprotobuf-dev libqglviewer-dev-qt5 libspdlog-dev libtclap-dev nlohmann-json3-dev \
    # MRPT libraries and apps
    libmrpt-dev mrpt-apps \
    # Networking / system utilities
    dnsmasq hostapd usbutils wireless-tools \
    # Qt development
    qt5-qmake qtdeclarative5-dev \
    # Cleanup
    && rm -rf /var/lib/apt/lists/*

ENV CC=gcc-12
ENV CXX=g++-12
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Fuzzy find tool for commandline
RUN git clone --depth 1 https://github.com/junegunn/fzf.git /root/.fzf \
    && /root/.fzf/install --all

#--------------------------------------------------
# Dependencies
#--------------------------------------------------

# Directorys
RUN mkdir -p /root/ARUS_logs
RUN mkdir -p /root/ARUS_logs/bagfiles /root/ARUS_logs/candumps
RUN mkdir -p /root/dependencies
WORKDIR /root/dependencies

# track editor
RUN python3 -m pip install --no-cache-dir \
    "numpy==1.23.*" \
    "ruamel.yaml<0.18"

# can_interface
WORKDIR /root/dependencies
RUN curl -L "https://www.kvaser.com/download/?utm_source=software&utm_ean=7330130980754&utm_status=latest" \
    -o linuxcan.tar.gz
RUN tar -xvzf linuxcan.tar.gz
WORKDIR /root/dependencies/linuxcan/canlib
RUN make && make install
WORKDIR /root/dependencies/linuxcan/linlib
RUN make && make install

# PCART
WORKDIR /root/dependencies
RUN curl -L "https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.55/mcap-linux-amd64" \
    -o mcap-linux-amd64
RUN chmod +x mcap-linux-amd64
RUN mv mcap-linux-amd64 /usr/local/bin/mcap

# Camera
WORKDIR /root/dependencies
RUN wget -O opencv-4.x.zip https://github.com/opencv/opencv/archive/4.x.zip 
RUN unzip opencv-4.x.zip 
RUN mkdir -p /root/dependencies/opencv-4.x/build
WORKDIR /root/dependencies/opencv-4.x/build
RUN cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    && make -j$(nproc) \
    && make install \
    && ldconfig

# CMake
WORKDIR /root/dependencies
RUN wget https://github.com/Kitware/CMake/releases/download/v4.1.2/cmake-4.1.2.tar.gz \
    && tar -xvf cmake-4.1.2.tar.gz
WORKDIR /root/dependencies/cmake-4.1.2
RUN ./bootstrap --prefix=$HOME/cmake-install \
    && make -j$(nproc) \
    && make install
ENV PATH="/root/cmake-install/bin:${PATH}"
ENV CMAKE_PREFIX_PATH="/root/cmake-install:${CMAKE_PREFIX_PATH:-}"

# Darknet
WORKDIR /root/dependencies
RUN git clone https://codeberg.org/CCodeRun/darknet.git
RUN mkdir -p /root/dependencies/darknet/build
WORKDIR /root/dependencies/darknet/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j$(nproc) package \
    && dpkg -i darknet*.deb \
    && ldconfig

# Darkhelp
WORKDIR /root/dependencies
RUN git clone https://github.com/stephanecharette/DarkHelp.git
RUN mkdir -p /root/dependencies/DarkHelp/build
WORKDIR /root/dependencies/DarkHelp/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make \
    && make -j$(nproc) package \
    && dpkg -i darkhelp*.deb \
    && ldconfig

# g2o
WORKDIR /root/dependencies
RUN git clone https://github.com/RainerKuemmerle/g2o 
RUN mkdir -p /root/dependencies/g2o/build
WORKDIR /root/dependencies/g2o/build
RUN cmake .. \
    && make \
    && make install \
    && ldconfig

# Driverless Code (This will be executed individually once the container is built)
# RUN git clone git@github.com:ARUSfs/DRIVERLESS2 
# RUN git clone git@github.com:ARUSfs/ARUSSim
# git clone -b fix/emx/decoder --single-branch git@github.com:ARUSfs/emx_sdk.git
# git clone git@github.com:ARUSfs/rslidar_msg.git

#--------------------------------------------------
# Post installation steps
#--------------------------------------------------
ENV PROJECT_ROOT=/root/ws
ENV WORKSPACE=${PROJECT_ROOT}/src

# Initialize rosdep (for installing ROS package dependencies)
RUN rosdep init 2>/dev/null || true && rosdep update || true

# Source environment in every new bash session
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc

# Make alias available for all users and all future shells
RUN echo "alias arussim='ros2 launch arussim arussim_launch.py'" >> /root/.bashrc

# Enable colorful bash prompts 
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc

# Set working directory and shell
WORKDIR ${WORKSPACE}
CMD ["bash"]
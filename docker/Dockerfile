# ROS 2 Humble + gz (Ignition) + PX4 SITL + MAVROS + MAVSDK + OpenCV + Python tooling
ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ---- Base tools & locales ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata locales sudo bash-completion ca-certificates curl wget git gnupg2 lsb-release \
    build-essential cmake ninja-build ccache gdb valgrind \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8 && update-ca-certificates
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ---- ROS 2 Humble apt repo (keyring method) ----
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'

# ---- ROS 2 Humble desktop + core ROS tools (after repo is added) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep python3-argcomplete python3-vcstool python3-colcon-common-extensions \
    ros-humble-desktop \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
# rosdep baseline (ignore if already inited)
RUN rosdep init || true

# ---- Gazebo Sim (gz) + ROS bridge ----
RUN apt-get update && apt-get install -y --no-install-recommends lsb-release wget gnupg && \
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gz-stable.list' && \
    apt-get update && \
    (apt-get install -y --no-install-recommends gz-harmonic || apt-get install -y --no-install-recommends gz-garden) && \
    rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Resource paths for your mounted worlds/models
ENV GZ_SIM_RESOURCE_PATH=/ws/sim/gazebo/models:/ws/sim/gazebo/worlds:${GZ_SIM_RESOURCE_PATH}
ENV IGN_GAZEBO_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}

# ---- MAVROS + GeographicLib datasets ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-mavros ros-humble-mavros-extras geographiclib-tools \
    && rm -rf /var/lib/apt/lists/*
RUN /bin/bash -lc "source /opt/ros/humble/setup.bash && /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh"

# ---- PX4 SITL deps + (image-bundled) PX4 clone (can be overridden by bind mount) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    git zip unzip tar xz-utils \
    python3-jinja2 python3-numpy python3-yaml \
    xterm xsltproc libxml2-utils \
    && rm -rf /var/lib/apt/lists/*
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot
RUN /bin/bash -lc "cd /opt/PX4-Autopilot && Tools/setup/ubuntu.sh --no-nuttx"

# ---- MAVSDK (C++) from source (latest stable v3.x) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    libprotobuf-dev protobuf-compiler libcurl4-openssl-dev libtinyxml2-dev \
    libjsoncpp-dev libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# ---- ACADOS C++ Dependencies ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake git libblas-dev liblapack-dev libopenblas-dev libeigen3-dev \
    libboost-all-dev gfortran \
    && rm -rf /var/lib/apt/lists/*

# ---- Clone and Build ACADOS ----
RUN git clone --recursive https://github.com/acados/acados.git /opt/acados && \
    cd /opt/acados && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    sudo make install

# Clone the official MAVSDK repo and build the latest stable release (v3.x+)
RUN git clone --recursive https://github.com/mavlink/MAVSDK.git /opt/MAVSDK && \
    cd /opt/MAVSDK && \
    git checkout v3.10.2 && \       
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON && \
    cmake --build build -j$(nproc) && \
    cmake --install build && \
    ldconfig

# ---- Useful C++/CV/Serial/Math + Python tooling ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev python3-opencv \
    libeigen3-dev libyaml-cpp-dev libboost-all-dev \
    libserial-dev libusb-1.0-0-dev libudev-dev libzmq3-dev \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir \
    numpy matplotlib pandas scipy tqdm pyyaml pyserial ipython pre-commit mavsdk

# ---- udev rules for FMU/USB bridges ----
RUN install -d -m 0755 /etc/udev/rules.d && \
    cat <<'EOF' >/etc/udev/rules.d/99-robotics-serial.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", MODE="0666", GROUP="dialout"   # STM32 VCP
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0666", GROUP="dialout"   # CP210x (Kakute H7, etc.)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0666", GROUP="dialout"   # FTDI
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666", GROUP="dialout"   # CH340/CH341
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", MODE="0666", GROUP="dialout"   # Prolific PL2303
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", MODE="0666", GROUP="dialout"   # Arduino
EOF
# udev isn't active during image build; this is harmless but kept for runtime containers.
RUN udevadm control --reload-rules || true && udevadm trigger || true

# ---- ROS 2 workspace (/ws) ----
ENV ROS_DISTRO=humble
WORKDIR /ws
RUN mkdir -p /ws/src
ENV COLCON_DEFAULTS_FILE=/ws/colcon.defaults
RUN bash -lc 'printf "{\\n  \\"build-base\\": \\"build\\",\\n  \\"install-base\\": \\"install\\",\\n  \\"test-result-base\\": \\"test_results\\"\\n}\\n" > ${COLCON_DEFAULTS_FILE}'

# Auto-source ROS + overlay
RUN bash -lc 'echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
              echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /etc/bash.bashrc && \
              echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /etc/bash.bashrc'

CMD ["/bin/bash"]

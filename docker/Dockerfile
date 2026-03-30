# =========================================================
# STAGE 1: base
# Core ROS 2, system tools, and user configuration
# =========================================================
ARG BASE_IMAGE=ros:humble-desktop-full
FROM ${BASE_IMAGE} AS base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Locales & Base Tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata locales sudo bash-completion ca-certificates curl wget git gnupg2 lsb-release \
    build-essential cmake ninja-build ccache gdb valgrind pkg-config \
    python3-pip python3-venv python3-dev swig \
    && rm -rf /var/lib/apt/lists/*
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/
RUN locale-gen en_US.UTF-8 && update-ca-certificates
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ROS 2 Environment
ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages

# ROS 2 Tooling & MAVROS
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep python3-argcomplete python3-vcstool python3-colcon-common-extensions \
    ros-humble-image-transport ros-humble-cv-bridge ros-humble-rmw-cyclonedds-cpp \
    ros-humble-mavros ros-humble-mavros-extras geographiclib-tools \
    ros-humble-nav2-msgs ros-humble-plotjuggler-ros \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly \
    libopencv-dev python3-opencv \
    libvulkan1 vulkan-tools clang clang-format \
    && rm -rf /var/lib/apt/lists/*
RUN rosdep init || true
RUN /bin/bash -lc "source /opt/ros/humble/setup.bash && /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh"

# Gazebo (GZ Harmonic)
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gz-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    gz-harmonic libgz-sim8-dev libgz-plugin2-dev libgz-msgs10-dev \
    libgz-transport13-dev libgz-math7-dev libgz-common5-dev libsdformat14-dev \
    ros-humble-ros-gzharmonic ros-humble-ros-gzharmonic-bridge ros-humble-ros-gzharmonic-image \
    ros-humble-ros-gzharmonic-interfaces ros-humble-ros-gzharmonic-sim \
    && rm -rf /var/lib/apt/lists/*

# udev rules for FMU/USB bridges
RUN install -d -m 0755 /etc/udev/rules.d && \
    cat <<'EOF' >/etc/udev/rules.d/99-robotics-serial.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0666", GROUP="dialout"
EOF

# User Setup
ARG USERNAME=drone_dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && chmod 0440 /etc/sudoers.d/$USERNAME

ENV WORKSPACE=/home/${USERNAME}/ros_ws
WORKDIR /home/${USERNAME}

# Set up autocompletion for user
RUN  echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc

# PX4 build dependencies — sourced from the PX4 submodule itself
COPY px4/PX4-Autopilot/Tools/setup/requirements.txt /tmp/px4_requirements.txt
RUN uv pip install --system --no-cache setuptools==58.2.0 && \
    uv pip install --system --no-cache -r /tmp/px4_requirements.txt && rm /tmp/px4_requirements.txt

# =========================================================
# STAGE 2: shared-perception-controls-deps
# Shared dependencies for perception, controls, and docs
# =========================================================
FROM base AS shared-perception-controls-deps

# Perception system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev python3-opencv \
    libeigen3-dev libyaml-cpp-dev libboost-all-dev \
    libceres-dev libsuitesparse-dev libgoogle-glog-dev libgflags-dev \
    ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/*

# Shared Python deps
RUN uv pip install --system --no-cache numpy==1.24.4 scipy==1.10.1

# =========================================================
# STAGE 3: docs (standalone)
# Minimal, disjoint stage for MkDocs only
# =========================================================
FROM python:3.10-slim AS docs
RUN pip install --no-cache-dir "mkdocs<2.0" "mkdocs-material==9.5.34" "pymdown-extensions<11.0"
EXPOSE 8000
COPY docs /docs
WORKDIR /docs
CMD ["mkdocs", "serve", "-a", "0.0.0.0:8000"]

# =========================================================
# STAGE 4: perception
# Perception-only build
# =========================================================
FROM shared-perception-controls-deps AS perception
USER drone_dev
COPY --chown=drone_dev:drone_dev src/perception ${WORKSPACE}/src/perception
COPY --chown=drone_dev:drone_dev src/ros2-vicon-receiver ${WORKSPACE}/src/ros2-vicon-receiver
COPY --chown=drone_dev:drone_dev src/utils ${WORKSPACE}/src/utils
COPY --chown=drone_dev:drone_dev launch ${WORKSPACE}/launch
WORKDIR ${WORKSPACE}
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to aruco_pose_estimation ov_msckf vicon_receiver gazebo_ros_bridges

# =========================================================
# STAGE 5: shared-mpc-rl-deps
# Dependencies for MPC, MAVSDK, ACADOS, and RL
# =========================================================
FROM shared-perception-controls-deps AS shared-mpc-rl-deps

# MAVSDK C++ Build
RUN apt-get update && apt-get install -y --no-install-recommends \
    libprotobuf-dev protobuf-compiler libcurl4-openssl-dev libtinyxml2-dev \
    libjsoncpp-dev libssl-dev && rm -rf /var/lib/apt/lists/*
RUN git clone --recursive https://github.com/mavlink/MAVSDK.git /opt/MAVSDK && \
    cd /opt/MAVSDK && git fetch --tags && git checkout v3.10.2 && \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON && \
    cmake --build build -j$(nproc) && cmake --install build && ldconfig

# ACADOS C++ Build
RUN apt-get update && apt-get install -y --no-install-recommends \
    libblas-dev liblapack-dev libopenblas-dev liblapacke-dev gfortran && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/acados/acados.git /opt/acados && \
    cd /opt/acados && git submodule update --recursive --init && \
    mkdir -p build && cd build && cmake -DACADOS_SHARED_LIBS=ON -DACADOS_WITH_QPOASES=ON .. && \
    make install -j$(nproc)

# Tera Renderer
RUN mkdir -p /opt/acados/bin && \
    wget -q https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64 -O /opt/acados/bin/t_renderer && \
    chmod +x /opt/acados/bin/t_renderer
ENV TERA_PATH=/opt/acados/bin/t_renderer
ENV LD_LIBRARY_PATH=/opt/acados/lib:${LD_LIBRARY_PATH}
ENV ACADOS_SOURCE_DIR=/opt/acados

# Python deps (MPC + RL)
RUN uv pip install --system --no-cache \
    setuptools==58.2.0 numpy==1.24.4 scipy==1.10.1 mavsdk casadi cython && \
    uv pip install --system --no-cache -e /opt/acados/interfaces/acados_template
RUN uv pip install --system --no-cache torch torchvision torchaudio stable-baselines3 shimmy gymnasium

# =========================================================
# STAGE 6: mpc
# MPC-only build
# =========================================================
FROM shared-mpc-rl-deps AS mpc
USER drone_dev
COPY --chown=drone_dev:drone_dev src/mpc ${WORKSPACE}/src/mpc
COPY --chown=drone_dev:drone_dev src/drone_control ${WORKSPACE}/src/drone_control
COPY --chown=drone_dev:drone_dev src/my_mavsdk ${WORKSPACE}/src/my_mavsdk
COPY --chown=drone_dev:drone_dev launch ${WORKSPACE}/launch
WORKDIR ${WORKSPACE}
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to drone_control my_mavsdk mpc

# =========================================================
# STAGE 7: rl
# RL build (includes MPC builds)
# =========================================================
FROM shared-mpc-rl-deps AS rl
USER drone_dev
# Copy MPC builds to avoid redundant compilation
COPY --from=mpc ${WORKSPACE}/install ${WORKSPACE}/install
COPY --from=mpc ${WORKSPACE}/build ${WORKSPACE}/build
COPY --chown=drone_dev:drone_dev src/mpc ${WORKSPACE}/src/mpc
COPY --chown=drone_dev:drone_dev src/drone_control ${WORKSPACE}/src/drone_control
COPY --chown=drone_dev:drone_dev src/my_mavsdk ${WORKSPACE}/src/my_mavsdk
COPY --chown=drone_dev:drone_dev launch ${WORKSPACE}/launch
WORKDIR ${WORKSPACE}

# =========================================================
# STAGE 8: dev (full build)
# The complete package — merges all targets
# =========================================================
FROM rl AS dev
USER root
# Copy perception builds from the perception stage
COPY --from=perception ${WORKSPACE}/install ${WORKSPACE}/install
COPY --from=perception ${WORKSPACE}/build ${WORKSPACE}/build
# Ensure perception source is present
COPY --chown=drone_dev:drone_dev src/perception ${WORKSPACE}/src/perception
COPY --chown=drone_dev:drone_dev src/ros2-vicon-receiver ${WORKSPACE}/src/ros2-vicon-receiver
COPY --chown=drone_dev:drone_dev src/utils ${WORKSPACE}/src/utils
COPY --chown=drone_dev:drone_dev launch ${WORKSPACE}/launch
# Add docs
COPY --chown=drone_dev:drone_dev docs /home/drone_dev/docs
RUN uv pip install --system --no-cache "mkdocs<2.0" "mkdocs-material==9.5.34" "pymdown-extensions<11.0"

USER drone_dev
WORKDIR ${WORKSPACE}
RUN echo "source ${WORKSPACE}/install/setup.bash" >> /home/drone_dev/.bashrc
CMD ["/bin/bash"]

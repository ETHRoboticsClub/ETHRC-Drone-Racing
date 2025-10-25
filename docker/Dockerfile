ARG BASE_IMAGE=ros:humble-desktop-full
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ---- Base tools & locales ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata locales sudo bash-completion ca-certificates curl wget git gnupg2 lsb-release \
    build-essential cmake ninja-build ccache gdb valgrind pkg-config \
    python3-pip python3-venv python3-dev swig \
    && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8 && update-ca-certificates
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Ensure ROS 2 apt repository is configured so additional ros-humble packages are available
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg2 lsb-release ca-certificates && rm -rf /var/lib/apt/lists/*
RUN if ! grep -R "packages.ros.org" /etc/apt/sources.list.d >/dev/null 2>&1 ; then \
            mkdir -p /etc/apt/keyrings && \
            curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
            sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'; \
        else \
            echo "ROS apt source already present, skipping add"; \
        fi

# The base image already provides many ROS 2 packages; install additional ROS tooling and bridge packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep python3-argcomplete python3-vcstool python3-colcon-common-extensions \
    ros-humble-image-transport ros-humble-cv-bridge ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
RUN rosdep init || true

# Install Ignition / Gazebo (ros_gz bridge and common rendering backends).
# We use OSRF's packages to get ignition/gz packages compatible with the OS.
RUN apt-get update && apt-get install -y --no-install-recommends lsb-release wget gnupg && \
    wget -qO /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg https://packages.osrfoundation.org/gazebo.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gz-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-gz gz-transport-tools gz-sim7 gz-tools || true && \
    rm -rf /var/lib/apt/lists/*

# Resource paths for your mounted worlds/models
# ENV GZ_SIM_RESOURCE_PATH=/ws/sim/gazebo/models:/ws/sim/gazebo/worlds:${GZ_SIM_RESOURCE_PATH}
# ENV IGN_GAZEBO_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}

# ---- MAVROS + GeographicLib datasets ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-mavros ros-humble-mavros-extras geographiclib-tools \
    && rm -rf /var/lib/apt/lists/*
RUN /bin/bash -lc "source /opt/ros/humble/setup.bash && /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh"

# PX4 is cloned and also submodule in repo -> conflict.
# ---- PX4 SITL deps + (image-bundled) PX4 clone (can be overridden by bind mount) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    git zip unzip tar xz-utils \
    python3-jinja2 python3-numpy python3-yaml \
    xterm xsltproc libxml2-utils \
    && rm -rf /var/lib/apt/lists/*
# RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot
# RUN /bin/bash -lc "cd /opt/PX4-Autopilot && Tools/setup/ubuntu.sh --no-nuttx"
RUN pip3 install --no-cache-dir \
    kconfiglib jsonschema pyros-genmsg

# ---- MAVSDK (C++) from source (latest stable v3.x) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    libprotobuf-dev protobuf-compiler libcurl4-openssl-dev libtinyxml2-dev \
    libjsoncpp-dev libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# ---- ACADOS C++ Dependencies ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake git libblas-dev liblapack-dev libopenblas-dev libeigen3-dev \
    libboost-all-dev gfortran python3-numpy python3-pip python3-setuptools \
    liblapacke-dev liblapack-dev libblas-dev \
    && rm -rf /var/lib/apt/lists/*

# Install CasADi (binary wheel) for Python — acados Python interface depends on this.
RUN python3 -m pip install --no-cache-dir casadi

# Clone and build acados (with python interface). We clone recursively to fetch hpipm/blasfeo.
RUN git clone --recursive https://github.com/acados/acados.git /opt/acados && \
    cd /opt/acados && \
    mkdir -p build && cd build && \
    cmake .. -DACADOS_SHARED_LIBS=ON && \
    make -j$(nproc) && \
    make install

# Install acados python interface (editable install so user code can import acados)
# The python interface lives under interfaces/acados_template in upstream acados
RUN python3 -m pip install --no-cache-dir cython || true
RUN python3 -m pip install --no-cache-dir -e /opt/acados/interfaces/acados_template

# ---- Tera renderer (Rust) ----
# Build and install the tera_renderer tool used by acados templating.
RUN apt-get update && apt-get install -y --no-install-recommends \
    rustc cargo libx11-dev libxrandr-dev libxcursor-dev libxinerama-dev \
    libgl1-mesa-dev libegl1-mesa-dev libvulkan-dev && rm -rf /var/lib/apt/lists/*
RUN git clone --depth 1 https://github.com/acados/tera_renderer.git /opt/tera_renderer && \
    cd /opt/tera_renderer && \
    cargo build --verbose --release && \
    if [ -f target/release/tera_renderer ]; then install -m 0755 target/release/tera_renderer /usr/local/bin/tera_renderer; fi || true

# Clone the official MAVSDK repo and build the latest stable release (v3.x+)
RUN git clone --recursive https://github.com/mavlink/MAVSDK.git /opt/MAVSDK && \
    cd /opt/MAVSDK && \
    git fetch --tags && git checkout v3.10.2 && \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON && \
    cmake --build build -j$(nproc) && \
    cmake --install build && \
    ldconfig || true

# ---- Useful C++/CV/Serial/Math + Python tooling ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev python3-opencv \
    libeigen3-dev libyaml-cpp-dev libboost-all-dev \
    libserial-dev libusb-1.0-0-dev libudev-dev libzmq3-dev \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir \
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

# Add custom user
ARG USERNAME=drone_dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV ROS_DISTRO=humble

# Find and replace any user with matching UID
RUN set -eux; \
    existing_user=$(getent passwd "$USER_UID" | cut -d: -f1 || true); \
    if [ -n "$existing_user" ]; then \
        usermod -l $USERNAME -d /home/$USERNAME -m $existing_user && \
        groupmod -n $USERNAME $existing_user && \
        echo "Renamed $existing_user to $USERNAME"; \
    else \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
        echo "Created new user $USERNAME"; \
    fi
# Ensure necessary directories and permissions
RUN mkdir -p /home/$USERNAME /run/user/$USER_UID && \
    chown -R $USER_UID:$USER_GID /home/$USERNAME /run/user/$USER_UID

# Add sudo support for the non-root user
RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && touch /home/$USERNAME/.sudo_as_admin_successful

# Set up autocompletion for user
RUN  echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /usr/share/vcs2l-completion/vcs.bash ]; then source /usr/share/vcs2l-completion/vcs.bash; fi" >> /home/$USERNAME/.bashrc \
  # Set up auto-source of workspace for user
  && echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Set up python symlinks for vscode paths
RUN PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info[0]}.{sys.version_info[1]}')") \
 && tgt="/opt/ros/${ROS_DISTRO}/lib/python${PYTHON_VERSION}/site-packages" \
 && [ -d "$tgt" ] && ln -s "$tgt" "/opt/ros/${ROS_DISTRO}/lib/ros_site_packages"

USER $USERNAME


WORKDIR /home/$USERNAME
# ---- ROS 2 workspace (/ros_ws) ----
RUN mkdir -p /home/$USERNAME/ros_ws/src
RUN mkdir -p /home/$USERNAME/ros_ws/install
RUN mkdir -p /home/$USERNAME/ros_ws/build
RUN mkdir -p /home/$USERNAME/ros_ws/log
# ENV COLCON_DEFAULTS_FILE=/home/$USERNAME/ros_ws/colcon.defaults
# RUN bash -lc 'printf "{\\n  \\"build-base\\": \\"build\\",\\n  \\"install-base\\": \\"install\\",\\n  \\"test-result-base\\": \\"test_results\\"\\n}\\n" > ${COLCON_DEFAULTS_FILE}'

# copy src and build
COPY --chown=$USERNAME:$USERNAME src /home/$USERNAME/ros_ws/src 
WORKDIR /home/$USERNAME/ros_ws
# Build workspace (non-fatal if user mounts their own workspace at runtime)
RUN bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build \
    --symlink-install \
    --cmake-args "-DCMAKE_BUILD_TYPE=RelWithDebInfo"' || true

# Auto-source ROS + overlay
RUN bash -lc 'echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc && \
              echo "[ -f /home/${USERNAME}/ros_ws/install/setup.bash ] && source /home/${USERNAME}/ros_ws/install/setup.bash" >> /home/$USERNAME/.bashrc && \
              echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc'

CMD ["/bin/bash"]

### Notes:
# - CasADi is installed via pip (binary wheel). If you need a specific casadi C library built from source,
#   replace the pip step with a source build and set CASADI_DIR accordingly.
# - "Tera renderer" was requested: please provide the exact repository or package name for the renderer
#   you want built from source. I left a placeholder here so we don't guess the wrong project.
#   Once you confirm, I will add the clone/build steps (it can be heavy and may need extra system deps).

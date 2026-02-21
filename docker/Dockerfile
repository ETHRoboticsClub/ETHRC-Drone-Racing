ARG BASE_IMAGE=ros:humble-desktop-full
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV MKDOCS_PORT=8000

SHELL ["/bin/bash", "-c"]

# ---- Base tools & locales ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata locales sudo bash-completion ca-certificates curl wget git gnupg2 lsb-release \
    build-essential cmake ninja-build ccache gdb valgrind pkg-config \
    python3-pip python3-venv python3-dev swig \
    ros-humble-robot-localization \
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

# ---- MAVROS + GeographicLib datasets ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-mavros ros-humble-mavros-extras geographiclib-tools \
    && rm -rf /var/lib/apt/lists/*
RUN /bin/bash -lc "source /opt/ros/humble/setup.bash && /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh"

# ---- MAVSDK (C++) from source (latest stable v3.x) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    libprotobuf-dev protobuf-compiler libcurl4-openssl-dev libtinyxml2-dev \
    libjsoncpp-dev libssl-dev \
    && rm -rf /var/lib/apt/lists/*

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
    libceres-dev libsuitesparse-dev libgoogle-glog-dev libgflags-dev \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    && rm -rf /var/lib/apt/lists/*

# Dependencies for OpenVINS
RUN apt-get update && apt-get install -y --no-install-recommends \
    libceres-dev libatlas-base-dev libsuitesparse-dev libgoogle-glog-dev libgflags-dev


# Ensure pip tooling is up-to-date and install Python packages pinned to
# versions compatible with system/compiled extensions. In particular we
# pin NumPy < 2 to avoid ABI conflicts with SciPy and other C extensions
# which were built against NumPy 1.x.
RUN python3 -m pip install --upgrade pip packaging wheel
RUN python3 -m pip install --upgrade pip setuptools==79.0.1
RUN python3 -m pip install --no-cache-dir \
    numpy==1.24.4 scipy==1.10.1 matplotlib pandas tqdm pyyaml pyserial ipython pre-commit mavsdk
# ---- ACADOS C++ Dependencies ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake git libblas-dev liblapack-dev libopenblas-dev libeigen3-dev \
    libboost-all-dev gfortran python3-pip python3-setuptools \
    liblapacke-dev liblapack-dev libblas-dev \
    && rm -rf /var/lib/apt/lists/*

# Install CasADi (binary wheel) for Python — acados Python interface depends on this.
RUN python3 -m pip install --no-cache-dir casadi

# Clone and build acados (with python interface). We clone recursively to fetch hpipm/blasfeo.
RUN git clone https://github.com/acados/acados.git /opt/acados && \
    cd /opt/acados && \
    git submodule update --recursive --init && \
    mkdir -p build && cd build && \
    cmake -DACADOS_SHARED_LIBS=ON -DACADOS_WITH_QPOASES=ON .. && \
    make install -j$(nproc)

# Install acados python interface (editable install so user code can import acados)
# The python interface lives under interfaces/acados_template in upstream acados
RUN python3 -m pip install --no-cache-dir cython
RUN python3 -m pip install --no-cache-dir -e /opt/acados/interfaces/acados_template

# Make acados available at the user's home path so runtime builds that expect
# /home/<user>/acados will find the headers/libs. Also expose lib path and
# tera renderer location to the environment.
ENV LD_LIBRARY_PATH=/opt/acados/lib
ENV ACADOS_SOURCE_DIR=/opt/acados

# ---- Tera renderer (Rust) ----
# Build and install the tera_renderer tool used by acados templating.
RUN mkdir -p /opt/acados/bin
RUN cd /opt/acados/bin 
RUN wget -q https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64
RUN mv t_renderer-v0.2.0-linux-amd64 t_renderer 
RUN chmod +x t_renderer
ENV TERA_PATH=/t_renderer


# ===========================================================
# Gazebo (GZ Harmonic) + ROS 2 Bridge + PX4 SITL deps
# ===========================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    lsb-release wget gnupg curl \
    dmidecode bc \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav libgstreamer-plugins-base1.0-dev \
    libeigen3-dev libopencv-dev libxml2-utils \
    libimage-exiftool-perl pkg-config protobuf-compiler \
    libunwind-dev \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gz-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    gz-harmonic \
    libgz-sim8-dev \
    libgz-plugin2-dev \
    libgz-msgs10-dev \
    libgz-transport13-dev \
    libgz-math7-dev \
    libgz-common5-dev \
    libsdformat14-dev \
    && rm -rf /var/lib/apt/lists/*

# ROS2 <--> Gazebo Harmonic bridge
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-humble-ros-gzharmonic \
    ros-humble-ros-gzharmonic-bridge \
    ros-humble-ros-gzharmonic-image \
    ros-humble-ros-gzharmonic-interfaces \
    ros-humble-ros-gzharmonic-sim \
    && rm -rf /var/lib/apt/lists/*

# ===========================================================
# PX4 SITL Dependencies (clean and correct)
# ===========================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    git zip unzip tar xz-utils \
    python3-dev python3-pip python3-rospkg python3-jinja2 python3-yaml \
    xsltproc libxml2-utils xterm \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libvulkan1 vulkan-tools \
    libopencv-dev \
    clang clang-format ccache \
    && rm -rf /var/lib/apt/lists/*

# PX4-specific Python tools
RUN pip3 install --no-cache-dir \
    pyulog pyserial empy toml jinja2 pandas \
    jsonschema pyros-genmsg kconfiglib \
    numpy matplotlib pandas scipy tqdm pyyaml pyserial ipython pre-commit mavsdk \
    "mkdocs<2.0" "mkdocs-material==9.5.34" "pymdown-extensions<11.0"

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


# --- Fix NumPy / SciPy compatibility ---
RUN pip install --upgrade --force-reinstall \
    numpy==1.24.4 scipy==1.10.1


# usefull ros packages:
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-nav2-msgs \
    ros-humble-plotjuggler-ros \
    && rm -rf /var/lib/apt/lists/*
# Add custom user
ARG USERNAME=drone_dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV ROS_DISTRO=humble
ENV WORKSPACE=/home/${USERNAME}/ros_ws

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
    && echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc

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

# copy src and launch files into workspace [Please add those directories in docker-compose.yaml if they can change - Dime]
COPY --chown=$USERNAME:$USERNAME src /home/$USERNAME/ros_ws/src 
COPY --chown=$USERNAME:$USERNAME launch /home/$USERNAME/ros_ws/launch
# COPY --chown=$USERNAME:$USERNAME docs /home/$USERNAME/docs

WORKDIR /home/$USERNAME/ros_ws
# Build workspace (non-fatal if user mounts their own workspace at runtime)
RUN bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build \
    --symlink-install \
    --cmake-args "-DCMAKE_BUILD_TYPE=RelWithDebInfo"' || \
    (echo "======================================================================" && \
    echo "WARNING: ROS 2 workspace build failed!" && \
    echo "The image will still be created to allow development with host mounts." && \
    echo "Check the errors above if you intend to run the baked-in code." && \
    echo "======================================================================")


ENV USERNAME=${USERNAME}
CMD ["/bin/bash", "-c", "mkdocs serve -f /home/${USERNAME}/docs/mkdocs.yml -a 0.0.0.0:${MKDOCS_PORT} & exec /bin/bash"]

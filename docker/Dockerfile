ARG BASE_IMAGE=ros:humble-ros-base

ARG PERCEPTION_CONTROLS_BASE=916780037007.dkr.ecr.us-east-2.amazonaws.com/aeronautics/droneracing:shared-perception-controls-deps-latest
# PERCEPTION_CONTROLS_BASE originates from docker/untracked/Dockerfile.base

# BUILD_JOBS caps compile parallelism for low-RAM machines (e.g. BUILD_JOBS=4).
# Leave it EMPTY (the default) to keep the original behaviour: -j$(nproc) and
# colcon's default worker count. Must be re-declared in every stage that uses it.
ARG BUILD_JOBS=

# =========================================================
# STAGE 1: docs (standalone)
# Minimal, disjoint stage for MkDocs only
# =========================================================
FROM python:3.10-slim AS docs
RUN pip install --no-cache-dir "mkdocs<2.0" "mkdocs-material==9.5.34" "pymdown-extensions<11.0"
EXPOSE 8000
COPY docs /docs
WORKDIR /docs
CMD ["mkdocs", "serve", "-a", "0.0.0.0:8000"]

# =========================================================
# STAGE 2 and 3 are defined in docker/untracked/Dockerfile.base as STAGE 1 and 2
# =========================================================

# =========================================================
# STAGE 4: perception
# Perception-only build
# =========================================================
FROM ${PERCEPTION_CONTROLS_BASE} AS perception
ARG BUILD_JOBS
USER drone_dev

COPY --chown=drone_dev:drone_dev src/perception ${WORKSPACE}/src/perception
COPY --chown=drone_dev:drone_dev src/msp ${WORKSPACE}/src/msp
COPY --chown=drone_dev:drone_dev src/ros2-vicon-receiver ${WORKSPACE}/src/ros2-vicon-receiver
COPY --chown=drone_dev:drone_dev src/utils ${WORKSPACE}/src/utils
COPY --chown=drone_dev:drone_dev src/h264_image_transport ${WORKSPACE}/src/h264_image_transport
COPY --chown=drone_dev:drone_dev src/jetson_h264_publisher ${WORKSPACE}/src/jetson_h264_publisher
COPY --chown=drone_dev:drone_dev src/msp ${WORKSPACE}/src/msp
COPY --chown=drone_dev:drone_dev launch ${WORKSPACE}/launch
WORKDIR ${WORKSPACE}
USER root
RUN apt-get update && apt-get install -y libserialport-dev && rm -rf /var/lib/apt/lists/*
USER drone_dev
RUN ARCH=$(dpkg --print-architecture); \
    PKGS="msp ov_msckf vicon_receiver betaflight_blackbox_driver ekf gate_detection gazebo_ground_thruth pointcloud_gt h264_image_transport aigp_camera_driver aigp_imu_driver betaflight_mavlink_bridge"; \
    if [ "$ARCH" != "arm64" ]; then PKGS="$PKGS gazebo_ros_bridges"; else PKGS="$PKGS jetson_h264_publisher"; fi; \
    if [ -n "$BUILD_JOBS" ]; then export MAKEFLAGS="-j$BUILD_JOBS"; fi; \
    source /opt/ros/humble/setup.bash && colcon build --symlink-install --parallel-workers 2 --packages-up-to $PKGS

# =========================================================
# STAGE 5: shared-mpc-rl-deps
# Dependencies for MPC, MAVSDK, ACADOS, and RL
# =========================================================
FROM ${PERCEPTION_CONTROLS_BASE} AS shared-mpc-rl-deps
ARG BUILD_JOBS

# MAVSDK C++ Build
RUN apt-get update && apt-get install -y --no-install-recommends \
    libprotobuf-dev protobuf-compiler libcurl4-openssl-dev libtinyxml2-dev \
    libjsoncpp-dev libssl-dev libserialport-dev && rm -rf /var/lib/apt/lists/*
RUN git clone --recursive https://github.com/mavlink/MAVSDK.git /opt/MAVSDK && \
    cd /opt/MAVSDK && git fetch --tags && git checkout v3.10.2 && \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON && \
    cmake --build build -j${BUILD_JOBS:-$(nproc)} && cmake --install build && ldconfig

# ACADOS C++ Build
RUN apt-get update && apt-get install -y --no-install-recommends \
    libblas-dev liblapack-dev libopenblas-dev liblapacke-dev gfortran && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/acados/acados.git /opt/acados && \
    cd /opt/acados && git submodule update --recursive --init && \
    mkdir -p build && cd build && cmake -DACADOS_SHARED_LIBS=ON -DACADOS_WITH_QPOASES=ON .. && \
    make install -j${BUILD_JOBS:-$(nproc)}

# Tera Renderer
RUN mkdir -p /opt/acados/bin && \
    ARCH=$(dpkg --print-architecture) && \
    if [ "$ARCH" = "arm64" ]; then \
        wget -q https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-arm64 -O /opt/acados/bin/t_renderer; \
    else \
        wget -q https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64 -O /opt/acados/bin/t_renderer; \
    fi && \
    chmod +x /opt/acados/bin/t_renderer
ENV TERA_PATH=/opt/acados/bin/t_renderer
ENV LD_LIBRARY_PATH=/opt/acados/lib:${LD_LIBRARY_PATH}
ENV ACADOS_SOURCE_DIR=/opt/acados

# Python deps (MPC + RL)
RUN uv pip install --system --no-cache \
    setuptools==58.2.0 numpy==1.24.4 scipy==1.10.1 mavsdk casadi cython pyserial && \
    uv pip install --system --no-cache -e /opt/acados/interfaces/acados_template
# torch is pinned: newer releases (>=2.12) require setuptools>=77, which silently
# overrides the setuptools==58.2.0 pin above; setuptools>=80 dropped
# `setup.py develop --editable`, breaking `colcon build --symlink-install`.
RUN uv pip install --system --no-cache torch==2.11.0 torchvision==0.26.0 torchaudio==2.11.0 stable-baselines3 shimmy gymnasium && \
    python3 -c 'import setuptools, sys; v = setuptools.__version__; sys.exit(0 if v == "58.2.0" else f"setuptools pin lost (got {v}) -- colcon --symlink-install will fail")'

# =========================================================
# STAGE 6: mpc
# MPC-only build
# =========================================================
FROM shared-mpc-rl-deps AS mpc
ARG BUILD_JOBS
USER drone_dev
COPY --chown=drone_dev:drone_dev src/mpc ${WORKSPACE}/src/mpc
COPY --chown=drone_dev:drone_dev src/drone_control ${WORKSPACE}/src/drone_control
COPY --chown=drone_dev:drone_dev src/betaflight_msp_controller ${WORKSPACE}/src/betaflight_msp_controller
COPY --chown=drone_dev:drone_dev src/my_mavsdk ${WORKSPACE}/src/my_mavsdk
COPY --chown=drone_dev:drone_dev launch ${WORKSPACE}/launch
WORKDIR ${WORKSPACE}
RUN if [ -n "$BUILD_JOBS" ]; then export MAKEFLAGS="-j$BUILD_JOBS"; fi; \
    source /opt/ros/humble/setup.bash && colcon build --symlink-install ${BUILD_JOBS:+--parallel-workers 2} --packages-up-to drone_control my_mavsdk mpc betaflight_msp_controller

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
COPY --chown=drone_dev:drone_dev src/betaflight_msp_controller ${WORKSPACE}/src/betaflight_msp_controller
COPY --chown=drone_dev:drone_dev src/groundstation ${WORKSPACE}/src/groundstation
COPY --chown=drone_dev:drone_dev launch ${WORKSPACE}/launch
# Add docs
COPY --chown=drone_dev:drone_dev docs /home/drone_dev/docs
RUN uv pip install --system --no-cache "mkdocs<2.0" "mkdocs-material==9.5.34" "pymdown-extensions<11.0" fastapi uvicorn av

USER drone_dev
WORKDIR ${WORKSPACE}
RUN echo "source ${WORKSPACE}/install/setup.bash" >> /home/drone_dev/.bashrc
CMD ["/bin/bash"]

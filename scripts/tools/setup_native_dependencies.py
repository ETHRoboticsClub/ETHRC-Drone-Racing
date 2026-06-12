#!/usr/bin/env python3
import os
import subprocess
import sys
import platform

def run(cmd, shell=True):
    print(f"Running: {cmd}")
    subprocess.check_call(cmd, shell=shell)

def try_run(cmd, warning, shell=True):
    try:
        run(cmd, shell=shell)
    except subprocess.CalledProcessError as e:
        print(f"Warning: {warning}")
        print(f"Command failed with exit code {e.returncode}: {cmd}")

def get_arch():
    return platform.machine()

def setup_native():
    print("=== ETHRC Drone Racing - Native Dependencies Setup ===")
    arch = get_arch()
    print(f"Detected architecture: {arch}")

    # 0. Initialize Submodules
    print("--- Initializing Submodslules ---")
    try:
        run("git submodule update --init --recursive")
    except Exception as e:
        print(f"Warning: Submodule update failed: {e}")
        print("Continuing setup, but you might need to fix submodules manually.")

    # 1. System Dependencies
    print("--- Installing System Dependencies ---")
    sys_deps = [
        "libprotobuf-dev", "protobuf-compiler", "libcurl4-openssl-dev", "libtinyxml2-dev",
        "libjsoncpp-dev", "libssl-dev", "libblas-dev", "liblapack-dev", "libopenblas-dev",
        "liblapacke-dev", "gfortran", "cmake", "python3-pip", "python3-dev", "libserialport-dev",
        "libasio-dev", "libceres-dev", "libgoogle-glog-dev", "libgflags-dev",
        "libatlas-base-dev", "libsuitesparse-dev", "python3-matplotlib", "python3-psutil",
        "python3-tk", "python3-gi", "gir1.2-gstreamer-1.0", "gstreamer1.0-tools",
        "gstreamer1.0-plugins-base", "gstreamer1.0-plugins-good",
        "libavcodec-dev", "libavformat-dev", "libavutil-dev", "libswscale-dev", "libavdevice-dev"
    ]
    
    # On non-Jetson, we can safely install -bad. On Jetson, we are careful.
    if arch != "aarch64":
        sys_deps.append("gstreamer1.0-plugins-bad")
    
    # ROS distro detection
    ros_distro = "humble"
    if os.path.exists("/opt/ros/rolling"):
        ros_distro = "rolling"
    elif os.path.exists("/opt/ros/jazzy"):
        ros_distro = "jazzy"
    print(f"Detected ROS distro: {ros_distro}")

    # ROS 2 specific dependencies
    ros_deps = [
        f"ros-{ros_distro}-mavros",
        f"ros-{ros_distro}-mavros-msgs",
        f"ros-{ros_distro}-camera-calibration-parsers",
        f"ros-{ros_distro}-cv-bridge",
        f"ros-{ros_distro}-image-transport",
        f"ros-{ros_distro}-diagnostic-updater",
        f"ros-{ros_distro}-foxglove-bridge",
    ]
    if ros_distro != "rolling":
        ros_deps.append(f"ros-{ros_distro}-nav2-msgs")

    sys_deps.extend(ros_deps)

    if ros_distro in ["rolling", "jazzy"]:
        if "libatlas-base-dev" in sys_deps:
            sys_deps.remove("libatlas-base-dev")

    # Check if ROS Rolling needs nav2_msgs from source
    if ros_distro == "rolling":
        ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
        if not os.path.exists(os.path.join(ws_root, "src/nav2_msgs")):
            print("--- Cloning nav2_msgs for ROS Rolling ---")
            run(f"git clone -b main --filter=blob:none --sparse https://github.com/ros-navigation/navigation2.git /tmp/navigation2 && "
                f"cd /tmp/navigation2 && git sparse-checkout set nav2_msgs && "
                f"cp -r nav2_msgs {ws_root}/src/ && rm -rf /tmp/navigation2")

    if arch == "aarch64":
        # Core NITROS dependencies (only on Jetson Orin / aarch64)
        sys_deps.extend([
            f"ros-{ros_distro}-isaac-ros-argus-camera",
            f"ros-{ros_distro}-isaac-ros-image-proc",
        ])
        print("--- Adding Jetson CUDA, cuDNN and Profiling libraries ---")
        sys_deps.extend([
            "libcudnn9-cuda-12",
            "cuda-cupti-12-6",
            "cuda-libraries-12-6",
            "cuda-libraries-dev-12-6"
        ])

    run(f"sudo apt-get update || echo 'Warning: apt-get update failed' && sudo apt-get install -y {' '.join(sys_deps)}")

    if arch == "aarch64":
        print("--- Installing cuSPARSELt library on Jetson (aarch64) ---")
        if not os.path.exists("/usr/lib/aarch64-linux-gnu/libcusparseLt.so.0"):
            try:
                run("cd /tmp && "
                    "wget -q https://developer.download.nvidia.com/compute/cusparselt/redist/libcusparse_lt/linux-aarch64/libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive.tar.xz && "
                    "tar xf libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive.tar.xz && "
                    "sudo cp -a libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive/include/* /usr/local/cuda/include/ && "
                    "sudo cp -a libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive/lib/* /usr/local/cuda/targets/aarch64-linux/lib/ && "
                    "sudo cp -a libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive/include/* /usr/local/cuda-12.6/include/ && "
                    "sudo cp -a libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive/lib/* /usr/local/cuda-12.6/targets/aarch64-linux/lib/ && "
                    "sudo cp -a libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive/lib/* /usr/lib/aarch64-linux-gnu/ && "
                    "sudo ldconfig")
            except Exception as e:
                print(f"Warning: cuSPARSELt library setup failed: {e}")

    # Isaac ROS Compression is the preferred H.264 path for racing bringup.
    # h264_image_transport and jetson_h264_publisher remain in-tree as fallback/debug
    # tools, but native installs should use the NITROS/NVENC-backed Isaac packages
    # when the NVIDIA Isaac ROS apt repository is configured.
    isaac_h264_deps = ["ros-humble-isaac-ros-h264-decoder"]
    if arch == "aarch64":
        isaac_h264_deps.insert(0, "ros-humble-isaac-ros-h264-encoder")

    print("--- Installing Isaac ROS H.264 Dependencies ---")
    try_run(
        f"sudo apt-get install -y {' '.join(isaac_h264_deps)}",
        "Isaac ROS H.264 packages were not installed. "
        "Configure the NVIDIA Isaac ROS apt repository or build "
        "isaac_ros_compression from source before using h264_backend:=isaac_ros.",
    )

    # MAVROS GeographicLib datasets
    geo_path = f"/opt/ros/{ros_distro}/lib/mavros/install_geographiclib_datasets.sh"
    if os.path.exists(geo_path):
        print("--- Installing MAVROS GeographicLib Datasets ---")
        run(f"sudo {geo_path}")

    # 2. MAVSDK C++
    if not os.path.exists("/usr/local/include/mavsdk"):
        print("--- Building MAVSDK ---")
        run("git clone --recursive https://github.com/mavlink/MAVSDK.git /tmp/MAVSDK && "
            "cd /tmp/MAVSDK && git checkout v3.10.2 && "
            "cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON && "
            "cmake --build build -j$(nproc) && sudo cmake --install build && sudo ldconfig")

    # 3. ACADOS C++
    acados_path = os.path.expanduser("~/acados")
    if not os.path.exists(acados_path):
        print("--- Building acados ---")
        run(f"git clone https://github.com/acados/acados.git {acados_path} && "
            f"cd {acados_path} && git submodule update --recursive --init && "
            f"mkdir -p build && cd build && cmake -DACADOS_SHARED_LIBS=ON -DACADOS_WITH_QPOASES=ON .. && "
            f"make install -j$(nproc)")

    # 4. MSP C++ Library
    if not os.path.exists("/usr/local/include/msp"):
        print("--- Building MSP Library ---")
        # Note: If src/msp exists as a submodule, we could build from there, 
        # but the script traditionally clones to /tmp. Let's keep it consistent.
        run("git clone https://github.com/christianrauch/msp.git /tmp/msp && "
            "cd /tmp/msp && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && "
            "cmake --build build -j$(nproc) && sudo cmake --install build && sudo ldconfig")

    # 5. Tera Renderer
    tera_path = os.path.join(acados_path, "bin/t_renderer")
    needs_download = not os.path.exists(tera_path)
    
    if not needs_download:
        # Check if architecture matches
        file_out = subprocess.check_output(f"file {tera_path}", shell=True).decode()
        if arch == "aarch64" and "ARM aarch64" not in file_out:
            needs_download = True
        elif arch == "x86_64" and "x86-64" not in file_out:
            needs_download = True

    if needs_download:
        print(f"--- Downloading Tera Renderer for {arch} ---")
        tera_url = "https://github.com/acados/tera_renderer/releases/download/v0.2.0/"
        if arch == "aarch64":
            tera_url += "t_renderer-v0.2.0-linux-arm64"
        else:
            tera_url += "t_renderer-v0.2.0-linux-amd64"
        
        run(f"mkdir -p {os.path.dirname(tera_path)} && "
            f"wget -q {tera_url} -O {tera_path} && "
            f"chmod +x {tera_path}")

    # 6. Python Dependencies
    print("--- Installing Python Dependencies ---")
    # Ensure pyserial is installed and the conflicting 'serial' package is removed
    run("pip3 uninstall -y serial || true")
    
    # Core dependencies
    py_deps = ["pyserial", "numpy<2.0", "scipy", "mavsdk", "casadi", "cython", "onnxruntime", "onnx"]
    if ros_distro == "humble":
        py_deps.append("setuptools==58.2.0")

    # RL/Inference dependencies
    # On Jetson, we use the NVIDIA-optimized torch wheels for CUDA support
    if arch == "aarch64":
        print("--- Detected Jetson (aarch64). Attempting to install NVIDIA-optimized Torch ---")
        # Default for JetPack 6.2 (JP6.2 / CUDA 12.6)
        torch_url = "https://developer.download.nvidia.com/compute/redist/jp/v62/pytorch/torch-2.5.0a0+872d974e.nv24.08-cp310-cp310-linux_aarch64.whl"
        torchvision_url = "https://developer.download.nvidia.com/compute/redist/jp/v62/pytorch/torchvision-0.20.0a0+635759a-cp310-cp310-linux_aarch64.whl"

        try:
            # Adding torch-tensorrt for maximum inference performance on Orin
            run(f"pip3 install --user {torch_url} {torchvision_url} torch-tensorrt")
            # Filter out torch/vision from py_deps so they aren't re-installed
            py_deps = [d for d in py_deps if d not in ["torch", "torchvision", "torchaudio"]]
        except Exception as e:
            print(f"Warning: Optimized Torch install failed: {e}. Falling back to standard pip.")
    
    # Check if PEP 668 externally managed environment requires --break-system-packages
    pip_extra = ""
    try:
        import glob
        if sys.prefix == sys.base_prefix:
            if glob.glob("/usr/lib/python3.*/EXTERNALLY-MANAGED") or os.path.exists("/usr/lib/EXTERNALLY-MANAGED"):
                pip_extra = " --break-system-packages"
    except Exception:
        pass

    # Quote dependencies to prevent shell redirection (e.g., 'numpy<2.0')
    quoted_deps = [f"'{d}'" for d in py_deps]
    
    run(f"pip3 install --user{pip_extra} {' '.join(quoted_deps)}")
    run(f"pip3 install --user{pip_extra} -e {acados_path}/interfaces/acados_template")

    # 7. Environment Variables
    bashrc = os.path.expanduser("~/.bashrc")
    env_vars = [
        f'export ACADOS_SOURCE_DIR="{acados_path}"',
        f'export TERA_PATH="{tera_path}"',
        f'export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:{acados_path}/lib"',
        f'export PYTHONPATH="$PYTHONPATH:{acados_path}/interfaces/acados_template"',
        f'source /opt/ros/{ros_distro}/setup.bash'
    ]
    
    with open(bashrc, "a+") as f:
        f.seek(0)
        content = f.read()
        for var in env_vars:
            if var not in content:
                print(f"Adding to .bashrc: {var}")
                f.write(f"\n{var}\n")

    print("\n=== SETUP COMPLETE ===")
    print("Please run: source ~/.bashrc")
    print("Check submodules status: git submodule status")

if __name__ == "__main__":
    setup_native()

#!/usr/bin/env python3
import os
import subprocess
import sys
import platform

def run(cmd, shell=True):
    print(f"Running: {cmd}")
    subprocess.check_call(cmd, shell=shell)

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
    
    # ROS 2 Humble specific dependencies
    sys_deps.extend(["ros-humble-mavros", "ros-humble-mavros-msgs", "ros-humble-camera-calibration-parsers"])

    run(f"sudo apt-get update || echo 'Warning: apt-get update failed' && sudo apt-get install -y {' '.join(sys_deps)}")

    # MAVROS GeographicLib datasets
    if os.path.exists("/opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh"):
        print("--- Installing MAVROS GeographicLib Datasets ---")
        run("sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh")

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
    if not os.path.exists(tera_path):
        print("--- Downloading Tera Renderer ---")
        tera_url = "https://github.com/acados/tera_renderer/releases/download/v0.2.0/"
        if arch == "aarch64":
            tera_url += "t_renderer-v0.2.0-linux-arm64"
        else:
            tera_url += "t_renderer-v0.2.0-linux-amd64"
        
        run(f"mkdir -p {os.path.dirname(tera_path)} && "
            f"wget {tera_url} -O {tera_path} && "
            f"chmod +x {tera_path}")

    # 6. Python Dependencies
    print("--- Installing Python Dependencies ---")
    # Ensure pyserial is installed and the conflicting 'serial' package is removed
    run("pip3 uninstall -y serial || true")
    run("pip3 install --user pyserial 'numpy<2.0' mavsdk casadi cython setuptools==58.2.0")
    run(f"pip3 install --user -e {acados_path}/interfaces/acados_template")

    # 7. Environment Variables
    bashrc = os.path.expanduser("~/.bashrc")
    env_vars = [
        f'export ACADOS_SOURCE_DIR="{acados_path}"',
        f'export TERA_PATH="{tera_path}"',
        f'export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:{acados_path}/lib"',
        f'export PYTHONPATH="$PYTHONPATH:{acados_path}/interfaces/acados_template"',
        'source /opt/ros/humble/setup.bash'
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

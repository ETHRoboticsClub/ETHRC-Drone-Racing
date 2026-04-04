#!/usr/bin/env python3
import os
import subprocess
import sys

def run(cmd, shell=True):
    print(f"Running: {cmd}")
    subprocess.check_call(cmd, shell=shell)

def setup_native():
    print("=== Installing Native MPC/MAVSDK Dependencies ===")
    
    # 1. System Dependencies
    run("sudo apt-get update && sudo apt-get install -y "
        "libprotobuf-dev protobuf-compiler libcurl4-openssl-dev libtinyxml2-dev "
        "libjsoncpp-dev libssl-dev libblas-dev liblapack-dev libopenblas-dev "
        "liblapacke-dev gfortran cmake python3-pip python3-dev")

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

    # 4. Tera Renderer (arm64)
    tera_path = os.path.join(acados_path, "bin/t_renderer")
    if not os.path.exists(tera_path):
        print("--- Downloading Tera Renderer ---")
        run(f"mkdir -p {os.path.dirname(tera_path)} && "
            f"wget https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-arm64 -O {tera_path} && "
            f"chmod +x {tera_path}")

    # 5. Python Dependencies
    run("pip3 install --user mavsdk casadi cython setuptools==58.2.0")
    run(f"pip3 install --user -e {acados_path}/interfaces/acados_template")

    # 6. Environment Variables
    bashrc = os.path.expanduser("~/.bashrc")
    env_vars = [
        f'export ACADOS_SOURCE_DIR="{acados_path}"',
        f'export TERA_PATH="{tera_path}"',
        f'export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:{acados_path}/lib"',
        f'export PYTHONPATH="$PYTHONPATH:{acados_path}/interfaces/acados_template"'
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

if __name__ == "__main__":
    setup_native()

#!/bin/bash

# Colori
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Setup Ambiente Drone MPC ===${NC}"

# 1. Verifica ACADOS
if [ -d "$HOME/acados" ]; then
    echo -e "${GREEN}✓ ACADOS trovato${NC}"
    export ACADOS_SOURCE_DIR=$HOME/acados
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/acados/lib
    export PYTHONPATH=$PYTHONPATH:$HOME/acados/interfaces/acados_template
else
    echo -e "${RED}✗ ACADOS non trovato in $HOME/acados${NC}"
    exit 1
fi

# 2. Verifica Python packages
echo "Verifica dipendenze Python..."
python3 -c "import casadi; import numpy; import scipy" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Dipendenze Python OK${NC}"
else
    echo -e "${RED}✗ Mancano dipendenze Python${NC}"
    echo "Installa con: pip3 install casadi numpy scipy"
    exit 1
fi

# 3. Verifica ROS
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ ROS non configurato${NC}"
    echo "Esegui: source /opt/ros/noetic/setup.bash"
    exit 1
else
    echo -e "${GREEN}✓ ROS $ROS_DISTRO configurato${NC}"
fi

# 4. Compila il workspace
echo "Compilazione workspace..."
cd ~/catkin_ws
catkin_make
source devel/setup.bash

echo -e "${GREEN}=== Setup completato! ===${NC}"
echo ""
echo "Per avviare la simulazione:"
echo "  Terminale 1: cd ~/PX4-Autopilot && make px4_sitl gazebo"
echo "  Terminale 2: roslaunch drone_mpc_control drone_mpc.launch"
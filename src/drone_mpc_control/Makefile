.PHONY: build run clean test gazebo

# Setup
setup:
	@echo "Setup ambiente..."
	@chmod +x scripts/*.py
	@chmod +x setup_environment.sh
	@./setup_environment.sh

# Build
build:
	@echo "Compilazione workspace..."
	@cd ~/catkin_ws && catkin_make
	@source ~/catkin_ws/devel/setup.bash

# Run simulation
gazebo:
	@echo "Avvio Gazebo + PX4..."
	@cd ~/PX4-Autopilot && make px4_sitl gazebo

# Run controller
run:
	@echo "Avvio controllore MPC..."
	@roslaunch drone_mpc_control drone_mpc.launch

# Test ACADOS standalone
test:
	@echo "Test ACADOS..."
	@cd scripts && python3 test_acados.py

# Clean
clean:
	@echo "Pulizia..."
	@rm -rf ~/catkin_ws/build ~/catkin_ws/devel
	@rm -rf scripts/acados_models
	@rm -f scripts/*.json
	@rm -f scripts/*.csv
	@find . -name "*.pyc" -delete
	@find . -name "__pycache__" -delete
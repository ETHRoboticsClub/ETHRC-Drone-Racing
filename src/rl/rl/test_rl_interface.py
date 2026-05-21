#!/usr/bin/env python3

import numpy as np
import sys
import os
from pathlib import Path

# Add the src directory to Python path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

# Also try sourcing ROS setup if available
ros_setup = '/home/drone_dev/ros_ws/install/setup.bash'
if os.path.exists(ros_setup):
    import subprocess
    subprocess.run(['bash', '-c', f'source {ros_setup} && env'], capture_output=True)
    # This won't affect the current process, but let's try importing directly

# Try direct import with full path as fallback
try:
    from rl.drone_rl_controller import DroneRLController
except ImportError:
    import importlib.util
    
    # Load interfaces first
    spec_interfaces = importlib.util.spec_from_file_location("interfaces", "/home/drone_dev/ros_ws/src/interfaces/__init__.py")
    if spec_interfaces and spec_interfaces.loader:
        interfaces_module = importlib.util.module_from_spec(spec_interfaces)
        spec_interfaces.loader.exec_module(interfaces_module)
        sys.modules['interfaces'] = interfaces_module
    
    # Load controller_interface
    spec_ci = importlib.util.spec_from_file_location("controller_interface", "/home/drone_dev/ros_ws/src/interfaces/controller_interface.py")
    if spec_ci and spec_ci.loader:
        ci_module = importlib.util.module_from_spec(spec_ci)
        spec_ci.loader.exec_module(ci_module)
        interfaces_module.ControllerInterface = ci_module.ControllerInterface
    
    # Load policy_model
    spec_pm = importlib.util.spec_from_file_location("policy_model", "/home/drone_dev/ros_ws/src/rl/policy_model.py")
    if spec_pm and spec_pm.loader:
        pm_module = importlib.util.module_from_spec(spec_pm)
        spec_pm.loader.exec_module(pm_module)
    
    # Load drone_rl_controller
    spec = importlib.util.spec_from_file_location("drone_rl_controller", "/home/drone_dev/ros_ws/src/rl/drone_rl_controller.py")
    rl_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rl_module)
    DroneRLController = rl_module.DroneRLController

from mavsdk.offboard import AttitudeRate

def test_rl_controller_interface():
    """Test that RL controller has the correct interface and parameters."""

    print("Testing RL Controller Interface...")

    # Instantiate the controller
    controller = DroneRLController()

    # Check if it has the required methods
    assert hasattr(controller, 'initialize'), "Missing initialize method"
    assert hasattr(controller, 'controller_update'), "Missing controller_update method"
    assert hasattr(controller, 'trajectory_controller_update'), "Missing trajectory_controller_update method"

    # Initialize (this loads the policy model)
    try:
        controller.initialize()
        print("✓ Controller initialized successfully")
    except Exception as e:
        print(f"✗ Initialization failed: {e}")
        return False

    # Test controller_update with sample data
    # Sample drone state: [p(3), v(3), q(4), ω(3)] = 13 dims
    x_current = np.array([
        0.0, 0.0, 1.0,  # position
        0.0, 0.0, 0.0,  # velocity
        1.0, 0.0, 0.0, 0.0,  # quaternion (w,x,y,z)
        0.0, 0.0, 0.0   # angular velocity
    ])

    # Sample goal state: same format
    goal_position = np.array([
        1.0, 0.0, 2.0,  # position
        0.0, 0.0, 0.0,  # velocity
        1.0, 0.0, 0.0, 0.0,  # quaternion
        0.0, 0.0, 0.0   # angular velocity
    ])

    try:
        result = controller.controller_update(x_current, goal_position)
        print("✓ controller_update called successfully")
        print(f"  Returned type: {type(result)}")
        assert isinstance(result, AttitudeRate), f"Expected AttitudeRate, got {type(result)}"
        print(f"  AttitudeRate: roll={result.roll_deg_s}, pitch={result.pitch_deg_s}, yaw={result.yaw_deg_s}, thrust={result.thrust_value}")
    except Exception as e:
        print(f"✗ controller_update failed: {e}")
        return False

    # Test trajectory_controller_update with sample data
    # Sample trajectory: shape (13, N) - let's use N=2 for simplicity
    goal_trajectory = np.array([
        [1.0, 2.0],  # x positions
        [0.0, 0.0],  # y
        [2.0, 3.0],  # z
        [0.0, 0.0],  # vx
        [0.0, 0.0],  # vy
        [0.0, 0.0],  # vz
        [1.0, 1.0],  # qw
        [0.0, 0.0],  # qx
        [0.0, 0.0],  # qy
        [0.0, 0.0],  # qz
        [0.0, 0.0],  # ωx
        [0.0, 0.0],  # ωy
        [0.0, 0.0]   # ωz
    ])

    try:
        result_traj = controller.trajectory_controller_update(x_current, goal_trajectory)
        print("✓ trajectory_controller_update called successfully")
        print(f"  Returned type: {type(result_traj)}")
        assert isinstance(result_traj, AttitudeRate), f"Expected AttitudeRate, got {type(result_traj)}"
        print(f"  AttitudeRate: roll={result_traj.roll_deg_s}, pitch={result_traj.pitch_deg_s}, yaw={result_traj.yaw_deg_s}, thrust={result_traj.thrust_value}")
    except Exception as e:
        print(f"✗ trajectory_controller_update failed: {e}")
        return False

    print("✓ All interface checks passed!")
    return True

if __name__ == "__main__":
    success = test_rl_controller_interface()
    sys.exit(0 if success else 1)
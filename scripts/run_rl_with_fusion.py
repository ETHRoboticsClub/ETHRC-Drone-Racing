#!/usr/bin/env python3
"""
Runs the Nicola RL policy model with non-ground-truth state estimation:
- AigpMavlinkBridge (MAVLink bridge -> /imu0)
- GateDetectorNode (Neon orange vision PnP -> /aigp_state_estimator/set_pose)
- AigpStateEstimatorNode (IMU + Vision fusion -> /drone_state_estimated)
- AigpPolicyNode (Nicola TorchScript RL policy -> /drone_attitude_cmd)
"""

import os
import sys
from pathlib import Path
import rclpy
from rclpy.executors import MultiThreadedExecutor

# Import nodes
from betaflight_mavlink_bridge.aigp_bridge_node import AigpMavlinkBridge
from gate_detection.detector_node import GateDetectorNode
from rl.aigp_state_estimator_node import AigpStateEstimatorNode
from rl.aigp_policy_node import AigpPolicyNode


def main(args=None):
    rclpy.init(args=args)

    project_root = Path(__file__).resolve().parent.parent
    policy_path = project_root / "ext" / "nicola" / "policy.pt"
    track_path = project_root / "src" / "rl" / "config" / "aigp_competition_track.yaml"

    print("==================================================")
    print("  LAUNCHING NICOLA RL MODEL WITH SENSOR FUSION    ")
    print("==================================================")
    print(f"Policy Path: {policy_path}")
    print(f"Track Path:  {track_path}")
    print("State Topic: /drone_state_estimated (IMU + Vision Fusion)")
    print("--------------------------------------------------")

    # Override parameter values directly on node creation
    bridge_node = AigpMavlinkBridge()

    # Pass parameters to GateDetectorNode
    detector_node = GateDetectorNode()

    # Pass parameters to AigpStateEstimatorNode
    estimator_node = AigpStateEstimatorNode()

    # Pass parameters to AigpPolicyNode
    policy_node = AigpPolicyNode()

    executor = MultiThreadedExecutor()
    executor.add_node(bridge_node)
    executor.add_node(detector_node)
    executor.add_node(estimator_node)
    executor.add_node(policy_node)

    print("All 4 RL pipeline nodes added to executor. Spinning...")
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\nShutting down RL policy pipeline...")
    finally:
        bridge_node.destroy_node()
        detector_node.destroy_node()
        estimator_node.destroy_node()
        policy_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

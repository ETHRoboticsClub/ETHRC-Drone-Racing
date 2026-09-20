import os
import sys
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

# Add gate_detection directory to python path
this_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.abspath(os.path.join(this_dir, '..'))
if pkg_dir not in sys.path:
    sys.path.insert(0, pkg_dir)

import rclpy
from gate_detection.detector_node import (
    R_TRACK_FROM_PNP_GATE,
    GateDetectorNode,
    camera_optical_to_body_flu,
    pnp_gate_pose_to_body_nwu,
    translation_with_known_rotation,
)


def make_math_node():
    """Build only the pure OpenCV portion of the node (no ROS graph required)."""
    node = GateDetectorNode.__new__(GateDetectorNode)
    half_width = 2.7 / 2.0
    node.gate_points_3d = np.array([
        [-half_width, -half_width, 0.0],
        [half_width, -half_width, 0.0],
        [half_width, half_width, 0.0],
        [-half_width, half_width, 0.0],
    ], dtype=np.float32)
    node.camera_matrix = np.array([
        [320.0, 0.0, 320.0],
        [0.0, 320.0, 180.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)
    node.dist_coeffs = np.zeros(5, dtype=np.float32)
    node.active_gate_index = 0
    node.last_candidate_corners = None
    node.last_pose_solution = None
    node.last_solution_gate_index = None
    node.min_contour_area = 100.0
    node.max_reprojection_error_px = 6.0
    node.min_pnp_distance_m = 0.2
    node.max_pnp_distance_m = 60.0
    node.require_state_consistency = False
    node.max_position_innovation_m = 3.0
    node.latest_state_position_ned = None
    return node


def test_candidate_selection_rejects_larger_orange_streak():
    node = make_math_node()
    mask = np.zeros((360, 640), dtype=np.uint8)
    # Larger by pixel count, but far too elongated to be a projected gate.
    cv2.rectangle(mask, (10, 20), (620, 60), 255, thickness=-1)
    # A smaller closed frame is the geometrically plausible candidate.
    cv2.rectangle(mask, (220, 100), (420, 300), 255, thickness=14)

    candidate = node.select_gate_candidate(mask)

    assert candidate is not None
    assert np.allclose(np.mean(candidate['corners'], axis=0), [320.0, 200.0], atol=3.0)


def test_candidate_selection_uses_temporal_continuity():
    node = make_math_node()
    mask = np.zeros((360, 640), dtype=np.uint8)
    cv2.rectangle(mask, (80, 100), (220, 240), 255, thickness=12)
    cv2.rectangle(mask, (400, 100), (540, 240), 255, thickness=12)
    node.last_candidate_corners = np.array([
        [80.0, 100.0], [220.0, 100.0],
        [220.0, 240.0], [80.0, 240.0],
    ], dtype=np.float32)

    candidate = node.select_gate_candidate(mask)

    assert candidate is not None
    assert np.mean(candidate['corners'][:, 0]) < 300.0


def test_planar_pnp_recovers_synthetic_pose_and_rejects_negative_depth():
    node = make_math_node()
    expected_rvec = np.array([[0.08], [-0.12], [0.03]], dtype=np.float64)
    expected_tvec = np.array([[0.25], [-0.10], [7.0]], dtype=np.float64)
    image_points, _ = cv2.projectPoints(
        node.gate_points_3d, expected_rvec, expected_tvec,
        node.camera_matrix, node.dist_coeffs)

    solution = node.solve_gate_pose(image_points.reshape(4, 2))

    assert solution is not None
    assert solution['reprojection_rms'] < 0.1
    assert np.allclose(solution['tvec'], expected_tvec, atol=0.03)
    behind_camera = node._pose_candidate(
        np.zeros((3, 1)), np.array([[0.0], [0.0], [-5.0]]),
        image_points.reshape(4, 2))
    assert behind_camera is None


def test_known_rotation_translation_recovers_distant_planar_gate():
    node = make_math_node()
    rotation = R.from_euler('xyz', [4.0, -8.0, 3.0], degrees=True).as_matrix()
    expected_translation = np.array([[0.25], [-0.10], [23.0]])
    image_points, _ = cv2.projectPoints(
        node.gate_points_3d,
        cv2.Rodrigues(rotation)[0],
        expected_translation,
        node.camera_matrix,
        node.dist_coeffs,
    )

    actual_translation = translation_with_known_rotation(
        node.gate_points_3d,
        image_points.reshape(4, 2),
        node.camera_matrix,
        rotation,
    )

    np.testing.assert_allclose(actual_translation, expected_translation, atol=1.0e-5)


def test_full_optical_extrinsic_recovers_synthetic_body_pose():
    node = make_math_node()
    body_position_w = np.array([1.2, -0.7, 2.4], dtype=np.float64)
    rotation_w_body = R.from_euler(
        'xyz', [7.0, -11.0, 28.0], degrees=True
    ).as_matrix()
    rotation_body_camera = camera_optical_to_body_flu(20.0)
    rotation_w_camera = rotation_w_body @ rotation_body_camera

    # Put a mildly oblique gate eight metres in front of the optical camera.
    rotation_camera_pnp_gate = R.from_euler(
        'xyz', [4.0, -8.0, 3.0], degrees=True
    ).as_matrix()
    rotation_w_pnp_gate = rotation_w_camera @ rotation_camera_pnp_gate
    rotation_w_gate = rotation_w_pnp_gate @ R_TRACK_FROM_PNP_GATE.T
    gate_position_w = body_position_w + rotation_w_camera @ np.array([0.3, -0.2, 8.0])

    rotation_camera_from_pnp_gate = rotation_w_camera.T @ rotation_w_pnp_gate
    translation_camera_from_pnp_gate = (
        rotation_w_camera.T @ (gate_position_w - body_position_w)
    )
    expected_rvec, _ = cv2.Rodrigues(rotation_camera_from_pnp_gate)
    image_points, _ = cv2.projectPoints(
        node.gate_points_3d,
        expected_rvec,
        translation_camera_from_pnp_gate,
        node.camera_matrix,
        node.dist_coeffs,
    )

    solution = node.solve_gate_pose(image_points.reshape(4, 2))
    assert solution is not None
    actual_position_w, actual_rotation_w_body = pnp_gate_pose_to_body_nwu(
        solution['rvec'],
        solution['tvec'],
        gate_position_w,
        rotation_w_gate,
        camera_up_tilt_deg=20.0,
    )

    np.testing.assert_allclose(actual_position_w, body_position_w, atol=2.0e-4)
    rotation_error = R.from_matrix(
        actual_rotation_w_body.T @ rotation_w_body
    ).magnitude()
    assert rotation_error < 2.0e-4


def test_covariance_increases_with_reprojection_error_and_distance():
    node = make_math_node()
    corners = np.array([
        [240.0, 100.0], [400.0, 100.0],
        [400.0, 260.0], [240.0, 260.0],
    ], dtype=np.float32)

    good = node.estimate_pose_covariance(corners, np.array([0.0, 0.0, 5.0]), 0.5)
    noisy = node.estimate_pose_covariance(corners, np.array([0.0, 0.0, 10.0]), 4.0)

    assert good.shape == (6, 6)
    assert np.all(np.diag(good) > 0.0)
    assert np.all(np.diag(noisy) > np.diag(good))


def test_detector_coordinate_math():
    if not rclpy.ok():
        rclpy.init()

    # Instantiate node
    node = GateDetectorNode()
    
    # 1. Setup a test gate: placed at world NWU pos [-23.298, 0.400, 0.032], rotated 270 deg (facing +Y)
    # The rotation representation in the node for 270 deg is:
    # R.from_euler("xyz", [0, 0, -270], degrees=True)
    gate_pos_w = np.array([-23.298, 0.400, 0.032], dtype=np.float64)
    R_w_gate = R.from_euler("xyz", [0, 0, -270], degrees=True).as_matrix()
    
    node.gates = [{'position': gate_pos_w, 'rotation': R.from_matrix(R_w_gate)}]
    node.active_gate_index = 0
    
    # 2. Assume the drone is exactly at world NWU pos [-28.298, 0.400, 0.032] (5m back from the gate along -X)
    # and has yaw=0, pitch=20 degrees (meaning camera is pitched up 20 degrees, making it look horizontal).
    # Relative offset from camera to gate in world: P_gate - P_cam = [5.0, 0.0, 0.0]
    # Since the gate's forward direction is +Y (yaw = 270 = -90 deg), the offset in the gate's Track frame is:
    # T_cam_gate_track = R_w_gate^T @ (P_cam - P_gate) = R_z(-90)^T @ [-5, 0, 0]^T
    # R_z(-90) = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
    # The stored 270-degree clockwise gate yaw becomes +90 degrees in scipy,
    # so a camera 5 m toward world -X is at local gate offset [0, +5, 0].
    T_cam_gate_track = np.array([0.0, 5.0, 0.0])
    
    # Relative rotation: camera has yaw=0, pitch=20 degrees.
    # Camera rotation in world: R_w_cam = R_z(0) * R_y(0) * R_x(20) = R_x(20)
    # Camera rotation relative to gate: R_cam_gate_track = R_w_gate^T * R_w_cam = R_z(-90)^T * R_x(20)
    R_w_cam = R.from_euler('xyz', [20.0, 0.0, 0.0], degrees=True).as_matrix()
    R_cam_gate_track = R_w_gate.T @ R_w_cam
    
    # 3. Convert from Track gate frame back to PnP camera frame (to simulate solvePnP output)
    # Track: X right, Y forward (normal), Z up
    # PnP: X right, Y down, Z forward
    # R_t_p = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
    # We have T_cam_gate = R_t_p^T @ T_cam_gate_track
    # And R_cam_gate = R_t_p^T @ R_cam_gate_track
    # Since rvec and tvec map from gate to camera, we have:
    # tvec = -R_cam_gate^T @ T_cam_gate
    # R_gate_cam = R_cam_gate^T
    R_t_p = np.array([
        [1,  0, 0],
        [0,  0, 1],
        [0, -1, 0]
    ], dtype=np.float64)
    
    T_cam_gate = R_t_p.T @ T_cam_gate_track
    R_cam_gate = R_t_p.T @ R_cam_gate_track
    
    rvec, _ = cv2.Rodrigues(R_cam_gate.T)
    tvec = -R_cam_gate.T @ T_cam_gate
    
    # 4. Now, run the node's transformation logic step-by-step to verify it gets back the correct world pose
    R_gate_cam, _ = cv2.Rodrigues(rvec)
    T_cam_gate_reconstructed = -R_gate_cam.T @ tvec
    R_cam_gate_reconstructed = R_gate_cam.T
    
    # Check intermediate
    assert np.allclose(T_cam_gate, T_cam_gate_reconstructed)
    assert np.allclose(R_cam_gate, R_cam_gate_reconstructed)
    
    # Reconstruct Track frame pose
    T_cam_gate_track_reconstructed = R_t_p @ T_cam_gate_reconstructed
    R_cam_gate_track_reconstructed = R_t_p @ R_cam_gate_reconstructed
    
    assert np.allclose(T_cam_gate_track, T_cam_gate_track_reconstructed)
    assert np.allclose(R_cam_gate_track, R_cam_gate_track_reconstructed)
    
    # World pose in NWU
    cam_pos_w = gate_pos_w + R_w_gate @ T_cam_gate_track_reconstructed
    R_w_cam_reconstructed = R_w_gate @ R_cam_gate_track_reconstructed
    
    # Drone position is same as camera position in this model
    assert np.allclose(cam_pos_w, np.array([-28.298, 0.400, 0.032]))
    assert np.allclose(R_w_cam_reconstructed, R_w_cam)
    
    # Align to body by undoing camera pitch
    R_body_cam = R.from_euler('x', 20.0, degrees=True).as_matrix()
    R_w_body = R_w_cam_reconstructed @ R_body_cam.T
    
    # Drone body should have yaw=0, pitch=0, roll=0 relative to world!
    assert np.allclose(R_w_body, np.identity(3), atol=1e-6)
    
    # 5. Convert to AIGP NED/FRD
    FRAME_FLIP = np.array([1.0, -1.0, -1.0])
    pos_ned = FRAME_FLIP * cam_pos_w
    R_w_body_ned = np.diag(FRAME_FLIP) @ R_w_body @ np.diag(FRAME_FLIP)
    q_ned = R.from_matrix(R_w_body_ned).as_quat()
    
    # Expected position in NED: X = -28.298, Y = -0.400, Z = -0.032
    assert np.allclose(pos_ned, np.array([-28.298, -0.400, -0.032]))
    # Expected rotation: identity (represented as [0, 0, 0, 1] or [0, 0, 0, -1])
    assert np.allclose(np.abs(q_ned), np.array([0, 0, 0, 1]), atol=1e-6)

    rclpy.shutdown()

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from scipy.spatial.transform import Rotation as R


def _rotation_distance(rvec_a, rvec_b):
    """Return the geodesic distance between two Rodrigues rotations."""
    rot_a, _ = cv2.Rodrigues(np.asarray(rvec_a, dtype=np.float64))
    rot_b, _ = cv2.Rodrigues(np.asarray(rvec_b, dtype=np.float64))
    cosine = np.clip((np.trace(rot_a.T @ rot_b) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.arccos(cosine))


# PnP gate/object coordinates (right, down, forward) to the track gate frame
# (right, passage-forward, up).
R_TRACK_FROM_PNP_GATE = np.array(
    [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
    dtype=np.float64,
)

# Level camera optical coordinates (right, down, forward) expressed in body
# FLU coordinates (forward, left, up).
R_BODY_FROM_CAMERA_OPTICAL_LEVEL = np.array(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
    dtype=np.float64,
)


def camera_optical_to_body_flu(camera_up_tilt_deg=17.3):
    """Return body-FLU <- camera-optical rotation for the fixed mount."""

    # A positive upward camera tilt rotates optical +Z toward body +Z. In the
    # body FLU convention this is an active negative rotation about body +Y.
    tilt = R.from_euler('y', -float(camera_up_tilt_deg), degrees=True).as_matrix()
    return tilt @ R_BODY_FROM_CAMERA_OPTICAL_LEVEL


def pnp_gate_pose_to_body_nwu(
        rvec, tvec, gate_position_nwu, rotation_nwu_from_gate,
        camera_up_tilt_deg=17.3):
    """Recover coincident body pose from a gate-to-optical-camera PnP pose."""

    rotation_camera_from_pnp_gate, _ = cv2.Rodrigues(
        np.asarray(rvec, dtype=np.float64).reshape(3, 1)
    )
    translation_camera_from_pnp_gate = np.asarray(
        tvec, dtype=np.float64
    ).reshape(3)
    rotation_pnp_gate_from_camera = rotation_camera_from_pnp_gate.T
    camera_position_pnp_gate = (
        -rotation_pnp_gate_from_camera @ translation_camera_from_pnp_gate
    )

    camera_position_track_gate = R_TRACK_FROM_PNP_GATE @ camera_position_pnp_gate
    rotation_track_gate_from_camera = (
        R_TRACK_FROM_PNP_GATE @ rotation_pnp_gate_from_camera
    )
    rotation_nwu_from_gate = np.asarray(rotation_nwu_from_gate, dtype=np.float64)
    body_position_nwu = (
        np.asarray(gate_position_nwu, dtype=np.float64)
        + rotation_nwu_from_gate @ camera_position_track_gate
    )
    rotation_nwu_from_camera = (
        rotation_nwu_from_gate @ rotation_track_gate_from_camera
    )
    rotation_body_from_camera = camera_optical_to_body_flu(camera_up_tilt_deg)
    rotation_nwu_from_body = rotation_nwu_from_camera @ rotation_body_from_camera.T
    return body_position_nwu, rotation_nwu_from_body


def translation_with_known_rotation(
        object_points, image_points, camera_matrix, rotation_camera_from_object):
    """Solve camera-frame translation when the object rotation is known."""

    object_points = np.asarray(object_points, dtype=np.float64).reshape(-1, 3)
    image_points = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    rotation = np.asarray(rotation_camera_from_object, dtype=np.float64).reshape(3, 3)
    if len(object_points) != len(image_points) or len(object_points) < 2:
        raise ValueError('Object and image point counts must match')

    normalized = np.column_stack([
        (image_points[:, 0] - camera_matrix[0, 2]) / camera_matrix[0, 0],
        (image_points[:, 1] - camera_matrix[1, 2]) / camera_matrix[1, 1],
    ])
    rotated = (rotation @ object_points.T).T
    system = np.zeros((2 * len(object_points), 3), dtype=np.float64)
    target = np.zeros(2 * len(object_points), dtype=np.float64)
    for index, ((x_norm, y_norm), (x_obj, y_obj, z_obj)) in enumerate(
            zip(normalized, rotated)):
        system[2 * index] = [1.0, 0.0, -x_norm]
        target[2 * index] = x_norm * z_obj - x_obj
        system[2 * index + 1] = [0.0, 1.0, -y_norm]
        target[2 * index + 1] = y_norm * z_obj - y_obj
    translation, _, _, _ = np.linalg.lstsq(system, target, rcond=None)
    return translation.reshape(3, 1)


class GateDetectorNode(Node):
    def __init__(self):
        super().__init__('gate_detector_node')
        
        self.declare_parameter('camera_topic', '/cam0/image_rgb')
        self.declare_parameter('output_topic', '/aigp_state_estimator/set_pose')
        self.declare_parameter('track_path', '')
        self.declare_parameter('max_rate_hz', 30.0)
        self.declare_parameter('use_orange_detection', True)
        # Nominal documentation says 20 degrees. VQ1 ground truth plus gate
        # centre projection calibrates build 3390's effective optical tilt to
        # 17.3 degrees (the value that removes a 1.1 m vertical pose bias).
        self.declare_parameter('camera_up_tilt_deg', 17.3)
        self.declare_parameter('state_topic', '/drone_state_estimated')
        self.declare_parameter('require_state_consistency', True)
        self.declare_parameter('max_position_innovation_m', 3.0)
        self.declare_parameter('max_reprojection_error_px', 2.0)
        
        self.use_orange_detection = self.get_parameter('use_orange_detection').value
        self.camera_up_tilt_deg = float(self.get_parameter('camera_up_tilt_deg').value)
        self.require_state_consistency = bool(
            self.get_parameter('require_state_consistency').value
        )
        self.max_position_innovation_m = float(
            self.get_parameter('max_position_innovation_m').value
        )
        
        # Load track YAML
        track_path = self.get_parameter('track_path').value
        if not track_path or not os.path.exists(track_path):
            local_track = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'rl', 'config', 'aigp_competition_track.yaml'))
            if os.path.exists(local_track):
                track_path = local_track
            else:
                from ament_index_python.packages import get_package_share_directory
                track_path = os.path.join(
                    get_package_share_directory('rl'),
                    'config', 'aigp_competition_track.yaml')
        
        self.get_logger().info(f'Loading track: {track_path}')
        self.gates = self.load_gates_yaml(track_path)
        
        self.bridge = CvBridge()
        self.last_process_time = None
        self.min_period_ns = int(1e9 / self.get_parameter('max_rate_hz').value)
        self.active_gate_index = 0
        self.last_candidate_corners = None
        self.last_pose_solution = None
        self.last_solution_gate_index = None
        self.last_detection_time_ns = None
        self.latest_state_position_ned = None
        self.latest_state_rotation_ned_from_frd = None
        self.temporal_prior_timeout_ns = int(0.5e9)
        self.min_contour_area = 100.0
        self.max_reprojection_error_px = float(
            self.get_parameter('max_reprojection_error_px').value
        )
        self.min_pnp_distance_m = 0.20
        self.max_pnp_distance_m = 60.0
        
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.camera_callback,
            sensor_qos)
        
        self.active_gate_sub = self.create_subscription(
            Int32,
            '/aigp/active_gate_index',
            self.active_gate_callback,
            10)

        self.state_sub = self.create_subscription(
            Float64MultiArray,
            self.get_parameter('state_topic').value,
            self.state_callback,
            10)
        
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('output_topic').value,
            10)
            
        # Physical Gate Dimensions (Outer: 2.7m x 2.7m)
        # In PnP gate frame: X right, Y down, Z forward
        half_w = 2.7 / 2.0  # 1.35
        self.gate_points_3d = np.array([
            [-half_w, -half_w, 0.0],  # Top-Left
            [ half_w, -half_w, 0.0],  # Top-Right
            [ half_w,  half_w, 0.0],  # Bottom-Right
            [-half_w,  half_w, 0.0]   # Bottom-Left
        ], dtype=np.float32)

        # Camera intrinsics (640x360)
        self.camera_matrix = np.array([
            [320.0, 0.0, 320.0],
            [0.0, 320.0, 180.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros(5, dtype=np.float32)

    def extract_gate_corners(self, contour):
        """Extract 4 corner points (TL, TR, BR, BL) from a contour."""
        peri = cv2.arcLength(contour, True)
        for eps_factor in [0.03, 0.02, 0.04, 0.015, 0.05, 0.06, 0.08, 0.01]:
            approx = cv2.approxPolyDP(contour, eps_factor * peri, True)
            if len(approx) == 4:
                return self.order_corners(approx.reshape(4, 2).astype(np.float32))

        hull = cv2.convexHull(contour).reshape(-1, 2)
        if len(hull) >= 4:
            M = cv2.moments(hull)
            if M['m00'] > 0:
                cx = M['m10'] / M['m00']
                cy = M['m01'] / M['m00']
                pts = hull.astype(np.float32)
                tl = pts[np.argmin((pts[:, 0] - cx) + (pts[:, 1] - cy))]
                tr = pts[np.argmax((pts[:, 0] - cx) - (pts[:, 1] - cy))]
                br = pts[np.argmax((pts[:, 0] - cx) + (pts[:, 1] - cy))]
                bl = pts[np.argmin((pts[:, 0] - cx) - (pts[:, 1] - cy))]
                return np.array([tl, tr, br, bl], dtype=np.float32)

        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect).astype(np.float32)
        return self.order_corners(box)

    @staticmethod
    def _quad_geometry(corners):
        """Return (valid, quality, area, span) for an ordered quadrilateral."""
        corners = np.asarray(corners, dtype=np.float32).reshape(4, 2)
        contour = corners.reshape(-1, 1, 2)
        area = abs(float(cv2.contourArea(contour)))
        if area < 1.0 or not cv2.isContourConvex(contour):
            return False, 0.0, area, 0.0

        edges = np.roll(corners, -1, axis=0) - corners
        lengths = np.linalg.norm(edges, axis=1)
        if np.min(lengths) < 8.0:
            return False, 0.0, area, float(np.max(lengths))

        width = 0.5 * (lengths[0] + lengths[2])
        height = 0.5 * (lengths[1] + lengths[3])
        aspect = width / max(height, 1e-6)
        # Perspective can make the projected square non-square, so this gate is
        # deliberately permissive while rejecting obvious orange streaks.
        if not 0.25 <= aspect <= 4.0:
            return False, 0.0, area, float(max(width, height))

        adjacent_cosines = []
        for i in range(4):
            a = -edges[(i - 1) % 4] / lengths[(i - 1) % 4]
            b = edges[i] / lengths[i]
            adjacent_cosines.append(abs(float(np.dot(a, b))))
        angle_quality = np.clip(1.0 - np.mean(adjacent_cosines), 0.0, 1.0)
        opposite_quality = min(lengths[0], lengths[2]) / max(lengths[0], lengths[2])
        opposite_quality *= min(lengths[1], lengths[3]) / max(lengths[1], lengths[3])
        aspect_quality = np.exp(-0.35 * abs(np.log(aspect)))
        quality = float(0.45 * angle_quality + 0.30 * opposite_quality + 0.25 * aspect_quality)
        return True, quality, area, float(max(width, height))

    def score_gate_candidate(self, contour, image_shape, has_inner_contour=False):
        """Score a contour without assuming the largest orange blob is the gate."""
        contour_area = float(cv2.contourArea(contour))
        if contour_area < self.min_contour_area:
            return None
        corners = self.extract_gate_corners(contour)
        if self.last_candidate_corners is not None:
            previous = np.asarray(self.last_candidate_corners, dtype=np.float32)
            # Preserve the physical corner identities through image roll and
            # through the top-left heuristic's discontinuity at +/- 45 degrees.
            alternatives = [np.roll(corners, shift, axis=0) for shift in range(4)]
            corners = min(
                alternatives,
                key=lambda value: np.mean(np.linalg.norm(value - previous, axis=1)))
        valid, geometry_quality, quad_area, _ = self._quad_geometry(corners)
        if not valid:
            return None

        height, width = image_shape[:2]
        image_area = float(max(height * width, 1))
        area_quality = np.clip(quad_area / (0.12 * image_area), 0.0, 1.0)
        contour_fill = np.clip(contour_area / max(quad_area, 1.0), 0.0, 1.0)
        score = 1.8 * geometry_quality + 0.35 * area_quality + 0.15 * contour_fill
        if has_inner_contour:
            score += 0.30  # A closed orange frame normally has a visible hole.

        if self.last_candidate_corners is not None:
            old = np.asarray(self.last_candidate_corners, dtype=np.float32)
            displacement = float(np.mean(np.linalg.norm(corners - old, axis=1)))
            diagonal = float(np.hypot(width, height))
            continuity = np.exp(-4.0 * displacement / max(diagonal, 1.0))
            score += 0.9 * continuity
        return {'score': float(score), 'corners': corners, 'area': contour_area}

    def select_gate_candidate(self, mask):
        """Select a contour whose inferred pose agrees with the inertial state."""
        result = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours, hierarchy = result[-2], result[-1]
        if not contours:
            return None
        hierarchy = hierarchy[0] if hierarchy is not None else None
        candidates = []
        for i, contour in enumerate(contours):
            # Inner contours describe the hole; their parent is the outer frame.
            if hierarchy is not None and hierarchy[i][3] >= 0:
                continue
            has_inner = bool(hierarchy is not None and hierarchy[i][2] >= 0)
            candidate = self.score_gate_candidate(contour, mask.shape, has_inner)
            if candidate is not None:
                candidates.append(candidate)
        if not candidates:
            return None

        if not self.require_state_consistency:
            return max(candidates, key=lambda candidate: candidate['score'])
        if self.latest_state_position_ned is None:
            return None

        plausible = []
        for candidate in candidates:
            solution = self.solve_gate_pose_with_state_attitude(candidate['corners'])
            if solution is None:
                continue
            position_ned, _ = self.solution_to_body_pose_ned(solution)
            innovation = float(np.linalg.norm(
                position_ned - self.latest_state_position_ned
            ))
            if innovation > self.max_position_innovation_m:
                continue
            candidate['position_innovation_m'] = innovation
            candidate['pose_solution'] = solution
            # Prefer agreement with the inertial prediction when multiple
            # similarly gate-shaped orange contours remain.
            candidate['score'] += np.exp(
                -innovation / max(self.max_position_innovation_m, 1.0e-6)
            )
            plausible.append(candidate)
        return max(plausible, key=lambda candidate: candidate['score']) if plausible else None

    def solution_to_body_pose_ned(self, solution):
        """Convert a gate-to-camera PnP solution to body pose in NED/FRD."""
        idx = int(np.clip(self.active_gate_index, 0, len(self.gates) - 1))
        gate_pos_w = self.gates[idx]['position']
        rotation_w_gate = self.gates[idx]['rotation'].as_matrix()
        body_position_nwu, rotation_nwu_from_body = pnp_gate_pose_to_body_nwu(
            solution['rvec'],
            solution['tvec'],
            gate_pos_w,
            rotation_w_gate,
            camera_up_tilt_deg=self.camera_up_tilt_deg,
        )
        frame_flip = np.array([1.0, -1.0, -1.0])
        position_ned = frame_flip * body_position_nwu
        rotation_ned_from_frd = (
            np.diag(frame_flip)
            @ rotation_nwu_from_body
            @ np.diag(frame_flip)
        )
        return position_ned, rotation_ned_from_frd

    def solve_gate_pose_with_state_attitude(self, image_corners):
        """Use inertial attitude to resolve distant planar-gate ambiguity."""
        if self.latest_state_rotation_ned_from_frd is None:
            return None
        frame_flip = np.diag([1.0, -1.0, -1.0])
        rotation_nwu_from_body = (
            frame_flip
            @ self.latest_state_rotation_ned_from_frd
            @ frame_flip
        )
        rotation_nwu_from_camera = (
            rotation_nwu_from_body
            @ camera_optical_to_body_flu(self.camera_up_tilt_deg)
        )
        idx = int(np.clip(self.active_gate_index, 0, len(self.gates) - 1))
        rotation_nwu_from_gate = self.gates[idx]['rotation'].as_matrix()
        rotation_nwu_from_pnp_gate = (
            rotation_nwu_from_gate @ R_TRACK_FROM_PNP_GATE
        )
        rotation_camera_from_pnp_gate = (
            rotation_nwu_from_camera.T @ rotation_nwu_from_pnp_gate
        )
        rvec, _ = cv2.Rodrigues(rotation_camera_from_pnp_gate)
        tvec = translation_with_known_rotation(
            self.gate_points_3d,
            image_corners,
            self.camera_matrix,
            rotation_camera_from_pnp_gate,
        )
        return self._pose_candidate(rvec, tvec, image_corners)

    @staticmethod
    def refine_corners(gray, corners):
        """Refine contour-derived corners, retaining the input on unstable fits."""
        gray = np.asarray(gray)
        original = np.asarray(corners, dtype=np.float32).reshape(4, 2)
        height, width = gray.shape[:2]
        if (np.any(original[:, 0] < 4) or np.any(original[:, 0] >= width - 4)
                or np.any(original[:, 1] < 4) or np.any(original[:, 1] >= height - 4)):
            return original.copy()
        refined = original.reshape(-1, 1, 2).copy()
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        try:
            cv2.cornerSubPix(gray, refined, (5, 5), (-1, -1), criteria)
        except cv2.error:
            return original.copy()
        refined = refined.reshape(4, 2)
        # CornerSubPix can attach to a nearby texture edge. Do not let refinement
        # turn a valid contour fit into a discontinuous measurement.
        if np.max(np.linalg.norm(refined - original, axis=1)) > 8.0:
            return original.copy()
        valid, _, _, _ = GateDetectorNode._quad_geometry(refined)
        return refined.astype(np.float32) if valid else original.copy()

    def _pose_candidate(self, rvec, tvec, image_corners):
        rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
        tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
        if not np.all(np.isfinite(rvec)) or not np.all(np.isfinite(tvec)):
            return None
        rotation, _ = cv2.Rodrigues(rvec)
        points_cam = (rotation @ self.gate_points_3d.astype(np.float64).T + tvec).T
        distance = float(np.linalg.norm(tvec))
        if (np.min(points_cam[:, 2]) <= 0.05
                or not self.min_pnp_distance_m <= distance <= self.max_pnp_distance_m):
            return None
        projected, _ = cv2.projectPoints(
            self.gate_points_3d, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        residuals = projected.reshape(4, 2) - np.asarray(image_corners).reshape(4, 2)
        rms = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))
        if not np.isfinite(rms) or rms > self.max_reprojection_error_px:
            return None

        score = rms
        if (self.last_pose_solution is not None
                and self.last_solution_gate_index == self.active_gate_index):
            previous_rvec, previous_tvec = self.last_pose_solution
            translation_scale = max(float(np.linalg.norm(previous_tvec)), 1.0)
            previous_translation = np.asarray(previous_tvec).reshape(3)
            translation_jump = float(np.linalg.norm(tvec.reshape(3) - previous_translation))
            score += 2.0 * translation_jump / translation_scale
            score += 0.75 * _rotation_distance(rvec, previous_rvec)
        return {'rvec': rvec, 'tvec': tvec, 'reprojection_rms': rms, 'score': score}

    def solve_gate_pose(self, image_corners):
        """Resolve planar PnP ambiguity using cheirality, error, and pose history."""
        image_corners = np.asarray(image_corners, dtype=np.float32).reshape(4, 2)
        raw_solutions = []
        # IPPE returns both planar solutions. Four corner correspondences do not
        # provide redundancy for RANSAC, so explicit ambiguity handling is more
        # appropriate here than a nominal four-point RANSAC call.
        if hasattr(cv2, 'SOLVEPNP_IPPE'):
            try:
                result = cv2.solvePnPGeneric(
                    self.gate_points_3d, image_corners, self.camera_matrix,
                    self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE)
                if result[0]:
                    raw_solutions.extend(zip(result[1], result[2]))
            except cv2.error:
                pass
        candidates = []
        for rvec, tvec in raw_solutions:
            candidate = self._pose_candidate(rvec, tvec, image_corners)
            if candidate is not None:
                candidates.append(candidate)
        # Some OpenCV/IPPE versions return algebraic candidates that both fail
        # cheirality near a fronto-parallel singularity. Let the iterative solver
        # recover only when no physically acceptable IPPE result survived.
        if not candidates:
            try:
                success, rvec, tvec = cv2.solvePnP(
                    self.gate_points_3d, image_corners, self.camera_matrix,
                    self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
                if success:
                    candidate = self._pose_candidate(rvec, tvec, image_corners)
                    if candidate is not None:
                        candidates.append(candidate)
            except cv2.error:
                pass
        return min(candidates, key=lambda candidate: candidate['score']) if candidates else None

    def estimate_pose_covariance(self, image_corners, tvec, reprojection_rms):
        """Build a conservative 6x6 covariance from image-space pose quality."""
        corners = np.asarray(image_corners, dtype=np.float64).reshape(4, 2)
        edges = np.roll(corners, -1, axis=0) - corners
        gate_span_px = max(float(np.mean(np.linalg.norm(edges, axis=1))), 1.0)
        focal_px = max(float(0.5 * (self.camera_matrix[0, 0] + self.camera_matrix[1, 1])), 1.0)
        distance_m = max(float(np.linalg.norm(tvec)), self.min_pnp_distance_m)
        # A four-point planar fit can report nearly zero residual even though the
        # thresholded physical gate edge is only known to roughly a pixel.
        pixel_sigma = float(np.clip(max(reprojection_rms, 1.0), 1.0, 12.0))
        lateral_sigma = max(0.01, distance_m * pixel_sigma / focal_px)
        depth_sigma = max(0.02, 1.5 * distance_m * pixel_sigma / gate_span_px)
        angle_sigma = max(np.deg2rad(1.0), np.arctan2(1.5 * pixel_sigma, gate_span_px))

        # Depth direction depends on gate attitude. Isotropic use of its larger
        # uncertainty is conservative for the world-frame message consumed by
        # the estimator and avoids claiming accuracy along the wrong axis.
        position_variance = float(max(lateral_sigma, depth_sigma) ** 2)
        attitude_variance = float(angle_sigma ** 2)
        covariance = np.zeros((6, 6), dtype=np.float64)
        covariance[0:3, 0:3] = np.eye(3) * position_variance
        covariance[3:6, 3:6] = np.eye(3) * attitude_variance
        return covariance

    def order_corners(self, corners):
        """Sort corners: Top-Left, Top-Right, Bottom-Right, Bottom-Left."""
        corners = np.asarray(corners, dtype=np.float32).reshape(4, 2)
        center = np.mean(corners, axis=0)
        angles = np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0])
        ordered = corners[np.argsort(angles)]
        # Image coordinates have Y down, so ascending image-space polar angle is
        # TL, TR, BR, BL after choosing the upper-left cyclic starting point.
        start = int(np.argmin(ordered[:, 0] + ordered[:, 1]))
        return np.roll(ordered, -start, axis=0).astype(np.float32)

    def load_gates_yaml(self, path):
        import yaml
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        gates = []
        for idx, entry in enumerate(data['gate_track']):
            pos = np.array(entry['position'], dtype=np.float64)
            rot_rpy = entry['rotation']
            r = R.from_euler("xyz", [rot_rpy[0], rot_rpy[1], -rot_rpy[2]], degrees=True)
            gates.append({'position': pos, 'rotation': r})
        return gates

    def active_gate_callback(self, msg):
        new_index = int(msg.data)
        if new_index != self.active_gate_index:
            self.last_candidate_corners = None
            self.last_pose_solution = None
            self.last_solution_gate_index = None
            self.last_detection_time_ns = None
        self.active_gate_index = new_index

    def state_callback(self, msg):
        if len(msg.data) >= 10:
            position = np.asarray(msg.data[:3], dtype=np.float64)
            quaternion_wxyz = np.asarray(msg.data[6:10], dtype=np.float64)
            quaternion_norm = np.linalg.norm(quaternion_wxyz)
            if np.all(np.isfinite(position)) and quaternion_norm > 1.0e-9:
                self.latest_state_position_ned = position
                quaternion_wxyz /= quaternion_norm
                self.latest_state_rotation_ned_from_frd = R.from_quat([
                    quaternion_wxyz[1],
                    quaternion_wxyz[2],
                    quaternion_wxyz[3],
                    quaternion_wxyz[0],
                ]).as_matrix()

    def camera_callback(self, msg):
        now = self.get_clock().now()
        if self.last_process_time is not None:
            elapsed_ns = (now - self.last_process_time).nanoseconds
            if elapsed_ns < self.min_period_ns:
                return
        self.last_process_time = now

        if self.last_detection_time_ns is not None:
            prior_age_ns = now.nanoseconds - self.last_detection_time_ns
            if prior_age_ns < 0 or prior_age_ns > self.temporal_prior_timeout_ns:
                self.last_candidate_corners = None
                self.last_pose_solution = None
                self.last_solution_gate_index = None
                self.last_detection_time_ns = None

        # Convert image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        
        refined_corners = None
        if self.use_orange_detection:
            # Color-thresholding for bright neon orange (with HSV hue wrap)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
            lower_orange1 = np.array([0, 90, 70], dtype=np.uint8)
            upper_orange1 = np.array([25, 255, 255], dtype=np.uint8)
            lower_orange2 = np.array([170, 90, 70], dtype=np.uint8)
            upper_orange2 = np.array([180, 255, 255], dtype=np.uint8)
            mask1 = cv2.inRange(hsv, lower_orange1, upper_orange1)
            mask2 = cv2.inRange(hsv, lower_orange2, upper_orange2)
            mask = cv2.bitwise_or(mask1, mask2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            candidate = self.select_gate_candidate(mask)
            if candidate is not None:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
                refined_corners = self.refine_corners(gray, candidate['corners'])
                self.get_logger().info(
                    f"Selected orange gate candidate: area={candidate['area']:.1f}, "
                    f"score={candidate['score']:.2f}, "
                    f"position_innovation={candidate.get('position_innovation_m', -1.0):.2f}m",
                    throttle_duration_sec=1.0)

        if refined_corners is not None:
            if self.require_state_consistency:
                solution = self.solve_gate_pose_with_state_attitude(refined_corners)
            else:
                solution = self.solve_gate_pose(refined_corners)
            if solution is not None:
                rvec = solution['rvec']
                tvec = solution['tvec']
                idx = int(np.clip(self.active_gate_index, 0, len(self.gates) - 1))
                pos_ned, rotation_ned_from_frd = self.solution_to_body_pose_ned(
                    solution
                )
                if (
                    self.require_state_consistency
                    and self.latest_state_position_ned is not None
                ):
                    innovation = float(np.linalg.norm(
                        pos_ned - self.latest_state_position_ned
                    ))
                    if innovation > self.max_position_innovation_m:
                        # Subpixel refinement is poorly conditioned for a small,
                        # distant planar gate. Fall back to the contour-corner
                        # solution that already passed inertial consistency.
                        raw_solution = candidate.get('pose_solution')
                        if raw_solution is None:
                            return
                        solution = raw_solution
                        refined_corners = candidate['corners']
                        rvec = solution['rvec']
                        tvec = solution['tvec']
                        pos_ned, rotation_ned_from_frd = (
                            self.solution_to_body_pose_ned(solution)
                        )
                        innovation = float(np.linalg.norm(
                            pos_ned - self.latest_state_position_ned
                        ))
                q_ned = R.from_matrix(rotation_ned_from_frd).as_quat()
                
                # Publish Pose message in NED/FRD for the estimator
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header.stamp = msg.header.stamp
                pose_msg.header.frame_id = "world"
                pose_msg.pose.pose.position.x = float(pos_ned[0])
                pose_msg.pose.pose.position.y = float(pos_ned[1])
                pose_msg.pose.pose.position.z = float(pos_ned[2])
                pose_msg.pose.pose.orientation.x = float(q_ned[0])
                pose_msg.pose.pose.orientation.y = float(q_ned[1])
                pose_msg.pose.pose.orientation.z = float(q_ned[2])
                pose_msg.pose.pose.orientation.w = float(q_ned[3])
                pose_msg.pose.covariance = self.estimate_pose_covariance(
                    refined_corners, tvec, solution['reprojection_rms']).reshape(-1).tolist()
                
                self.publisher.publish(pose_msg)
                self.last_candidate_corners = refined_corners.copy()
                self.last_pose_solution = (rvec.copy(), tvec.copy())
                self.last_solution_gate_index = self.active_gate_index
                self.last_detection_time_ns = now.nanoseconds
                self.get_logger().info(
                    f'Published gate-corrected pose for gate {idx}: '
                    f'pos=[{pos_ned[0]:.3f}, {pos_ned[1]:.3f}, {pos_ned[2]:.3f}], '
                    f'reprojection_rms={solution["reprojection_rms"]:.2f}px',
                    throttle_duration_sec=1.0,
                )
            else:
                self.get_logger().warning(
                    'Rejected gate pose: no cheirality-valid low-error PnP solution',
                    throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = GateDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

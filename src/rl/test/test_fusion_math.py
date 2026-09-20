import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R

from rl.aigp_state_estimator_runtime import (
    ImuEstimatorState,
    integrate_imu_state,
    fuse_vision_pose,
)


class TestStateFusionMath(unittest.TestCase):
    def test_zero_state(self):
        state = ImuEstimatorState.zero()
        np.testing.assert_array_equal(state.position_ned, [0.0, 0.0, 0.0])
        np.testing.assert_array_equal(state.velocity_ned, [0.0, 0.0, 0.0])
        self.assertEqual(len(state.as_policy_state()), 13)

    def test_imu_integration(self):
        state = ImuEstimatorState.zero()
        accel_body = np.array([0.0, 0.0, -9.80665])  # Hovering in specific force
        gyro_body = np.array([0.0, 0.0, 0.0])
        
        next_state = integrate_imu_state(
            state, accel_body, gyro_body, dt_s=0.1, accel_is_specific_force=True
        )
        # Specific force + gravity [0,0,9.80665] = 0 acceleration -> velocity stay 0
        np.testing.assert_allclose(next_state.velocity_ned, [0.0, 0.0, 0.0], atol=1e-5)
        np.testing.assert_allclose(next_state.position_ned, [0.0, 0.0, 0.0], atol=1e-5)

    def test_vision_fusion_blend(self):
        state = ImuEstimatorState.zero()
        state.position_ned = np.array([10.0, 0.0, -2.0])
        state.velocity_ned = np.array([1.0, 0.0, 0.0])
        
        pos_vis = np.array([12.0, 0.5, -2.0])
        rot_vis = R.from_euler("z", 10.0, degrees=True)
        
        fused = fuse_vision_pose(
            state,
            pos_vis_ned=pos_vis,
            rot_vis_ned_from_frd=rot_vis,
            dt_vis_s=0.1,
            alpha_pos=0.5,
            alpha_rot=0.5,
            alpha_vel=0.5,
        )
        
        # Position should blend halfway between 10.0 and 12.0 -> 11.0
        self.assertAlmostEqual(fused.position_ned[0], 11.0)
        self.assertAlmostEqual(fused.position_ned[1], 0.25)
        
        # Velocity update derived from delta pos (12-10)/0.1 = 20.0, blended with 1.0 -> 10.5
        self.assertAlmostEqual(fused.velocity_ned[0], 10.5)

    def test_outlier_rejection(self):
        state = ImuEstimatorState.zero()
        state.position_ned = np.array([5.0, 0.0, 0.0])
        
        # Spurious detection 100 meters away
        pos_vis_spurious = np.array([105.0, 0.0, 0.0])
        rot_vis = R.identity()
        
        fused = fuse_vision_pose(
            state,
            pos_vis_ned=pos_vis_spurious,
            rot_vis_ned_from_frd=rot_vis,
            max_pos_err=10.0,
        )
        
        # Should reject outlier and maintain current position
        np.testing.assert_array_equal(fused.position_ned, [5.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()

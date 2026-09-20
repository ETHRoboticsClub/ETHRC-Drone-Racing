#!/usr/bin/env python3
"""Republish Vicon ``geometry_msgs/PoseStamped`` as
``nav_msgs/Odometry`` on the same topic the EKF normally consumes from
OpenVINS in production (``/ov_msckf/odomimu``).

Why this exists
---------------
We want to validate the production EKF code path without depending on
OpenVINS, which we know diverges on the 20260416_175153 bag because of
unresolved IMU↔camera calibration. Vicon at 100 Hz with sub-mm noise
floor is a perfect VIO stand-in — feeding it on the EKF's production
input topic exercises the *exact* odom0 path that runs during flight,
i.e. the same code path that mock_publisher's tests exercised
successfully during the EKF refactor.

Output Odometry layout matches what OpenVINS would publish:
- header.frame_id = "world"        (matches rl_ekf.yaml ``world_frame``)
- child_frame_id  = "base_link"    (matches rl_ekf.yaml ``base_link_frame``)
- pose.pose       = Vicon pose, copied verbatim
- pose.covariance = small diagonal (Vicon's actual sub-mm noise + a
                    safety pad to keep the filter numerically tame)
- twist           = zero           (we don't have it; relying on the
                    EKF's motion model + high twist covariance to
                    effectively ignore this dimension)

Usage
-----
    python3 scripts/perception/replay/vicon_to_odometry.py \\
        --input /vicon/zephyrus/zephyrus \\
        --output /ov_msckf/odomimu

Replay invocation needs no ``--remap`` for /cam0/camera_info because
this launch doesn't run gate_detection or the camera_info_republisher
either.
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


# Pose covariance: 5 cm position, 2° orientation. Vicon is much tighter
# than this — but conservative values keep the EKF state covariance from
# shrinking to numerically-ill-conditioned territory under 100 Hz updates.
DEFAULT_POS_VARIANCE = 2.5e-3       # σ_pos = 50 mm
DEFAULT_ORI_VARIANCE = 1.2e-3       # σ_ori ≈ 2°

# Twist covariance: large diagonal so the EKF effectively ignores the
# zero-twist values we publish. We don't have velocity from Vicon (no
# numerical differentiation here, kept simple). The EKF will rely on its
# motion model to evolve velocity between Vicon updates.
DEFAULT_LIN_VEL_VARIANCE = 1.0      # σ_v   = 1 m/s — looser than any real motion
DEFAULT_ANG_VEL_VARIANCE = 1.0      # σ_ω   = 1 rad/s


def _diag_cov6(diag6: list[float]) -> list[float]:
    cov = [0.0] * 36
    for i, v in enumerate(diag6):
        cov[7 * i] = v   # diagonal of a 6×6 row-major
    return cov


class ViconToOdometry(Node):
    def __init__(
        self,
        in_topic: str,
        out_topic: str,
        frame_id: str,
        child_frame_id: str,
        pose_var: float,
        ori_var: float,
        lin_vel_var: float,
        ang_vel_var: float,
    ) -> None:
        super().__init__("vicon_to_odometry")

        self._frame_id = frame_id
        self._child_frame_id = child_frame_id
        self._pose_cov = _diag_cov6(
            [pose_var, pose_var, pose_var, ori_var, ori_var, ori_var]
        )
        self._twist_cov = _diag_cov6(
            [lin_vel_var, lin_vel_var, lin_vel_var,
             ang_vel_var, ang_vel_var, ang_vel_var]
        )

        self.get_logger().info(
            f"Vicon-as-VIO adapter: {in_topic!r} -> {out_topic!r} "
            f"(Odometry, frame {frame_id!r}/{child_frame_id!r}, "
            f"σ_pos={pose_var**0.5*1000:.1f} mm, "
            f"σ_ori={ori_var**0.5*180/3.14159:.2f}°, "
            f"twist=0 with σ_v={lin_vel_var**0.5:.2f} m/s, "
            f"σ_ω={ang_vel_var**0.5:.2f} rad/s)"
        )

        self.pub = self.create_publisher(Odometry, out_topic, 50)
        self.sub = self.create_subscription(PoseStamped, in_topic, self._cb, 50)
        self._n = 0

    def _cb(self, msg: PoseStamped) -> None:
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame_id
        out.child_frame_id = self._child_frame_id
        out.pose.pose = msg.pose
        out.pose.covariance = self._pose_cov
        # twist defaults to zero — only set covariance
        out.twist.covariance = self._twist_cov
        self.pub.publish(out)
        self._n += 1
        if self._n in (1, 100) or self._n % 1000 == 0:
            self.get_logger().info(
                f"forwarded {self._n} samples (in_frame={msg.header.frame_id!r})")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", default="/vicon/zephyrus/zephyrus")
    ap.add_argument("--output", default="/ov_msckf/odomimu")
    ap.add_argument("--frame-id", default="world")
    ap.add_argument("--child-frame-id", default="base_link")
    ap.add_argument("--pos-variance", type=float, default=DEFAULT_POS_VARIANCE)
    ap.add_argument("--ori-variance", type=float, default=DEFAULT_ORI_VARIANCE)
    ap.add_argument("--lin-vel-variance", type=float, default=DEFAULT_LIN_VEL_VARIANCE)
    ap.add_argument("--ang-vel-variance", type=float, default=DEFAULT_ANG_VEL_VARIANCE)
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = ViconToOdometry(
        args.input, args.output, args.frame_id, args.child_frame_id,
        args.pos_variance, args.ori_variance,
        args.lin_vel_variance, args.ang_vel_variance,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

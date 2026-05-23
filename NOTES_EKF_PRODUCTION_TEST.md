# EKF production test runbook

Purpose: triage guide for the upcoming production-pipeline test of the 16-state quaternion EKF
(real VIO + IMU + gate detection — not the Vicon-substituted replay we tuned against).
Written 2026-05-08 after the iter-11 fix landed.

## TL;DR predicted outcomes

| Metric | Replay result (iter 11) | Expected in production | Confidence |
|---|---:|---|---|
| Orientation RMS | 1.71° (legacy 2.44°) | should still beat legacy | high — the Ax/Ay sign fix is real physics, IMU will reinforce |
| ATE X / Y | 3.3 / 4.4 cm (legacy 5.5 / 5.2) | similar or better | medium — depends on VIO twist quality |
| **ATE Z** | **1.5 cm (legacy 1.7)** | **likely regresses** | high — the Z-column kludge exploits replay-specific zero-twist input |

Expect the orientation and horizontal wins to transfer.  Expect Z to require a lever pull.

## Why Z may regress in production

The replay's Vicon adapter publishes `twist = 0` with `σ_v = 1 m/s`.  That softly anchors body
`Vx, Vy, Vz` near zero, so the kinematic Z-update terms `R20·Vx + R21·Vy + R22·Vz` are all ≈ 0
already.  Iter 11 zeros the first two contributions, but on the replay they were zero anyway.

Real VIO publishes real body-frame twist with much tighter σ.  During tilted forward flight
(e.g. 30° pitch, 5 m/s horizontal): true `Vx_body ≈ 4 m/s`, true `Vz_body ≈ 2.3 m/s`.  In a full
kinematic model, `R20·Vx + R22·Vz = 0` (altitude-hold identity).  Iter 11 dropped the first term
but kept the second, so the world-Z prediction picks up `R22·Vz ≈ 2 m/s` per step — phantom Z drift.

If the bag's drone is mostly altitude-holding tilted flight, this **will** show up as a
`+sin²(θ)`-correlated Z bias of comparable magnitude to what we eliminated on the Vicon replay.

## The three levers

All in `src/perception/ekf/robot_localization/src/ekf.cpp`:

### Lever A — un-zero the Z column (revert the iter-11 kludge)

Restore `R20`, `R21` cross-couplings into the Z position update.

[ekf.cpp lines ~313-320](src/perception/ekf/robot_localization/src/ekf.cpp#L313):
```cpp
transfer_function_(StateMemberZ, StateMemberVx) = R20 * delta_sec;
transfer_function_(StateMemberZ, StateMemberVy) = R21 * delta_sec;
transfer_function_(StateMemberZ, StateMemberAx) = R20 * dt2h;
transfer_function_(StateMemberZ, StateMemberAy) = R21 * dt2h;
```

And restore the matching `∂Z/∂Q` Jacobian rows at [ekf.cpp lines ~408-414](src/perception/ekf/robot_localization/src/ekf.cpp#L408):
```cpp
transfer_function_jacobian_(StateMemberZ, StateMemberQw) =
  ((-2 * qy) * vx + (2 * qx) * vy) * delta_sec +
  ((-2 * qy) * ax + (2 * qx) * ay) * dt2h;
transfer_function_jacobian_(StateMemberZ, StateMemberQx) =
  ((2 * qz) * vx + (2 * qw) * vy + (-4 * qx) * vz) * delta_sec +
  ((2 * qz) * ax + (2 * qw) * ay + (-4 * qx) * az) * dt2h;
transfer_function_jacobian_(StateMemberZ, StateMemberQy) =
  ((-2 * qw) * vx + (2 * qz) * vy + (-4 * qy) * vz) * delta_sec +
  ((-2 * qw) * ax + (2 * qz) * ay + (-4 * qy) * az) * dt2h;
transfer_function_jacobian_(StateMemberZ, StateMemberQz) =
  ((2 * qx) * vx + (2 * qy) * vy) * delta_sec +
  ((2 * qx) * ax + (2 * qy) * ay) * dt2h;
```

(Reference: `git show HEAD:src/perception/ekf/robot_localization/src/ekf.cpp` has the original block.)

**Effect:** restores full kinematic body→world position mapping.  In production with real VIO twist
this is mathematically correct — the cancellation `R20·Vx + R22·Vz ≈ 0` does its job.  Trade-off:
on this Vicon replay it would re-introduce the −g·sin²(θ) Z bias because the canceling Vz term is
zero, so don't expect this to also pass on the same replay.

### Lever B — re-enable IMU in `rl_ekf.yaml`

[`src/perception/ekf/ekf/config/rl_ekf.yaml:28-31`](src/perception/ekf/ekf/config/rl_ekf.yaml#L28) — uncomment:
```yaml
imu0: /imu/data
imu0_config: [false, false, false,  true, true, true, true,
              false, false, false,  true, true, true,  false, false, false]
imu0_remove_gravitational_acceleration: true
```

This pulls Ax/Ay/Az toward the accelerometer's measured kinematic acceleration, replacing the
hard-coded `T = mg` assumption with real thrust data.  With IMU on, body Az tracks reality, body
Vz integrates correctly, R22·Vz contribution to world Z is correct.

**Effect:** the gravity-coupling formulas in the predict step become less load-bearing — IMU
overrides them in correct().  If lever A breaks something, lever B is the principled second-order
fix.

### Lever C — process noise on body accel (filter_base.cpp)

[`src/perception/ekf/robot_localization/src/filter_base.cpp:127-129`](src/perception/ekf/robot_localization/src/filter_base.cpp#L127):
```cpp
process_noise_covariance_(StateMemberAx, StateMemberAx) = 0.01;
process_noise_covariance_(StateMemberAy, StateMemberAy) = 0.01;
process_noise_covariance_(StateMemberAz, StateMemberAz) = 0.015;
```

If predict-step accel disagrees with IMU/data (because T ≠ mg in racing), bumping these (×4 to
×10) lets the filter trust measurements over the model.  Already tried in replay (iter 2): made
things slightly worse with no IMU.  Could help in production with IMU on.

## Triage: what to do based on what you see

Read `out/eval_<bag>/metrics.csv` after the production replay.  Pull the row with `source==ekf_odometry`,
look at `ate_per_axis_rms_m` (x,y,z), `orientation_geodesic_rms_deg`, and the umeyama translation.

| Symptom | Diagnosis | Action |
|---|---|---|
| All 3 axes still beat legacy | iter 11 transferred fine | celebrate, no change needed |
| Z degrades, X/Y still beat legacy | exactly the predicted failure mode (zero-twist kludge fails on real twist) | **Lever A**: un-zero the Z column |
| Z degrades AND X/Y degrade | the Ax/Ay sign fix may also be conflicting with VIO twist; or process noise mistuned | **Lever A** first; if X/Y still bad, **Lever C** (bump Q_a) |
| Orientation regresses past legacy | unlikely, but means IMU is fighting the gravity-coupling formulas | **Lever B**: enable IMU and let it dominate |
| Z RMS very large (>10 cm) and grows over time | filter divergence — either Vicon innovation outliers being accepted or covariance underflow | check `print_diagnostics: true` in the EKF YAML — look at Mahalanobis rejections |
| ATE explodes during high-yaw segments | the quaternion sign-flip in `preparePose` may not be triggering on VIO orientation jumps | check `out/_launch_*.log` for `WARN: An input was not normalized` lines; investigate `ros_filter.cpp:3198-3210` |

### Quick diagnostic — which lever to pull first

After the production replay, run this to get the per-axis Z bias slope (the same metric we used to
identify the iter-3 → iter-11 mechanism):

```bash
docker compose exec dev bash -lc 'python3 -c "
import pandas as pd, numpy as np, scipy.stats as ss
new = pd.read_parquet(\"out/eval_<TAG>/aligned_<LABEL>_ekf_odometry.parquet\")
qx, qy = new.qx_gt.values, new.qy_gt.values
sin2 = 1 - (1 - 2*(qx*qx+qy*qy))**2
zerr = (new.z_est - new.z_gt).values
slope = np.polyfit(sin2, zerr, 1)[0]
r = ss.pearsonr(zerr, sin2).statistic
print(f\"slope={slope*100:+.2f} cm/unit, r={r:+.3f}, mean={zerr.mean()*100:+.2f}, rms={np.sqrt((zerr*zerr).mean())*100:.2f}\")
"'
```

- `slope ≈ 0, r ≈ 0`: no tilt-correlated bias.  Whatever Z RMS you have is just measurement noise; no lever needed (or a different bug).
- `slope strongly positive (+5 cm and up)`: classic "missing R20·Vx canceler" signature.  Pull **Lever A**.
- `slope strongly negative (−5 cm and down)`: same as the iter-3 bug we diagnosed.  Either the
  current build doesn't have iter-11's Z-column zero (rebuild with the fix), or you have a
  different mechanism — check Lever B.

## How to run the production replay

(Adjust `<bag-path>` and `<TAG>` for the actual bag.)

```bash
# inside dev container
docker compose exec dev bash

# 1. Make sure the production launch is what you want:
#    launch/perception_replay.launch.py uses real VIO + gate detection.
#    Keep IMU off initially to test Lever A in isolation; enable IMU as Lever B.

# 2. Run the replay
./scripts/perception/replay/_run_iter.sh <TAG>     # uses perception_replay_vio_subst.launch.py
# OR for full production with real VIO:
ros2 launch launch/perception.launch.py            # depends on /ov_msckf to be live
ros2 bag record -o out/replay_<TAG> /ekf/odometry /tf /tf_static /ov_msckf/odomimu /imu0
ros2 bag play <bag-path> --clock

# 3. Extract + evaluate
python3 scripts/perception/bag_analysis/bag_extract.py \
  --bag out/replay_<TAG> --out-dir out/extract_<TAG> --no-images
python3 scripts/perception/bag_analysis/eval_replay.py \
  --gt-extract out/extract_<gt-bag> \
  --est-extract out/extract_<TAG> --label new \
  --est-extract out/extract_subst_legacy --label legacy \
  --out-dir out/eval_<TAG>
cat out/eval_<TAG>/metrics.csv
```

## What stays unchanged regardless

These wins are real physics and will transfer cleanly — do not revert:

- Sign-corrected gravity coupling: `Ax = +2g(QwQy − QxQz)`, `Ay = −2g(QwQx + QyQz)` in
  [ekf.cpp lines 289-291](src/perception/ekf/robot_localization/src/ekf.cpp#L289) and
  [post-injection lines 480-485](src/perception/ekf/robot_localization/src/ekf.cpp#L480).
- The matching Velocity- and Acceleration-row Jacobian sign fixes.
- `Az = 0` (hover assumption — already in the code from `a1a6d25`, just unchanged).
- The submodule fix `78395d3` (Qz process noise + Qw=1 reset).
- The quaternion sign-flip in `preparePose` (already in `a1a6d25`).

## Long-term path if Lever A doesn't satisfy

The principled fix is to switch the EKF's velocity state to **world frame** (drone_model.py-style):

- Position update becomes `p_world += v_world·dt + ½·a_world·dt²` — diagonal, no R rotation.
- Body-frame twist messages get rotated to world inside `prepareTwist` before fusion.
- World-Z dynamics is fully decoupled from horizontal velocity.

This eliminates the asymmetric Z-column trick entirely.  Bigger refactor though — touches
prepareTwist, F, J, every position-row entry.  Don't do this unless the simpler levers fail.

## File reference

- Predict step: [src/ekf.cpp](src/perception/ekf/robot_localization/src/ekf.cpp)
- Process noise / initial cov: [src/filter_base.cpp:112-129](src/perception/ekf/robot_localization/src/filter_base.cpp#L112)
- Measurement preprocessing (incl. quaternion sign-flip): [src/ros_filter.cpp:3198-3210](src/perception/ekf/robot_localization/src/ros_filter.cpp#L3198)
- Sensor config: [src/perception/ekf/ekf/config/rl_ekf.yaml](src/perception/ekf/ekf/config/rl_ekf.yaml)
- Replay launch (Vicon-sub, what we tuned with): [launch/perception_replay_vio_subst.launch.py](launch/perception_replay_vio_subst.launch.py)
- Production launch: [launch/perception.launch.py](launch/perception.launch.py)
- Iteration helper: [scripts/perception/replay/_run_iter.sh](scripts/perception/replay/_run_iter.sh)
- Eval script: [scripts/perception/bag_analysis/eval_replay.py](scripts/perception/bag_analysis/eval_replay.py)

## Final replay metrics (reference baseline for tomorrow's comparison)

Bag `20260416_175153`, Vicon-substituted into `/ov_msckf/odomimu`, IMU off:

| | new (iter 11) | legacy | delta |
|---|---:|---:|---:|
| ATE RMS | 5.71 cm | 7.75 cm | −2.04 |
| ATE x | 3.31 cm | 5.52 cm | −2.21 |
| ATE y | 4.41 cm | 5.17 cm | −0.76 |
| ATE z | 1.49 cm | 1.69 cm | −0.20 |
| Orientation RMS | 1.71° | 2.44° | −0.73 |
| Orientation max | 60.7° | 82.2° | −21.5 |
| Z bias slope vs sin²(tilt) | +0.31 cm | −0.05 cm | (effectively zero) |

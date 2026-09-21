# EKF production test — runbook

Triage guide for the first run of the 16-state quaternion EKF on the real pipeline (OpenVINS + gate detection instead of the Vicon-substituted replay it was tuned on). Written 2026-05-08, updated 2026-05-27 when the merge decision was made.

**What shipped:** the correct-physics predict step, submodule commit `484337f` — full body→world Z-column coupling and `Az = +2g(Qx² + Qy²)`.
**What did not:** the replay-tuned Z kludge (`Az = 0`, Z column zeroed), commit `2308a32`, tag `replay-z-kludge`, branch `humble-devel`. It is Lever A below and is applied only if Z regresses on real VIO.

## Why the kludge stays out

The replay's Vicon adapter publishes zero twist with σ = 1 m/s, which anchors the body velocities near zero. The world-Z update `R20·Vx + R21·Vy + R22·Vz` is therefore ≈ 0 on the replay whether or not the first two terms are zeroed; the kludge only removed the `−g·sin²θ` drift that the gravity-coupled `Vx` state produced in the absence of real twist.

Real VIO publishes body twist with tight σ. In tilted level flight (30° pitch, 5 m/s) the true `Vx ≈ 4 m/s` and `Vz ≈ 2.3 m/s`, and `R20·Vx + R22·Vz = 0` is the altitude-hold identity. Zeroing `R20·Vx` while keeping `R22·Vz` would add ≈ 2 m/s of phantom vertical velocity per predict step — a `+sin²θ` Z bias of the same magnitude the kludge removed on the replay. The full Z column is kinematically consistent; the kludge is not.

## Expected outcome

| Metric | Replay (kludge variant) | Legacy 15-state | Expectation on real VIO |
| :--- | ---: | ---: | :--- |
| Orientation RMS | 1.71° | 2.44° | Transfers — the Ax/Ay sign fix is physics, and the IMU (if enabled) reinforces it |
| ATE x / y | 3.3 / 4.4 cm | 5.5 / 5.2 cm | Similar; depends on VIO twist quality |
| ATE z | 1.5 cm | 1.7 cm | Not measured for the shipped variant; watch for a tilt-correlated bias |

## Levers

All in `src/perception/ekf/robot_localization/` unless noted. Apply one at a time.

### A — Z kludge (`2308a32`)

Cherry-pick `2308a32` onto the submodule; the commit is exactly the `484337f → 2308a32` diff. By hand, in `src/ekf.cpp` around line 313:

```cpp
transfer_function_(StateMemberZ, StateMemberVx) = 0.0;
transfer_function_(StateMemberZ, StateMemberVy) = 0.0;
transfer_function_(StateMemberZ, StateMemberVz) = R22 * delta_sec;
transfer_function_(StateMemberZ, StateMemberAx) = 0.0;
transfer_function_(StateMemberZ, StateMemberAy) = 0.0;
transfer_function_(StateMemberZ, StateMemberAz) = R22 * dt2h;
```

plus `state_(StateMemberAz) = 0.0` (lines 291 and 492) and the reduced ∂Z/∂Q Jacobian (Qw/Qz rows zero; Qx/Qy rows keep only the `R22·Vz` term). Use only if the diagnostic below shows a negative tilt-correlated Z bias.

### B — fuse the IMU

`src/perception/ekf/ekf/config/rl_ekf.yaml`, lines 40–43, uncomment:

```yaml
imu0: /imu/data
imu0_config: [false, false, false,  true, true, true, true,
              false, false, false,  true, true, true,  false, false, false]
imu0_remove_gravitational_acceleration: true
```

The accelerometer then drives Ax/Ay/Az in `correct()`, replacing the `T = mg` assumption of the predict step. Requires the topic to exist (`/imu0` on hardware — adjust) and a `base_link`→`imu_link` transform.

### C — process noise on body acceleration

`src/filter_base.cpp`, lines 127–129:

```cpp
process_noise_covariance_(StateMemberAx, StateMemberAx) = 0.01;
process_noise_covariance_(StateMemberAy, StateMemberAy) = 0.01;
process_noise_covariance_(StateMemberAz, StateMemberAz) = 0.015;
```

Raising these by 4–10× lets measurements override the model when thrust differs from `mg`. Tried on the replay without IMU (iteration 2): slightly worse. May help with Lever B on.

## Triage

Read the `source == ekf_odometry` row of `out/eval_<tag>/metrics.csv`: `ate_per_axis_rms_m`, `orientation_geodesic_rms_deg`, Umeyama translation.

| Observation | Likely cause | Action |
| :--- | :--- | :--- |
| All three axes beat legacy | — | Nothing |
| Z worse, x/y fine | Z column interacting with real twist | Run the Z-bias diagnostic; Lever A if the slope is negative |
| Z and x/y worse | Process noise, or Ax/Ay fighting VIO twist | Lever A, then Lever C |
| Orientation worse than legacy | Gravity-coupling formulas fighting the IMU | Lever B |
| Z RMS > 10 cm and growing | Divergence: accepted outliers or covariance underflow | `print_diagnostics: true`, inspect Mahalanobis rejections |
| ATE jumps during fast yaw | Quaternion hemisphere flip not triggering | Look for `An input was not normalized` in the launch log; `src/ros_filter.cpp` ~3195–3210 |

### Z-bias diagnostic

Slope of the Z error against `sin²(tilt)`, the metric that identified the original mechanism:

```bash
python3 - <<'PY'
import pandas as pd, numpy as np, scipy.stats as ss
new = pd.read_parquet("out/eval_<TAG>/aligned_<LABEL>_ekf_odometry.parquet")
qx, qy = new.qx_gt.values, new.qy_gt.values
sin2 = 1 - (1 - 2*(qx*qx + qy*qy))**2
zerr = (new.z_est - new.z_gt).values
slope = np.polyfit(sin2, zerr, 1)[0]
r = ss.pearsonr(zerr, sin2).statistic
print(f"slope={slope*100:+.2f} cm/unit  r={r:+.3f}  mean={zerr.mean()*100:+.2f} cm  rms={np.sqrt((zerr*zerr).mean())*100:.2f} cm")
PY
```

| Result | Meaning | Lever |
| :--- | :--- | :--- |
| slope ≈ 0, r ≈ 0 | No tilt-correlated bias; remaining Z error is noise or a different bug | none |
| slope ≤ −5 cm | The `−g·sin²θ` drift from `R20·Vx` without a cancelling `R22·Vz` — what the kludge removes | A |
| slope ≥ +5 cm | Not the kludge's failure mode; suspect VIO twist frame/sign or process noise | B, then C |

## Running the replay

Inside the dev container (`docker compose exec dev bash`).

Vicon-substituted (the tuning configuration):

```bash
./scripts/perception/replay/_run_iter.sh <TAG>       # perception_replay_vio_subst.launch.py, output in out/
```

Real VIO + gate detection:

```bash
ros2 launch launch/perception_replay.launch.py       # publishes the world→global and base_link→imu TFs the EKF needs
ros2 bag record -o out/replay_<TAG> /ekf/odometry /tf /tf_static /ov_msckf/odomimu /imu0
ros2 bag play <bag> --clock
```

Do not use `launch/perception.launch.py` for this: it does not publish the static transforms and fails on its defaults (missing `pose1`, missing gate-detection checkpoint).

Evaluation:

```bash
python3 scripts/perception/bag_analysis/bag_extract.py --bag out/replay_<TAG> --out-dir out/extract_<TAG> --no-images
python3 scripts/perception/bag_analysis/eval_replay.py \
  --gt-extract out/extract_<gt-bag> \
  --est-extract out/extract_<TAG> --label new \
  --est-extract out/extract_subst_legacy --label legacy \
  --out-dir out/eval_<TAG>
cat out/eval_<TAG>/metrics.csv
```

`scripts/perception/bag_analysis/` is gitignored (`.gitignore:83`) and is not in a fresh clone. The scripts (`bag_extract.py`, `bag_inspect.py`, `bag_visualize.py`, `eval_replay.py`, `_common.py`) exist in the `ekf_merge` working copy; copy them in, or remove the ignore rule and commit them.

## Do not revert

- `Ax = +2g(QwQy − QxQz)`, `Ay = −2g(QwQx + QyQz)` — `src/ekf.cpp` lines 289–290 and 488 ff., with the matching velocity- and acceleration-row Jacobian signs.
- `Az = +2g(Qx² + Qy²)` (`484337f`).
- Isotropic quaternion process noise and identity seed (`78395d3`).
- Quaternion hemisphere flip in `preparePose` (`a1a6d25`).

## If the levers are not enough

Move the velocity state to the world frame, as in `src/mpc/mpc/drone_model.py`: the position row becomes `p += v·dt + ½a·dt²` with no rotation, body twist is rotated to world in `prepareTwist`, and world Z decouples from horizontal velocity. This removes the need for any Z-column special-casing but touches `prepareTwist`, F, J and every position-row entry. Not before the levers have been tried.

## Replay baseline (kludge variant)

Bag `20260416_175153`, Vicon substituted on `/ov_msckf/odomimu`, IMU off. "New" is `2308a32`; the shipped `484337f` matches it on orientation and x/y and has not been measured on z.

| | new (kludge) | legacy | Δ |
| :--- | ---: | ---: | ---: |
| ATE RMS | 5.71 cm | 7.75 cm | −2.04 |
| ATE x | 3.31 cm | 5.52 cm | −2.21 |
| ATE y | 4.41 cm | 5.17 cm | −0.76 |
| ATE z | 1.49 cm | 1.69 cm | −0.20 |
| Orientation RMS | 1.71° | 2.44° | −0.73 |
| Orientation max | 60.7° | 82.2° | −21.5 |
| Z-bias slope vs sin²(tilt) | +0.31 cm | −0.05 cm | ≈ 0 |

## Files

| Path | Content |
| :--- | :--- |
| `src/perception/ekf/robot_localization/src/ekf.cpp` | Predict step (physics at 289–291, Z column at 313–318, post-predict re-injection at 488) |
| `src/perception/ekf/robot_localization/src/filter_base.cpp` | Process and initial covariance defaults (112–129) |
| `src/perception/ekf/robot_localization/src/ros_filter.cpp` | Measurement preprocessing incl. hemisphere flip (~3195–3210) |
| `src/perception/ekf/ekf/config/rl_ekf.yaml` | Sensor config |
| `launch/perception_replay_vio_subst.launch.py` | Vicon-substituted replay |
| `launch/perception_replay.launch.py` | Real-VIO replay |
| `scripts/perception/replay/_run_iter.sh` | Iteration helper |
| `docs/docs/perception/ekf/technical.md` | Filter documentation |

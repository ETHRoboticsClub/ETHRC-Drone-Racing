# EKF production test runbook

Purpose: triage guide for the upcoming production-pipeline test of the 16-state quaternion EKF
(real VIO + IMU + gate detection — not the Vicon-substituted replay we tuned against).
Written 2026-05-08 after the iter-11 fix landed; **reframed after deciding to merge the
pre-kludge correct-physics commit (`484337f`) instead of the replay-tuned kludge (`2308a32`).**

> **Shipped model (as merged)**: the **correct physics** — full body→world Z-column coupling
> and `Az = +2g(Qx²+Qy²)`. The Z-zero kludge (`Az = 0`, zeroed Z-column) that we tuned on the
> zephyrus Vicon-substituted replay is **NOT shipped**; it is preserved at commit `2308a32`
> (tag `replay-z-kludge`, branch `humble-devel`) and is now an **opt-in lever** (Lever A below),
> to apply only if Z regresses on the real-VIO pipeline. Everything below is framed from the
> correct-physics default.

## TL;DR predicted outcomes

| Metric | Replay result (kludge variant) | Expected in production | Confidence |
|---|---:|---|---|
| Orientation RMS | 1.71° (legacy 2.44°) | should still beat legacy | high — the Ax/Ay sign fix is real physics, IMU will reinforce |
| ATE X / Y | 3.3 / 4.4 cm (legacy 5.5 / 5.2) | similar or better | medium — depends on VIO twist quality |
| **ATE Z** | **1.5 cm with kludge (legacy 1.7)** | **correct-physics default; revisit if it regresses** | the 1.5 cm was the *kludge* number; the shipped full-Z-column model is kinematically correct under real VIO twist |

Expect the orientation and horizontal wins to transfer.  Z is now handled by the correct
full-Z-column physics — apply Lever A (the kludge) only if real-VIO Z regresses.

## Why the kludge is replay-only (not shipped)

The replay's Vicon adapter publishes `twist = 0` with `σ_v = 1 m/s`.  That softly anchors body
`Vx, Vy, Vz` near zero, so the kinematic Z-update terms `R20·Vx + R21·Vy + R22·Vz` are all ≈ 0
already.  Iter 11 zeros the first two contributions, but on the replay they were zero anyway.

Real VIO publishes real body-frame twist with much tighter σ.  During tilted forward flight
(e.g. 30° pitch, 5 m/s horizontal): true `Vx_body ≈ 4 m/s`, true `Vz_body ≈ 2.3 m/s`.  In a full
kinematic model, `R20·Vx + R22·Vz = 0` (altitude-hold identity).  Iter 11 dropped the first term
but kept the second, so the world-Z prediction picks up `R22·Vz ≈ 2 m/s` per step — phantom Z drift.

That is why the shipped model keeps the full Z-column: on a real-VIO bag of altitude-holding
tilted flight, the kludge **would** inject a `+sin²(θ)`-correlated Z bias of comparable
magnitude to what it eliminated on the Vicon replay.

## The three levers

All in `src/perception/ekf/robot_localization/src/ekf.cpp`:

### Lever A — apply the Z-zero kludge (zero the Z column)

The shipped model has the **full** body→world Z-column coupling (correct physics, `484337f`).
If real-VIO Z shows a `+sin²(tilt)`-correlated bias, the replay-tuned kludge zeroes the
body-horizontal Z-column terms and pins `Az = 0`. It is preserved at commit `2308a32`
(tag `replay-z-kludge`); the cleanest way to apply it is to **cherry-pick that commit onto the
submodule** — that is exactly the `484337f → 2308a32` diff, so it is safer than hand-editing.

By hand, the Z-column becomes [ekf.cpp ~L313](src/perception/ekf/robot_localization/src/ekf.cpp#L313):
```cpp
transfer_function_(StateMemberZ, StateMemberVx) = 0.0;
transfer_function_(StateMemberZ, StateMemberVy) = 0.0;
transfer_function_(StateMemberZ, StateMemberVz) = R22 * delta_sec;
transfer_function_(StateMemberZ, StateMemberAx) = 0.0;
transfer_function_(StateMemberZ, StateMemberAy) = 0.0;
transfer_function_(StateMemberZ, StateMemberAz) = R22 * dt2h;
```
plus `state_(StateMemberAz) = 0.0` and the simplified `∂Z/∂Q` Jacobian (Qw/Qz rows → 0;
Qx/Qy rows keep only the `R22·Vz` term). See commit `2308a32` for the exact rows.

**Effect:** on the zero-twist Vicon replay this eliminates the −g·sin²(θ) Z bias. On real VIO
twist it is *not* kinematically correct — it drops the `R20·Vx + R22·Vz ≈ 0` altitude-hold
cancellation — so apply it only if the correct-physics default measurably regresses Z.

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
| All 3 axes still beat legacy | correct-physics transferred fine | celebrate, no change needed |
| Z degrades, X/Y still beat legacy | correct-physics Z-column may need the replay kludge under real twist (this is what the kludge addressed) | **Lever A**: apply the Z-zero kludge |
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
- `slope strongly positive (+5 cm and up)`: **not** the kludge's failure mode — the kludge
  targets a *negative* −g·sin²θ bias. Suspect VIO twist frame/sign or mistuned process noise;
  check **Lever B / Lever C**.
- `slope strongly negative (−5 cm and down)`: the −g·sin²θ drift from the uncompensated `R20·Vx`
  term — exactly what the kludge removes. Expected on zero-twist replay data; on real VIO twist
  it means the canceling `R22·Vz` isn't effective. Apply **Lever A** (the kludge).

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
- `Az = +2g(Qx² + Qy²)` (correct vertical gravity coupling, commit `484337f`). The `Az = 0` /
  zeroed-Z-column form is the dropped replay kludge — **not** shipped.
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

## Final replay metrics (kludge-variant baseline)

Bag `20260416_175153`, Vicon-substituted into `/ov_msckf/odomimu`, IMU off. **Note**: the
"new" column below is the **kludge variant** (`2308a32`), not the shipped correct-physics
model (`484337f`). The shipped model matches these on orientation and X/Y but will differ on
Z (it has no kludge), as discussed above.

| | new (kludge, iter 11) | legacy | delta |
|---|---:|---:|---:|
| ATE RMS | 5.71 cm | 7.75 cm | −2.04 |
| ATE x | 3.31 cm | 5.52 cm | −2.21 |
| ATE y | 4.41 cm | 5.17 cm | −0.76 |
| ATE z | 1.49 cm | 1.69 cm | −0.20 |
| Orientation RMS | 1.71° | 2.44° | −0.73 |
| Orientation max | 60.7° | 82.2° | −21.5 |
| Z bias slope vs sin²(tilt) | +0.31 cm | −0.05 cm | (effectively zero) |

# SITL Propulsion and Lift Calibration

This note summarizes the current SITL propulsion findings and the calibration targets for making the Gazebo rotor model more realistic.

## Current Situation

The SITL model uses the in-tree `gz::sim::systems::EthrcBetaflightSITLPlugin` from `src/sitl_gazebo_plugins` to apply thrust directly from Betaflight motor outputs.

That direct-thrust path was added because the previous `LiftDrag` setup did not produce reliable takeoff for this model. Do not enable both a calibrated direct-thrust model and a calibrated SDF lift model at the same time, or thrust will be double-counted.

## Physical Target

The drone thrust-to-weight ratio is approximately `4:1`.

The current model mass is:

```text
base_link mass = 0.893 kg
rotor mass     = 4 * 0.012 kg
total mass     = 0.941 kg
weight         = 0.941 * 9.80665 = 9.23 N
```

Calibration targets:

```text
max total thrust       ~= 36.9 N
max thrust per motor   ~= 9.23 N
hover total thrust     ~= 9.23 N
hover thrust per motor ~= 2.31 N
```

For a quadratic propeller thrust model:

```text
thrust = normalized_motor_output^2 * max_thrust
```

hover is expected near:

```text
normalized_motor_output = sqrt(1 / 4) = 0.5
```

## Propeller Geometry

The reported propeller is a Gemfan `4030`, but the standard prop naming convention and common Gemfan listings use `4030` to mean `4x3 inch`, not `5 inch`. A 5-inch, 3-inch-pitch prop would usually be named `5030`.

Before final calibration, confirm whether the actual prop is:

```text
4030 = 4.0 inch diameter, 3.0 inch pitch
5030 = 5.0 inch diameter, 3.0 inch pitch
```

Approximate disk areas:

```text
4 inch prop radius = 0.0508 m
4 inch disk area   = pi * 0.0508^2 = 0.0081 m^2

5 inch prop radius = 0.0635 m
5 inch disk area   = pi * 0.0635^2 = 0.0127 m^2
```

The current SDF uses rotor visual radius `0.06 m`, which is close to a 5-inch prop. If `LiftDrag` is reintroduced later, `<area>` should be treated as blade planform area or an empirically tuned effective area, not as the full disk area.

## Making `LiftDrag` More Accurate

`LiftDrag` is a wing-surface approximation. A propeller is better described by blade-element theory or by a thrust curve, so `LiftDrag` needs calibration against measured motor/prop data.

Recommended process:

1. Disable or remove `EthrcBetaflightSITLPlugin` from `sim/gazebo/models/drone_sitl/model.sdf`.
2. Run a single-rotor or fixed-throttle SITL test.
3. Log vertical acceleration and estimate generated thrust:
   ```text
   total_force = mass * (az + g)
   ```
4. Tune `LiftDrag` parameters until the model reaches the physical targets above.
5. Verify full manual throttle produces clear climb and hover throttle is plausible.

Important SDF parameters:

```xml
<area>...</area>
<cla>...</cla>
<cda>...</cda>
<a0>...</a0>
<alpha_stall>...</alpha_stall>
<cp>...</cp>
<forward>...</forward>
<upward>...</upward>
```

Key checks:

* `forward` and `upward` must produce lift in the correct direction for each rotor spin direction.
* Rotor joint velocity sign must match the aerodynamic sign assumptions.
* Rotor geometry should match the real prop diameter.
* The model should produce about `9.23 N` per motor at maximum output.

## Preferred Long-Term Model

For a racing quad, a motor thrust model is usually easier to calibrate than `LiftDrag`:

```text
thrust = motor_constant * omega^2
torque = moment_constant * thrust
```

Gazebo Harmonic includes `gz::sim::systems::MulticopterMotorModel`, which follows this style and may be a better long-term replacement for the SDF `LiftDrag` blocks. The in-tree `EthrcBetaflightSITLPlugin` can also be kept, but then the direct model should be the single source of truth and should use the measured `4:1` thrust-to-weight ratio.

## Verification Criteria

A propulsion model is acceptable when:

* manual full throttle lifts the drone decisively,
* hover is near the expected normalized motor output, approximately `0.5` for `4:1` TWR,
* the drone does not climb at minimum throttle after arming,
* Betaflight motor ordering and spin directions match the SDF,
* the RL policy receives consistent state signs and does not fight a frame mismatch.

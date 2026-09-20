# Boreas 4S Betaflight SITL Walkthrough

## Outcome

The control simulator now uses a shared Boreas 4S profile for Gazebo and MPC.
The plant includes first-order motor response, a configurable thrust curve,
reaction torque, body-frame drag, filtered RPM, and diagnostic wrenches. Both
`controller_type:=rl` and `controller_type:=mpc` now execute through Betaflight
SITL.

The profile deliberately remains marked `provisional`: no thrust-stand or
Vicon identification data was available during implementation.

## Key Files

- `config/airframes/boreas_4s.yaml`: shared physical profile and provenance.
- `src/sitl_gazebo_plugins/`: unit-tested actuator and Gazebo integration.
- `src/mpc/mpc/airframe.py`: MPC profile loader.
- `scripts/simulation/validate_boreas_model.py`: URDF/SDF/profile consistency.
- `scripts/simulation/validate_sitl_plant.py`: deterministic plant smoke test.
- `scripts/simulation/validate_sitl_controller.py`: RL/MPC full-stack test.
- `scripts/simulation/fit_boreas_propulsion.py`: thrust-stand curve fitting.

## Verification Commands

```bash
source scripts/source_sitl_env.sh

python3 scripts/simulation/validate_boreas_model.py
python3 scripts/simulation/validate_sitl_plant.py
python3 scripts/simulation/validate_sitl_controller.py rl
python3 scripts/simulation/validate_sitl_controller.py mpc
```

The controller validations are non-armed by default. Use `--arm` only when
motion and resets are acceptable.

Build and unit tests:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  sitl_gazebo_plugins interfaces mpc rl drone_control
colcon test --packages-select sitl_gazebo_plugins mpc drone_control
colcon test-result --verbose
```

Documentation:

```bash
cd docs
mkdocs build --strict
```

## Results on the Implementation Machine

- Boreas URDF/SDF/profile consistency: passed.
- C++ actuator and drag unit tests: 5 passed.
- Airframe and MPC frame-contract tests: passed.
- Airborne plant test: passed at a 1 ms physics step; measured real-time factor
  was approximately `0.67-0.81` under WSL, so normal racing worlds use 2 ms.
- RL full-stack contract: 201 finite controller commands in a two-second sample.
- MPC full-stack contract: 196 finite commands; ACADOS returned status 0 and
  loaded `0.8351086 kg` mass with `32.7585 N` maximum thrust.

The repository's package-wide `drone_control` flake8 and pydocstyle tests still
report numerous pre-existing errors across legacy launch and goal-client files.
Functional tests and checks scoped to the changed controller paths pass.

## Deferred Calibration

Before quantitative sim-to-real claims:

1. Record motor/propeller identity and thrust-stand RPM/thrust sweeps.
2. Fit and review the static curve with `fit_boreas_propulsion.py`.
3. Record held-out Vicon flights with commands, IMU, RPM, and battery voltage.
4. Fit motor time constants and body drag.
5. Replace provisional profile values and attach dataset identifiers.
6. Repeat armed RL and MPC trajectory tests and compare against held-out flights.

Camera realism, VIO noise, lighting randomization, propwash, ground effect,
battery sag, and thermal behavior remain out of scope for this controls-first
model.

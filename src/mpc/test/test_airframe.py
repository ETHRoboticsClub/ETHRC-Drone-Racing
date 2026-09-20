from pathlib import Path

import numpy as np

from mpc.airframe import load_boreas_airframe


ROOT = Path(__file__).resolve().parents[3]


def test_shared_boreas_profile_loads():
    profile = load_boreas_airframe(ROOT / "config/airframes/boreas_4s.yaml")
    assert np.isclose(profile.mass_kg, 0.8351086449771665)
    assert np.isclose(profile.maximum_thrust_n, 35.9315656)
    assert np.allclose(profile.thrust_curve_per_motor_n, [0.0, 0.0, 8.9828914])
    assert profile.calibration_status == "partially_measured"


def test_boreas_thrust_to_weight_uses_measured_static_maximum():
    profile = load_boreas_airframe(ROOT / "config/airframes/boreas_4s.yaml")
    ratio = profile.maximum_thrust_n / (profile.mass_kg * 9.80665)
    assert np.isclose(ratio, 4.3874530841, atol=1e-7)

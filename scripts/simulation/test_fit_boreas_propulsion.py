import numpy as np

from fit_boreas_propulsion import fit_curve


def test_fit_recovers_known_curve():
    rpm = np.linspace(1000.0, 30000.0, 30)
    normalized = rpm / 30000.0
    thrust = 0.4 * normalized + 7.8 * normalized**2
    result = fit_curve(rpm, thrust, 30000.0)
    assert np.allclose(result["thrust_curve_n"], [0.0, 0.4, 7.8])
    assert result["rmse_n"] < 1e-12

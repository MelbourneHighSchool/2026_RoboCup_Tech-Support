"""Polling controls reject invalid rates and report measured rather than requested Hz."""
from types import SimpleNamespace

import pytest

from calibration.polling import DEFAULT_RATES, PollingMeasurements, polling_rates


def test_rates_and_invalid_inputs():
    assert polling_rates({}) == DEFAULT_RATES
    assert polling_rates({"imu_hz": "400"})["imu_hz"] == 400
    for value in (0, -1, 401, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            polling_rates({"imu_hz": value})


def test_measurement_windows_use_counter_deltas_and_elapsed_time():
    now = [10.0]
    stats = {"motor_count": 100, "motor_wait_s": 0.1, "motor_work_s": 0.2,
             "motor_overruns": 4, "motor_max_wait_s": 0.01}
    controller = SimpleNamespace(timing_diagnostics=lambda: dict(stats))
    measurements = PollingMeasurements(controller, DEFAULT_RATES, lambda: now[0])
    now[0] += 0.5
    assert measurements.update(controller)["window_s"] == 0
    now[0] += 1.5
    stats.update(motor_count=180, motor_wait_s=0.18, motor_work_s=0.36, motor_overruns=7)
    result = measurements.update(controller)
    assert result["measured"]["motor_hz"] == 40  # Requested 50, achieved 40.
    assert result["measured"]["motor_wait_ms"] == pytest.approx(1)
    assert result["measured"]["motor_work_ms"] == pytest.approx(2)
    assert result["measured"]["motor_overruns"] == 3
    assert result["measured"]["pcb_hz"] == 0
    assert result["measured"]["pcb_work_ms"] is None
    now[0] += 1
    result = measurements.update(controller)
    assert result["measured"]["motor_hz"] == 0
    assert result["measured"]["motor_overruns"] == 0

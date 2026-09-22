"""Offline checks for the trajectory runner's movement guards."""

from types import SimpleNamespace

import pytest

from tests.localisation_trajectory import _safe_state


@pytest.mark.parametrize("fresh, valid", [(False, True), (True, False)])
def test_translation_rejects_stale_or_lost_pose(fresh, valid):
    hardware = SimpleNamespace(get_yaw=lambda: 0, get_gyro_z_deg_s=lambda: 0)
    state = {"pose": [1215, 910, 0, 0.9, valid], "fresh": fresh,
             "scan_age_s": 0.01, "imu_age_s": 0.01}
    with pytest.raises(RuntimeError, match="lost or stale"):
        _safe_state(state, hardware, 180, require_pose=True)

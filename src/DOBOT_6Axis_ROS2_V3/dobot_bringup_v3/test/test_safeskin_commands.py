import pytest

from dobot_bringup_v3.dobot_api import DobotApiDashboard
from dobot_bringup_v3.dobot_api import DobotParameterError


def make_dashboard():
    dashboard = object.__new__(DobotApiDashboard)
    dashboard.sent_commands = []

    def capture(command):
        dashboard.sent_commands.append(command)
        return '0,{},SetSafeSkin();'

    dashboard.sendRecvMsg = capture
    return dashboard


@pytest.mark.parametrize('status', [0, 1])
def test_set_safe_skin_serializes_legacy_v3_command(status):
    dashboard = make_dashboard()

    dashboard.SetSafeSkin(status)

    assert dashboard.sent_commands == [f'SetSafeSkin({status})']


@pytest.mark.parametrize('status', [-1, 2, 10])
def test_set_safe_skin_rejects_invalid_status(status):
    dashboard = make_dashboard()

    with pytest.raises(DobotParameterError, match='must be 0 or 1'):
        dashboard.SetSafeSkin(status)

import pytest

from dobot_bringup_v3.dobot_api import DobotApiDashboard
from dobot_bringup_v3.dobot_api import DobotParameterError


def make_dashboard():
    dashboard = object.__new__(DobotApiDashboard)
    dashboard.sent_commands = []

    def capture(command):
        dashboard.sent_commands.append(command)
        return '0,{},test();'

    dashboard.sendRecvMsg = capture
    return dashboard


def test_set_hold_regs_formats_ros_string_as_value_table():
    dashboard = make_dashboard()

    dashboard.SetHoldRegs(0, 0x0100, 1, '{1}', 'U16')

    assert dashboard.sent_commands == [
        'SetHoldRegs(0,256,1,{1},U16)'
    ]


def test_set_hold_regs_accepts_unbraced_single_value():
    dashboard = make_dashboard()

    dashboard.SetHoldRegs(0, 0x0103, 1, '1000', 'u16')

    assert dashboard.sent_commands == [
        'SetHoldRegs(0,259,1,{1000},U16)'
    ]


def test_set_hold_regs_accepts_multiple_values():
    dashboard = make_dashboard()

    dashboard.SetHoldRegs(0, 3095, 2, [6000, 300], 'U16')

    assert dashboard.sent_commands == [
        'SetHoldRegs(0,3095,2,{6000,300},U16)'
    ]


def test_get_hold_regs_omits_empty_optional_type():
    dashboard = make_dashboard()

    dashboard.GetHoldRegs(0, 0x0200, 3, '')

    assert dashboard.sent_commands == ['GetHoldRegs(0,512,3)']


def test_get_hold_regs_normalizes_value_type():
    dashboard = make_dashboard()

    dashboard.GetHoldRegs(0, 0x0200, 3, 'u16')

    assert dashboard.sent_commands == ['GetHoldRegs(0,512,3,U16)']


@pytest.mark.parametrize(
    'values,count',
    [
        ('{1,2}', 1),
        ('{not-a-number}', 1),
        ('{1', 1),
    ],
)
def test_set_hold_regs_rejects_malformed_values(values, count):
    dashboard = make_dashboard()

    with pytest.raises(DobotParameterError):
        dashboard.SetHoldRegs(0, 0x0100, count, values, 'U16')


def test_hold_register_commands_reject_invalid_type():
    dashboard = make_dashboard()

    with pytest.raises(DobotParameterError, match='Unsupported'):
        dashboard.GetHoldRegs(0, 0x0200, 1, 'INT16')

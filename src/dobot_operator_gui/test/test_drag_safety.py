import pytest

from dobot_operator_gui.ros_client import DobotRosClient
from dobot_operator_gui.ros_client import DobotServiceError


def make_client():
    return object.__new__(DobotRosClient)


@pytest.mark.parametrize(
    'enabled,expected_status',
    [(False, 0), (True, 1)],
)
def test_set_safe_skin_builds_expected_request(enabled, expected_status):
    client = make_client()
    calls = []
    client._call = lambda name, request: calls.append(
        (name, request.status)
    )

    client.set_safe_skin(enabled)

    assert calls == [('SetSafeSkin', expected_status)]


def test_start_drag_disables_selected_protections_in_order():
    client = make_client()
    calls = []
    client.get_mode = lambda: '5'
    client.set_collision_level = lambda level: calls.append(
        ('collision', level)
    )
    client.set_safe_skin = lambda enabled: calls.append(
        ('safe_skin', enabled)
    )
    client._call = lambda name, request: calls.append((name, None))
    client._wait_for_mode = lambda modes, timeout: '6'

    mode = client.start_drag(True, True, 4)

    assert mode == '6'
    assert calls == [
        ('collision', 0),
        ('safe_skin', False),
        ('StartDrag', None),
    ]


def test_start_drag_rolls_back_both_protections_on_failure():
    client = make_client()
    calls = []
    client.get_mode = lambda: '5'
    client.set_collision_level = lambda level: calls.append(
        ('collision', level)
    )
    client.set_safe_skin = lambda enabled: calls.append(
        ('safe_skin', enabled)
    )

    def reject_start(name, request):
        calls.append((name, None))
        raise DobotServiceError('simulated StartDrag failure')

    client._call = reject_start

    with pytest.raises(DobotServiceError, match='simulated'):
        client.start_drag(True, True, 4)

    assert calls == [
        ('collision', 0),
        ('safe_skin', False),
        ('StartDrag', None),
        ('safe_skin', True),
        ('collision', 4),
    ]


def test_stop_drag_waits_for_mode5_before_restoring_protections():
    client = make_client()
    calls = []

    client.get_mode = lambda: '6'
    client._call = lambda name, request: calls.append((name, None))
    client._wait_for_mode = lambda modes, timeout: (
        calls.append(('wait_mode', set(modes))),
        '5',
    )[1]
    client.set_safe_skin = lambda enabled: calls.append(
        ('safe_skin', enabled)
    )
    client.set_collision_level = lambda level: calls.append(
        ('collision', level)
    )

    mode = client.stop_drag(3, True)

    assert mode == '5'
    assert calls == [
        ('StopDrag', None),
        ('wait_mode', {'5'}),
        ('safe_skin', True),
        ('collision', 3),
    ]


def test_stop_drag_is_idempotent_after_mode_is_already_5():
    client = make_client()
    calls = []

    client.get_mode = lambda: '5'
    client._call = lambda name, request: calls.append((name, None))
    client.set_safe_skin = lambda enabled: calls.append(
        ('safe_skin', enabled)
    )
    client.set_collision_level = lambda level: calls.append(
        ('collision', level)
    )

    mode = client.stop_drag(3, True)

    assert mode == '5'
    assert calls == [('safe_skin', True), ('collision', 3)]


def test_stop_drag_restores_after_reported_error_if_mode_reaches_5():
    client = make_client()
    calls = []
    client.get_mode = lambda: '6'

    def reject_stop(name, request):
        calls.append((name, None))
        raise DobotServiceError('simulated StopDrag failure')

    client._call = reject_stop
    client._wait_for_mode = lambda modes, timeout: (
        calls.append(('wait_mode', set(modes))),
        '5',
    )[1]
    client.set_safe_skin = lambda enabled: calls.append(
        ('safe_skin', enabled)
    )
    client.set_collision_level = lambda level: calls.append(
        ('collision', level)
    )

    with pytest.raises(DobotServiceError, match='StopDrag失败'):
        client.stop_drag(3, True)

    assert calls == [
        ('StopDrag', None),
        ('wait_mode', {'5'}),
        ('safe_skin', True),
        ('collision', 3),
    ]


def test_stop_drag_does_not_write_protections_during_failed_transition():
    client = make_client()
    calls = []
    client.get_mode = lambda: '6'
    client._call = lambda name, request: calls.append((name, None))

    def reject_wait(modes, timeout):
        calls.append(('wait_mode', set(modes)))
        raise DobotServiceError('still mode 6')

    client._wait_for_mode = reject_wait
    client.set_safe_skin = lambda enabled: calls.append(
        ('safe_skin', enabled)
    )
    client.set_collision_level = lambda level: calls.append(
        ('collision', level)
    )

    with pytest.raises(DobotServiceError, match='未继续发送'):
        client.stop_drag(3, True)

    assert calls == [('StopDrag', None), ('wait_mode', {'5'})]


def test_restore_protections_attempts_collision_after_safeskin_error():
    client = make_client()
    calls = []

    def reject_safe_skin(enabled):
        calls.append(('safe_skin', enabled))
        raise DobotServiceError('simulated SafeSkin failure')

    client.set_safe_skin = reject_safe_skin
    client.set_collision_level = lambda level: calls.append(
        ('collision', level)
    )

    with pytest.raises(DobotServiceError, match='电子皮肤恢复失败'):
        client.restore_drag_protections(3, True)

    assert calls == [('safe_skin', True), ('collision', 3)]

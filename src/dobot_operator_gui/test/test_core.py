import json
from pathlib import Path

import pytest

from dobot_operator_gui.core import check_tcp_ports
from dobot_operator_gui.core import list_point_names
from dobot_operator_gui.core import load_point
from dobot_operator_gui.core import max_joint_error_deg
from dobot_operator_gui.core import OperatorInputError
from dobot_operator_gui.core import parse_integer_values
from dobot_operator_gui.core import parse_numeric_values
from dobot_operator_gui.core import point_image_path
from dobot_operator_gui.core import save_capture_group
from dobot_operator_gui.core import save_point
from dobot_operator_gui.core import validate_point_name


def test_connectivity_check_uses_wired_cr5_ports_by_default(monkeypatch):
    observed = []

    class Connection:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc_value, traceback):
            return False

    def fake_create_connection(address, timeout):
        observed.append((address, timeout))
        return Connection()

    monkeypatch.setattr(
        'dobot_operator_gui.core.socket.create_connection',
        fake_create_connection,
    )

    result = check_tcp_ports('192.168.192.201')

    assert result == {29999: '可连接', 30003: '可连接', 30005: '可连接'}
    assert [address for address, _ in observed] == [
        ('192.168.192.201', 29999),
        ('192.168.192.201', 30003),
        ('192.168.192.201', 30005),
    ]


def test_parse_existing_ros_response():
    text = (
        'response:\n'
        "dobot_msgs_v3.srv.GetAngle_Response(res=0, angle='{1,-2,3,4,5,6}')"
    )
    assert parse_numeric_values(text, 6) == (1.0, -2.0, 3.0, 4.0, 5.0, 6.0)


def test_save_and_load_legacy_point_files(tmp_path: Path):
    saved = save_point(
        tmp_path,
        'P3',
        (1, 2, 3, 4, 5, 6),
        (100, 200, 300, 10, 20, 30),
    )
    loaded = load_point(tmp_path, 'P3')

    assert loaded == saved
    assert list_point_names(tmp_path) == ['P3']
    assert "angle='{1.000000,2.000000" in (
        tmp_path / 'P3_joint.txt'
    ).read_text(encoding='utf-8')


def test_save_and_load_grouped_capture(tmp_path: Path):
    source_image = tmp_path / 'camera.jpg'
    source_image.write_bytes(b'jpeg-test-data')
    capture = save_capture_group(
        tmp_path / 'dataset',
        'P4',
        (1, 2, 3, 4, 5, 6),
        (100, 200, 300, 10, 20, 30),
        source_image,
        {'camera': {'model': 'SC3000'}},
    )

    assert capture.directory == tmp_path / 'dataset' / 'P4'
    assert capture.image_path.read_bytes() == b'jpeg-test-data'
    assert load_point(tmp_path / 'dataset', 'P4') == capture.point
    assert list_point_names(tmp_path / 'dataset') == ['P4']
    assert point_image_path(tmp_path / 'dataset', 'P4') == capture.image_path
    metadata = json.loads(capture.metadata_path.read_text(encoding='utf-8'))
    assert metadata['camera']['model'] == 'SC3000'
    assert metadata['joint_angles_deg'] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_grouped_capture_requires_explicit_overwrite(tmp_path: Path):
    source_image = tmp_path / 'camera.jpg'
    source_image.write_bytes(b'first')
    destination = tmp_path / 'dataset'
    save_capture_group(
        destination, 'P5', (1, 2, 3, 4, 5, 6),
        (10, 20, 30, 40, 50, 60), source_image,
    )
    source_image.write_bytes(b'second')

    with pytest.raises(OperatorInputError):
        save_capture_group(
            destination, 'P5', (6, 5, 4, 3, 2, 1),
            (60, 50, 40, 30, 20, 10), source_image,
        )

    replaced = save_capture_group(
        destination, 'P5', (6, 5, 4, 3, 2, 1),
        (60, 50, 40, 30, 20, 10), source_image, overwrite=True,
    )
    assert replaced.image_path.read_bytes() == b'second'
    assert load_point(destination, 'P5').joints == (6, 5, 4, 3, 2, 1)


@pytest.mark.parametrize('name', ['', '../P1', 'P 1', '中文点'])
def test_reject_unsafe_point_names(name):
    with pytest.raises(OperatorInputError):
        validate_point_name(name)


def test_wrapped_joint_error():
    assert max_joint_error_deg(
        (359, 0, 0, 0, 0, 0), (1, 0, 0, 0, 0, 0)
    ) == 2


def test_parse_gripper_modbus_values():
    assert parse_integer_values('1,2,1000', minimum_count=3) == (1, 2, 1000)


def test_reject_short_gripper_response():
    with pytest.raises(OperatorInputError):
        parse_integer_values('{1,2}', minimum_count=3)

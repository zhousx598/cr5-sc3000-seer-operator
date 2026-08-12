import json
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from seer_agv_driver.seer_client import (
    API_CANCEL_RELOCALIZATION,
    API_CONFIRM_LOCALIZATION,
    API_DOWNLOAD_MODEL,
    API_GOTO_STATION,
    API_JOG,
    API_LOAD_MAP,
    API_RELOCALIZE,
    API_STOP,
    HEADER,
    MAX_RESPONSE_BODY_BYTES,
    SeerAmbiguousMotionError,
    SeerApiError,
    SeerClient,
    SeerProtocolError,
    build_packet,
    evaluate_safety_state,
    parse_footprint,
    parse_header,
    recv_exact,
)


def test_build_stop_packet_matches_curated_doc_example():
    packet = build_packet(API_STOP.request, 1, b"")
    assert packet.hex(" ").upper() == "5A 01 00 01 00 00 00 00 07 D0 00 00 00 00 00 00"


def test_build_confirm_localization_packet_matches_curated_doc_example():
    packet = build_packet(API_CONFIRM_LOCALIZATION.request, 1, b"")
    assert packet.hex(" ").upper() == (
        "5A 01 00 01 00 00 00 00 07 D3 00 00 00 00 00 00"
    )
    assert API_CONFIRM_LOCALIZATION.response == 12003


@pytest.mark.parametrize(
    "spec,expected_type,expected_response",
    [
        (API_RELOCALIZE, "07 D2", 12002),
        (API_CANCEL_RELOCALIZATION, "07 D4", 12004),
        (API_LOAD_MAP, "07 E6", 12022),
    ],
)
def test_map_and_relocalization_api_headers(spec, expected_type, expected_response):
    packet = build_packet(spec.request, 1, b"")
    assert packet.hex(" ").upper()[24:29] == expected_type
    assert spec.response == expected_response


def test_build_jog_packet_uses_verified_api_type_and_body_length():
    body = json.dumps({"vx": -0.03, "vy": 0.0, "w": 0.0, "duration": 300}, separators=(",", ":")).encode()
    packet = build_packet(API_JOG.request, 7, body)
    packet_number, body_len, api_type = parse_header(packet[:16])
    assert packet[:2] == HEADER
    assert packet_number == 7
    assert body_len == len(body)
    assert api_type == 2010
    assert packet[16:] == body


def test_build_download_model_packet_uses_status_api_1500():
    packet = build_packet(API_DOWNLOAD_MODEL.request, 9, b"")
    packet_number, body_len, api_type = parse_header(packet[:16])
    assert packet_number == 9
    assert body_len == 0
    assert api_type == 1500
    assert API_DOWNLOAD_MODEL.response == 11500


def test_parse_footprint_extracts_chassis_rectangle_shape():
    model = {
        "deviceTypes": [
            {
                "name": "chassis",
                "devices": [
                    {
                        "deviceParams": [
                            {
                                "key": "shape",
                                "comboParam": {
                                    "childKey": "rectangle",
                                    "childParams": [
                                        {
                                            "key": "rectangle",
                                            "params": [
                                                {"key": "width", "doubleValue": 0.7},
                                                {"key": "head", "doubleValue": 0.52},
                                                {"key": "tail", "doubleValue": 0.48},
                                                {"key": "height", "doubleValue": 0.3},
                                            ],
                                        }
                                    ],
                                },
                            }
                        ]
                    }
                ],
            }
        ]
    }

    footprint = parse_footprint(model)
    assert footprint.width == 0.7
    assert footprint.head == 0.52
    assert footprint.tail == 0.48
    assert footprint.length == 1.0
    assert footprint.height == 0.3


def test_client_jog_clamps_to_commissioning_limits(monkeypatch):
    client = SeerClient(max_vx=0.1, max_vy=0.1, max_w=0.2, max_duration_ms=300)
    calls = []

    def fake_request(spec, body_obj=None, timeout=None):
        calls.append((spec, body_obj, timeout))
        return type("Resp", (), {"body": {"ret_code": 0}})()

    monkeypatch.setattr(client, "request", fake_request)
    client.jog(1.0, -1.0, 1.0, 1000)
    assert calls[0][0] == API_JOG
    assert calls[0][1] == {"vx": 0.1, "vy": -0.1, "w": 0.2, "duration": 300}


def test_control_commands_use_persistent_control_request(monkeypatch):
    client = SeerClient()
    calls = []

    def fake_persistent(spec, body_obj=None, timeout=None):
        calls.append((spec, body_obj, timeout))
        return type("Resp", (), {"body": {"ret_code": 0}})()

    monkeypatch.setattr(client, "_persistent_control_request", fake_persistent)
    client.jog(-0.03, 0.0, 0.0, 200)
    client.stop()
    client.confirm_localization()

    assert calls[0][0] == API_JOG
    assert calls[0][1] == {"vx": -0.03, "vy": 0.0, "w": 0.0, "duration": 200}
    assert calls[1][0] == API_STOP
    assert calls[2][0] == API_CONFIRM_LOCALIZATION
    assert calls[2][1] is None


def test_goto_station_uses_verified_navigation_api_and_limits(monkeypatch):
    client = SeerClient()
    calls = []

    def fake_request(spec, body_obj=None, timeout=None):
        calls.append((spec, body_obj, timeout))
        return type("Resp", (), {"body": {"ret_code": 0}})()

    monkeypatch.setattr(client, "request", fake_request)
    client.goto_station("LM2", task_id="test", max_speed=9.0, max_wspeed=9.0, max_acc=9.0, max_wacc=9.0)

    assert calls[0][0] == API_GOTO_STATION
    assert calls[0][1] == {
        "source_id": "SELF_POSITION",
        "id": "LM2",
        "max_speed": 0.2,
        "max_wspeed": 0.3,
        "max_acc": 0.2,
        "max_wacc": 0.3,
        "task_id": "test",
    }


def test_map_and_relocalization_request_bodies(monkeypatch):
    client = SeerClient()
    calls = []

    def fake_request(spec, body_obj=None, timeout=None):
        calls.append((spec, body_obj, timeout))
        return type("Resp", (), {"body": {"ret_code": 0}})()

    monkeypatch.setattr(client, "request", fake_request)
    client.load_map("2026.7.31_test1")
    client.relocalize(1.2, -0.4, 0.25, 0.5)
    client.cancel_relocalization()

    assert calls[0][:2] == (API_LOAD_MAP, {"map_name": "2026.7.31_test1"})
    assert calls[1][:2] == (
        API_RELOCALIZE,
        {"x": 1.2, "y": -0.4, "angle": 0.25, "length": 0.5},
    )
    assert calls[2][:2] == (API_CANCEL_RELOCALIZATION, None)


@pytest.mark.parametrize("name", ["", "bad/name", ".."])
def test_load_map_rejects_invalid_names(name):
    with pytest.raises(SeerApiError):
        SeerClient().load_map(name)


class BufferSocket:
    def __init__(self, incoming: bytes, chunk_size: int = 4096):
        self.incoming = bytearray(incoming)
        self.chunk_size = chunk_size
        self.sent = []

    def sendall(self, data: bytes):
        self.sent.append(data)

    def recv(self, size: int) -> bytes:
        count = min(size, self.chunk_size, len(self.incoming))
        if count <= 0:
            return b""
        data = bytes(self.incoming[:count])
        del self.incoming[:count]
        return data


def test_recv_exact_accepts_fragmented_socket_reads():
    sock = BufferSocket(b"0123456789", chunk_size=2)
    assert recv_exact(sock, 10) == b"0123456789"


def test_response_packet_number_must_match_request():
    client = SeerClient()
    with pytest.raises(SeerProtocolError, match="packet number mismatch"):
        client._checked_response(API_STOP, 7, 8, API_STOP.response, {})


def test_oversized_response_is_rejected_before_body_read():
    header = (
        HEADER
        + struct.pack(">H", 1)
        + struct.pack(">I", MAX_RESPONSE_BODY_BYTES + 1)
        + struct.pack(">H", API_STOP.response)
        + b"\x00" * 6
    )
    client = SeerClient()
    with pytest.raises(SeerProtocolError, match="too large"):
        client._request_on_socket(BufferSocket(header), API_STOP.request, 1, b"")


def test_jog_network_error_is_not_replayed(monkeypatch):
    client = SeerClient()
    attempts = []

    class BrokenSocket:
        def sendall(self, unused_data):
            attempts.append("send")
            raise OSError("lost reply")

        def close(self):
            pass

        def settimeout(self, unused_timeout):
            pass

    monkeypatch.setattr(client, "_get_control_sock", lambda timeout=None: BrokenSocket())
    with pytest.raises(SeerAmbiguousMotionError):
        client.jog(0.02, 0.0, 0.0, 200)
    assert attempts == ["send"]


def test_differential_drive_rejects_lateral_jog():
    client = SeerClient(max_vy=0.0)
    with pytest.raises(SeerApiError, match="lateral velocity"):
        client.jog(0.0, 0.01, 0.0, 200)


def _safe_state(**fast_overrides):
    fast = {
        "blocked": False,
        "slowed": False,
        "fatals": [],
        "errors": [],
        "warnings": [],
        "emergency": False,
        "driver_emc": False,
        "soft_emc": False,
        "task_status": 0,
    }
    fast.update(fast_overrides)
    return evaluate_safety_state(
        fast,
        {"reloc_status": 1},
        {"loadmap_status": 1},
        {"charging": False, "manual_charge": False, "auto_charge": False},
        {"locked": False},
    )


def test_localization_completed_but_unconfirmed_is_inhibited():
    state = evaluate_safety_state(
        {}, {"reloc_status": 3}, {"loadmap_status": 1}, {}, {"locked": False}
    )
    assert not state.safe_to_move
    assert state.localization_pending_confirmation
    assert "operator confirmation" in state.reason


@pytest.mark.parametrize(
    "fast,battery,owner,expected",
    [
        ({"emergency": True}, {}, {"locked": False}, "emergency stop"),
        ({}, {"charging": True}, {"locked": False}, "charging"),
        ({}, {}, {"locked": True, "ip": "192.0.2.1"}, "control owned"),
    ],
)
def test_new_safety_gates_inhibit_motion(fast, battery, owner, expected):
    state = evaluate_safety_state(
        fast,
        {"reloc_status": 1},
        {"loadmap_status": 1},
        battery,
        owner,
    )
    assert not state.safe_to_move
    assert expected in state.reason


def test_active_navigation_blocks_teleop_but_not_current_navigation():
    state = _safe_state(task_status=2)
    assert state.safe_to_move
    assert not state.safe_for_teleop
    assert not state.safe_to_start_navigation
    assert "navigation task active" in state.teleop_reason

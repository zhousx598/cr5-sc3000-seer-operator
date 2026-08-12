from collections import deque

import numpy as np
import pytest

from dobot_bringup_v3 import dobot_api
from dobot_bringup_v3.feedback import fankuis
from dobot_bringup_v3.feedback import MyType


class ChunkSocket:
    def __init__(self, chunks):
        self.chunks = deque(chunks)
        self.closed = False

    def recv(self, size):
        if not self.chunks:
            return b''
        chunk = self.chunks.popleft()
        if len(chunk) > size:
            self.chunks.appendleft(chunk[size:])
            return chunk[:size]
        return chunk

    def close(self):
        self.closed = True


def make_feedback_client(chunks):
    client = object.__new__(fankuis)
    client.socket_feedback = ChunkSocket(chunks)
    client._buffer = bytearray()
    return client


def test_feedback_reads_a_complete_frame_from_partial_tcp_chunks():
    frame = np.zeros(1, dtype=MyType)
    frame['test_value'][0] = 0x123456789ABCDEF
    frame['q_actual'][0] = [1, 2, 3, 4, 5, 6]
    frame['tool_vector_actual'][0] = [10, 20, 30, 40, 50, 60]
    raw = frame.tobytes()
    client = make_feedback_client([raw[:137], raw[137:900], raw[900:]])

    tool, joints = client.feed()

    assert joints.tolist() == [1, 2, 3, 4, 5, 6]
    assert tool.tolist() == [10, 20, 30, 40, 50, 60]


def test_feedback_reports_closed_connection():
    client = make_feedback_client([b''])

    with pytest.raises(ConnectionError, match='closed by robot'):
        client.feed()


def test_dashboard_socket_is_closed_when_connect_fails(monkeypatch):
    class FailingSocket:
        def __init__(self):
            self.closed = False

        def settimeout(self, timeout):
            self.timeout = timeout

        def connect(self, address):
            raise TimeoutError('simulated timeout')

        def close(self):
            self.closed = True

    failing_socket = FailingSocket()
    monkeypatch.setattr(dobot_api.socket, 'socket', lambda: failing_socket)

    with pytest.raises(TimeoutError, match='simulated timeout'):
        dobot_api.DobotApi('192.0.2.1', 29999)

    assert failing_socket.closed

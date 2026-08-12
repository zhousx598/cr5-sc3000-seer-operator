import socket
import struct
import threading

from dobot_operator_gui.camera_capture import Sc3000CameraCapture


def _recv_exact(sock, size):
    data = b''
    while len(data) < size:
        data += sock.recv(size - len(data))
    return data


def _fake_modbus_server(listener, requests):
    connection, unused_address = listener.accept()
    with connection:
        read_count = 0
        for unused in range(6):
            header = _recv_exact(connection, 7)
            transaction_id, protocol, length, unit_id = struct.unpack(
                '>HHHB', header
            )
            pdu = _recv_exact(connection, length - 1)
            requests.append((pdu[0], pdu[1:]))
            if pdu[0] == 0x06:
                response_pdu = pdu
            else:
                status = 0x0001 if read_count == 0 else 0x0100
                read_count += 1
                response_pdu = bytes([0x03, 0x02]) + struct.pack('>H', status)
            response = struct.pack(
                '>HHHB',
                transaction_id,
                protocol,
                len(response_pdu) + 1,
                unit_id,
            ) + response_pdu
            # Split the response to verify that the client handles TCP
            # fragmentation rather than assuming one recv() per frame.
            connection.sendall(response[:4])
            connection.sendall(response[4:])


def test_trigger_modbus_sequence_and_fragmented_responses():
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.bind(('127.0.0.1', 0))
    listener.listen(1)
    requests = []
    thread = threading.Thread(
        target=_fake_modbus_server, args=(listener, requests), daemon=True
    )
    thread.start()
    try:
        status = Sc3000CameraCapture._trigger_modbus(
            '127.0.0.1', listener.getsockname()[1], 1, 1.0
        )
    finally:
        listener.close()
        thread.join(timeout=1.0)

    assert status == 0x0100
    assert [function for function, unused in requests] == [
        0x06, 0x03, 0x06, 0x03, 0x06, 0x06
    ]
    written_values = [
        struct.unpack('>HH', payload)[1]
        for function, payload in requests
        if function == 0x06
    ]
    assert written_values == [0x0001, 0x0003, 0x0005, 0x0001]

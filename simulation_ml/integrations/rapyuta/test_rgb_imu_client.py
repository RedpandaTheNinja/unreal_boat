import base64
import unittest
from rgb_imu_client import SensorClient


class FakeClient(SensorClient):
    def __init__(self, replies):
        self.replies = iter(replies)
        self.sent = []

    def request(self, data, accept):
        self.sent.append(data)
        reply = next(self.replies)
        assert accept(reply)
        return reply


def frame(seq, size=6, count=2):
    return {'valid': True, 'rgb': {'valid': True, 'sequence': seq, 'byte_count': size, 'chunk_count': count}}


def chunk(seq, index, data):
    return {'valid': True, 'rgb_sequence': seq, 'rgb_chunk': index,
            'data_base64': base64.b64encode(data).decode()}


class CameraTransportTests(unittest.TestCase):
    def test_reassembles_and_never_commands_motors(self):
        client = FakeClient([frame(3), chunk(3, 0, b'\xff\xd8ab'), chunk(3, 1, b'\xff\xd9')])
        jpeg, meta = client.camera()
        self.assertEqual(jpeg, b'\xff\xd8ab\xff\xd9')
        self.assertEqual(meta['rgb']['sequence'], 3)
        self.assertTrue(all('v_mps' not in r and 'yaw_radps' not in r for r in client.sent))

    def test_frame_replacement_discards_partial_image(self):
        client = FakeClient([frame(1), chunk(1, 0, b'\xff\xd8xx'),
                             {'valid': False, 'rgb_sequence': 2, 'rgb_chunk': 1},
                             frame(2), chunk(2, 0, b'\xff\xd8yy'), chunk(2, 1, b'\xff\xd9')])
        jpeg, _ = client.camera()
        self.assertEqual(jpeg, b'\xff\xd8yy\xff\xd9')

    def test_rejects_truncated_payload(self):
        client = FakeClient([frame(1, size=7), chunk(1, 0, b'\xff\xd8ab'), chunk(1, 1, b'\xff\xd9')])
        with self.assertRaises(RuntimeError):
            client.camera()


if __name__ == '__main__':
    unittest.main()

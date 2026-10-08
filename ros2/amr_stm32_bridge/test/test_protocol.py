import struct
import unittest

from amr_stm32_bridge.protocol import (
    DEVICE_DRIVE,
    FLAG_ACK_REQUIRED,
    FLAG_NONE,
    MODE_AUTO,
    MODE_SAFE,
    MSG_ACK,
    MSG_CLEAR_FAULT,
    MSG_DRIVE_COMMAND,
    MSG_DRIVE_CONFIG,
    MSG_ODOMETRY,
    MSG_ODOMETRY_STREAM_CONTROL,
    MSG_SET_MODE,
    DRIVE_CONFIG_GET,
    DRIVE_CONFIG_REPORT,
    DRIVE_CONFIG_SET,
    DriveConfig,
    StreamDecoder,
    cobs_decode,
    cobs_encode,
    decode_ack,
    decode_drive_config,
    decode_frame,
    decode_odometry,
    encode_packet,
    make_clear_fault,
    make_drive_command,
    make_drive_config,
    make_set_mode,
    make_stream_control,
    u32_is_newer,
)


class ProtocolTest(unittest.TestCase):
    def test_cobs_round_trip(self):
        source = bytes(range(256)) + b"\x00AMR\x00"
        self.assertEqual(cobs_decode(cobs_encode(source)), source)

    def test_control_packets(self):
        clear = decode_frame(make_clear_fault(5)[:-1])
        safe = decode_frame(make_set_mode(6, MODE_SAFE)[:-1])
        auto = decode_frame(make_set_mode(7, MODE_AUTO)[:-1])
        stream = decode_frame(make_stream_control(8, 50)[:-1])

        self.assertEqual(clear.message_id, MSG_CLEAR_FAULT)
        self.assertEqual(clear.flags, FLAG_ACK_REQUIRED)
        self.assertEqual(clear.payload, b"")
        self.assertEqual(safe.message_id, MSG_SET_MODE)
        self.assertEqual(safe.payload, bytes([MODE_SAFE]))
        self.assertEqual(auto.payload, bytes([MODE_AUTO]))
        self.assertEqual(stream.message_id, MSG_ODOMETRY_STREAM_CONTROL)
        self.assertEqual(struct.unpack("<HH", stream.payload), (50, 0))

    def test_drive_command_packet_without_ack(self):
        packet = decode_frame(
            make_drive_command(9, 2.0, -1.5, 300)[:-1]
        )
        self.assertEqual(packet.device_id, DEVICE_DRIVE)
        self.assertEqual(packet.message_id, MSG_DRIVE_COMMAND)
        self.assertEqual(packet.flags, FLAG_NONE)
        self.assertEqual(
            struct.unpack("<iiHH", packet.payload),
            (2000, -1500, 300, 0),
        )

    def test_drive_config_set_packet(self):
        config = DriveConfig(
            heading_hold_enabled=False,
            wheel_kp=65.0,
            wheel_ki=35.0,
            turn_kp=1.9,
        )
        packet = decode_frame(
            make_drive_config(20, DRIVE_CONFIG_SET, config)[:-1]
        )
        self.assertEqual(packet.message_id, MSG_DRIVE_CONFIG)
        self.assertEqual(packet.flags, FLAG_ACK_REQUIRED)
        self.assertEqual(len(packet.payload), 116)
        self.assertEqual(packet.payload[0], DRIVE_CONFIG_SET)

    def test_drive_config_report_decode(self):
        config = DriveConfig(
            heading_hold_enabled=True,
            wheel_kp=61.25,
            heading_rate_kp=0.72,
            turn_kd=0.14,
            turn_settle_time_ms=180,
        )
        request = decode_frame(
            make_drive_config(21, DRIVE_CONFIG_GET, config, require_ack=False)[:-1]
        )
        payload = bytearray(request.payload)
        payload[0] = DRIVE_CONFIG_REPORT
        report = decode_frame(
            encode_packet(MSG_DRIVE_CONFIG, 22, bytes(payload), flags=0)[:-1]
        )
        decoded = decode_drive_config(report)
        self.assertTrue(decoded.heading_hold_enabled)
        self.assertAlmostEqual(decoded.wheel_kp, 61.25, places=4)
        self.assertAlmostEqual(decoded.heading_rate_kp, 0.72, places=4)
        self.assertAlmostEqual(decoded.turn_kd, 0.14, places=4)
        self.assertEqual(decoded.turn_settle_time_ms, 180)

    def test_ack_decode(self):
        wire = encode_packet(
            MSG_ACK,
            11,
            struct.pack("<HBB", 7, 0, 0),
            flags=0,
        )
        ack = decode_ack(decode_frame(wire[:-1]))
        self.assertEqual(ack.acknowledged_sequence, 7)
        self.assertEqual(ack.status, 0)
        self.assertFalse(ack.is_nack)

    def test_odometry_decode(self):
        payload = struct.pack(
            "<IiiiiiIB3x",
            1234,
            791,
            -16,
            -1602,
            200,
            -300,
            42,
            1,
        )
        wire = encode_packet(MSG_ODOMETRY, 8, payload, flags=0)
        sample = decode_odometry(decode_frame(wire[:-1]))
        self.assertAlmostEqual(sample.x_m, 0.791)
        self.assertAlmostEqual(sample.y_m, -0.016)
        self.assertAlmostEqual(sample.yaw_rad, -1.602)
        self.assertAlmostEqual(sample.linear_velocity_m_s, 0.2)
        self.assertAlmostEqual(sample.angular_velocity_rad_s, -0.3)
        self.assertTrue(sample.valid)

    def test_incremental_stream_decoder(self):
        first = encode_packet(MSG_ACK, 1, struct.pack("<HBB", 1, 0, 0))
        second = encode_packet(MSG_ACK, 2, struct.pack("<HBB", 2, 0, 0))
        decoder = StreamDecoder()
        packets = []
        data = first + second
        for offset in range(0, len(data), 3):
            packets.extend(decoder.feed(data[offset:offset + 3]))
        self.assertEqual([packet.sequence for packet in packets], [1, 2])
        self.assertEqual(decoder.error_count, 0)

    def test_bad_frame_resynchronizes_at_delimiter(self):
        bad = bytearray(
            encode_packet(MSG_ACK, 3, struct.pack("<HBB", 3, 0, 0))
        )
        bad[6] ^= 0x20
        good = encode_packet(MSG_ACK, 4, struct.pack("<HBB", 4, 0, 0))
        decoder = StreamDecoder()
        packets = decoder.feed(bytes(bad) + good)
        self.assertEqual([packet.sequence for packet in packets], [4])
        self.assertEqual(decoder.error_count, 1)

    def test_u32_sequence_wrap(self):
        self.assertTrue(u32_is_newer(0, 0xFFFFFFFF))
        self.assertTrue(u32_is_newer(10, 9))
        self.assertFalse(u32_is_newer(9, 10))
        self.assertFalse(u32_is_newer(10, 10))


if __name__ == "__main__":
    unittest.main()

import math
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch, Mock, call

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from jlink_protocol import (
    ACK_OFFSET, FirmwareSymbols, MAGIC, MAILBOX_SIZE, MailboxClient,
    PAYLOAD_OFFSET, REQUEST_OFFSET, SAMPLE_OFFSET, SAMPLE_STRUCT, Sample, VERSION, connect_background,
)


class FakeLink:
    def __init__(self):
        self.base = 0x20000000
        self.memory = bytearray(512)
        self.writes = []
        self.symbols = FirmwareSymbols(self.base, self.base + 256, b"test-build".ljust(32, b"\0"))
        self.memory[256:288] = self.symbols.build_id
        struct.pack_into("<3I", self.memory, 0, MAGIC, VERSION, MAILBOX_SIZE)
        self.publish()

    def memory_read32(self, address, count):
        return list(struct.unpack_from(f"<{count}I", self.memory, address - self.base))

    def memory_write32(self, address, words):
        self.writes.append((address, list(words)))
        struct.pack_into(f"<{len(words)}I", self.memory, address - self.base, *words)

    def publish(self, sequence=2, gains=None):
        values = [sequence, 100, 1, 0, 1, 60, 20, 100,
                  10.0, 8.0, 9.0, 9.0, 12.5, 1.0, 2.0, 3.0]
        self.memory[SAMPLE_OFFSET:MAILBOX_SIZE] = SAMPLE_STRUCT.pack(*values, *(gains or [0.1] * 18))

    def ack(self, status=1):
        request = struct.unpack_from("<I", self.memory, REQUEST_OFFSET)[0]
        struct.pack_into("<2I", self.memory, ACK_OFFSET, request, status)


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.link = FakeLink()
        self.client = MailboxClient(self.link, self.link.symbols)

    def test_verify_and_decode(self):
        self.client.verify()
        sample = self.client.read_sample()
        self.assertEqual((sample.target, sample.left, sample.right, sample.pwm), (10, 8, 9, 60))
        self.assertEqual(len(sample.gains), 18)

    def test_background_attach_avoids_device_reset_hooks(self):
        link = Mock()
        link.halted.return_value = False
        connect_background(link, "STM32G070RBTx", 1000)
        self.assertEqual(link.mock_calls[:2], [call.exec_command("InhibitConnectRetries = 1"),
                                               call.exec_command("SetRestartOnClose = 0")])
        link.connect.assert_called_once_with("Cortex-M0", speed=1000, verbose=False)
        link.reset.assert_not_called()
        link.halt.assert_not_called()
        link.restart.assert_not_called()

    def test_paused_target_not_resumed(self):
        link = Mock()
        link.halted.return_value = True
        with self.assertRaises(RuntimeError):
            connect_background(link, "STM32G070RBTx", 1000)
        link.restart.assert_not_called()
        link.reset.assert_not_called()

    def test_unknown_device_never_connects(self):
        link = Mock()
        with self.assertRaises(ValueError):
            connect_background(link, "STM32F407VG", 1000)
        self.assertEqual(link.mock_calls, [])

    def test_reject_wrong_firmware_and_abi(self):
        for offset in (0, 4, 8, 256):
            with self.subTest(offset=offset):
                link = FakeLink()
                link.memory[offset] ^= 1
                with self.assertRaises(ValueError):
                    MailboxClient(link, link.symbols).verify()
                self.assertEqual(link.writes, [])

    def test_refuse_outstanding_command(self):
        struct.pack_into("<I", self.link.memory, REQUEST_OFFSET, 1)
        with self.assertRaises(RuntimeError):
            self.client.verify()

    def test_torn_sample_is_not_returned(self):
        self.link.publish(sequence=3)
        self.assertIsNone(self.client.read_sample())
        self.link.publish(sequence=2)
        original = self.link.memory_read32

        def changing(address, count):
            values = original(address, count)
            if count == 1:
                values[0] += 2
            return values
        self.link.memory_read32 = changing
        self.assertIsNone(self.client.read_sample())

    def test_payload_then_commit_and_confirmed_readback(self):
        gains = (1.1, 0.2, 0.03)
        self.client.send_gains(1, 2, gains)
        self.assertEqual([addr - self.link.base for addr, _ in self.link.writes], [PAYLOAD_OFFSET, REQUEST_OFFSET])
        self.assertIsNone(self.client.poll_ack(self.client.read_sample()))
        with self.assertRaises(RuntimeError):
            self.client.send_gains(1, 2, gains)
        self.link.ack()
        self.assertIsNone(self.client.poll_ack(self.client.read_sample()))  # stale gains
        self.link.publish(sequence=4, gains=[0.1] * 15 + list(gains))
        result = self.client.poll_ack(self.client.read_sample())
        self.assertEqual(result[:2], (1, 2))
        self.assertIsNone(self.client.pending)

    def test_invalid_values_never_write(self):
        for value in (float("nan"), float("inf"), -0.1, 1001):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.client.send_gains(0, 0, (value, 1, 0))
        with self.assertRaises(ValueError):
            self.client.send_gains(2, 0, (1, 1, 0))
        self.assertEqual(self.link.writes, [])

    def test_corrupted_payload_not_committed(self):
        original = self.link.memory_write32

        def corrupt(address, words):
            original(address, words)
            self.link.memory[PAYLOAD_OFFSET] ^= 1
        self.link.memory_write32 = corrupt
        with self.assertRaises(IOError):
            self.client.send_gains(0, 0, (1, 1, 0))
        self.assertEqual(len(self.link.writes), 1)

    def test_ack_timeout_no_retry(self):
        with patch("jlink_protocol.time.monotonic", return_value=100):
            self.client.send_gains(0, 0, (1, 0, 0))
        with patch("jlink_protocol.time.monotonic", return_value=103):
            with self.assertRaises(TimeoutError):
                self.client.poll_ack(self.client.read_sample())
        self.assertEqual(len(self.link.writes), 2)

    def test_firmware_rejection(self):
        self.client.send_gains(0, 0, (1, 0, 0))
        self.link.ack(status=2)
        with self.assertRaises(ValueError):
            self.client.poll_ack(self.client.read_sample())

    def test_sequence_wrap(self):
        struct.pack_into("<2I", self.link.memory, REQUEST_OFFSET, 0xFFFFFFFF, 0xFFFFFFFF)
        self.client.send_gains(0, 0, (1, 0, 0))
        self.assertEqual(self.link.writes[-1][1], [1])


if __name__ == "__main__":
    unittest.main()

"""Transport behaviour against a fake robot on a real pty."""

from __future__ import annotations

import sys
import threading

import pytest

from eilik.errors import (
    BlacklistedCommandError,
    EilikTimeoutError,
    FrameError,
    UnsupportedCommandError,
)
from eilik.protocol import BLACKLISTED_COMMANDS, Command, encode_frame, encode_heartbeat
from eilik.transport import DEFAULT_BAUDRATE, SerialTransport, list_candidate_ports

from .fake_robot import raw_frame


class TestBaudRate:
    """125000 is non-standard; Linux reaches it through TCSETS2/BOTHER."""

    def test_default_is_the_documented_nominal_rate(self):
        assert DEFAULT_BAUDRATE == 125000

    @pytest.mark.skipif(not sys.platform.startswith("linux"), reason="termios2 is Linux-only")
    def test_custom_rate_is_applied_and_reads_back(self, fake_robot):
        link = SerialTransport(port=fake_robot.port, baudrate=125000, timeout=1.0)
        try:
            assert link.actual_baudrate == 125000
        finally:
            link.close()

    @pytest.mark.skipif(not sys.platform.startswith("linux"), reason="termios2 is Linux-only")
    def test_a_standard_rate_still_works(self, fake_robot):
        link = SerialTransport(port=fake_robot.port, baudrate=115200, timeout=1.0)
        try:
            assert link.actual_baudrate == 115200
        finally:
            link.close()

    def test_repr_mentions_port_and_rate(self, transport):
        assert transport.port in repr(transport)
        assert "125000" in repr(transport)


class TestWriteGuard:
    """The blacklist is enforced again on the raw write path."""

    @pytest.mark.parametrize("opcode", sorted(BLACKLISTED_COMMANDS))
    def test_hand_assembled_destructive_frames_are_refused(self, transport, fake_robot, opcode):
        """Bypassing the encoder must not get a destructive opcode onto the wire."""
        with pytest.raises(BlacklistedCommandError):
            transport.send_frame(raw_frame(opcode))
        assert fake_robot.received == []

    def test_unknown_opcode_is_refused_on_the_raw_path(self, transport):
        with pytest.raises(UnsupportedCommandError):
            transport.send_frame(raw_frame(0x7B))

    def test_non_frame_bytes_are_refused(self, transport):
        with pytest.raises(FrameError, match="well-formed frame"):
            transport.send_frame(b"hello world")

    def test_short_bytes_are_refused(self, transport):
        with pytest.raises(FrameError):
            transport.send_frame(b"\xaa\xaa\xaa")


class TestRequestResponse:
    """A request waits for the reply carrying the same opcode."""

    def test_ping(self, transport):
        frame = transport.request(Command.PING)
        assert frame.command == Command.PING
        assert len(frame) == 41

    def test_read_servos(self, transport):
        frame = transport.request(Command.READ_SERVOS)
        assert frame.command == Command.READ_SERVOS
        assert frame.data[0] == 4

    def test_read_screen_moves_1032_bytes(self, transport):
        frame = transport.request(Command.READ_SCREEN, timeout=5.0)
        assert len(frame) == 1032
        assert len(frame.data) == 1025

    def test_heartbeat_roundtrip(self, transport):
        transport.send_frame(encode_heartbeat())
        assert transport.read_frame(timeout=2.0).command == Command.ENVELOPE

    def test_the_frame_reaching_the_robot_is_the_golden_ping(self, transport, fake_robot):
        transport.request(Command.PING)
        assert fake_robot.received[0] == (Command.PING, b"")

    def test_unsolicited_frames_are_skipped(self, transport, fake_robot):
        """A heartbeat arriving mid-exchange must not be mistaken for the reply."""
        fake_robot.inject_before_reply = [raw_frame(Command.ENVELOPE, b"\x01")] * 3
        frame = transport.request(Command.READ_SERVOS)
        assert frame.command == Command.READ_SERVOS

    def test_a_corrupt_reply_is_discarded_and_times_out(self, transport, fake_robot):
        """Corruption surfaces as a timeout, not as bad data.

        The firmware never answers a frame whose checksum is wrong.
        """
        fake_robot.corrupt_next_reply = True
        with pytest.raises(EilikTimeoutError):
            transport.request(Command.PING, timeout=0.4)

    def test_silence_times_out(self, transport, fake_robot):
        fake_robot.drop_next_request = True
        with pytest.raises(EilikTimeoutError, match="no reply to command 0x01"):
            transport.request(Command.PING, timeout=0.3)

    def test_resynchronises_after_leading_garbage(self, transport, fake_robot):
        fake_robot.inject_before_reply = [b"\x00\xff\xaa\xaa garbage \xaa"]
        assert transport.request(Command.PING).command == Command.PING

    def test_recovers_on_the_next_request_after_a_timeout(self, transport, fake_robot):
        fake_robot.drop_next_request = True
        with pytest.raises(EilikTimeoutError):
            transport.request(Command.PING, timeout=0.3)
        assert transport.request(Command.PING).command == Command.PING


class TestSerialisation:
    """Frames must never interleave, whatever the caller does."""

    def test_concurrent_requests_do_not_interleave(self, transport, fake_robot):
        """Requests from different threads must stay strictly serialised.

        Mixing a servo write and a screen write without waiting for the
        acknowledgement crashes the servo controller, so the transport holds a
        lock across each whole exchange.
        """
        errors = []

        def servo_writes():
            try:
                for _ in range(15):
                    transport.request(Command.WRITE_SERVOS, b"\x01\x04\xdc\x05", timeout=5.0)
            except Exception as exc:
                errors.append(exc)

        def screen_writes():
            try:
                for _ in range(15):
                    transport.request(Command.WRITE_SCREEN, bytes(1024), timeout=5.0)
            except Exception as exc:
                errors.append(exc)

        threads = [threading.Thread(target=servo_writes), threading.Thread(target=screen_writes)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=30)

        assert errors == []
        commands = [command for command, _ in fake_robot.received]
        assert commands.count(Command.WRITE_SERVOS) == 15
        assert commands.count(Command.WRITE_SCREEN) == 15

    def test_frames_are_spaced_by_the_inter_frame_delay(self, fake_robot):
        link = SerialTransport(port=fake_robot.port, timeout=2.0, inter_frame_delay=0.05)
        try:
            import time

            start = time.monotonic()
            for _ in range(4):
                link.request(Command.PING)
            assert time.monotonic() - start >= 0.05 * 3
        finally:
            link.close()


class TestLifecycle:
    """Opening, closing and discovery."""

    def test_context_manager_closes(self, fake_robot):
        with SerialTransport(port=fake_robot.port, timeout=1.0) as link:
            assert link.is_open
        assert not link.is_open

    def test_close_is_idempotent(self, fake_robot):
        link = SerialTransport(port=fake_robot.port, timeout=1.0)
        link.close()
        link.close()
        assert not link.is_open

    def test_candidate_ports_is_a_list_of_strings(self):
        assert all(isinstance(device, str) for device in list_candidate_ports())


def test_encode_frame_and_send_frame_agree(transport):
    """The encoder and the raw write path accept exactly the same opcodes."""
    transport.send_frame(encode_frame(Command.READ_SERVOS))
    assert transport.read_frame(timeout=2.0).command == Command.READ_SERVOS

"""End-to-end behaviour of the high-level API against a fake robot."""

from __future__ import annotations

import pytest

from eilik.errors import ProtocolError, ServoRangeWarning
from eilik.protocol import Command
from eilik.robot import Eilik, FirmwareInfo, _extract_text
from eilik.screen import FRAMEBUFFER_SIZE, blank, get_pixel, rotate180, set_pixel
from eilik.servo import Motor, ServoLimits

from .fake_robot import PING_PAYLOAD, raw_frame


class TestPing:
    def test_returns_firmware_info(self, robot):
        info = robot.ping()
        assert isinstance(info, FirmwareInfo)
        assert info.status == 0x94
        assert info.payload == PING_PAYLOAD

    def test_str_is_a_one_liner(self, robot):
        assert "\n" not in str(robot.ping())

    def test_extracts_printable_text(self):
        assert _extract_text(b"\x00\x01EILIK v1.2\xff\x00ab") == "EILIK v1.2"

    def test_empty_reply_is_refused(self, robot, fake_robot, monkeypatch):
        monkeypatch.setattr(
            fake_robot, "_reply_for", lambda *_: raw_frame(Command.PING), raising=True
        )
        with pytest.raises(ProtocolError, match="no data"):
            robot.ping()


class TestServos:
    def test_read_returns_all_four_motors(self, robot):
        assert set(robot.read_servos()) == set(Motor)

    def test_write_then_read_roundtrip(self, robot):
        robot.write_servos({Motor.ARM_RIGHT: 1200, Motor.HEAD: 1600})
        positions = robot.read_servos()
        assert positions[Motor.ARM_RIGHT] == 1200
        assert positions[Motor.HEAD] == 1600

    def test_write_returns_what_was_actually_sent(self, robot):
        assert robot.write_servos({Motor.BODY: 1500}) == {Motor.BODY: 1500}

    def test_positions_are_clamped_before_transmission(self, robot):
        """An out-of-range request must reach the robot already clamped."""
        assert robot.write_servos({Motor.HEAD: 9999}) == {Motor.HEAD: 1800}
        assert robot.read_servos()[Motor.HEAD] == 1800

    def test_widened_limits_warn_but_still_move(self, fake_robot, transport):
        instance = Eilik(transport=transport, limits=ServoLimits.widened(HEAD=(1000, 2000)))
        with pytest.warns(ServoRangeWarning):
            instance.write_servos({Motor.HEAD: 1950})
        assert fake_robot.servos[Motor.HEAD] == 1950

    def test_center(self, robot):
        robot.write_servos({Motor.BODY: 1300})
        assert robot.center() == dict.fromkeys(Motor, 1500)
        assert robot.read_servos() == dict.fromkeys(Motor, 1500)

    def test_all_four_motors_in_one_frame(self, robot, fake_robot):
        robot.write_servos(dict.fromkeys(Motor, 1500))
        commands = [command for command, _ in fake_robot.received]
        assert commands.count(Command.WRITE_SERVOS) == 1

    def test_failure_status_is_reported(self, robot, fake_robot, monkeypatch):
        monkeypatch.setattr(
            fake_robot,
            "_reply_for",
            lambda *_: raw_frame(Command.WRITE_SERVOS, b"\x00"),
            raising=True,
        )
        with pytest.raises(ProtocolError, match="servo write failed with status 0x00"):
            robot.write_servos({Motor.BODY: 1500})


class TestScreen:
    def test_read_returns_1024_bytes(self, robot):
        assert len(robot.read_screen()) == FRAMEBUFFER_SIZE

    def test_write_then_read_roundtrip(self, robot):
        framebuffer = blank()
        set_pixel(framebuffer, 3, 5, 1)
        set_pixel(framebuffer, 127, 63, 1)
        robot.write_screen(framebuffer)
        assert robot.read_screen() == bytes(framebuffer)

    def test_rotation_is_applied_on_the_wire(self, robot, fake_robot):
        """The bytes on the wire are the rotation of what the caller supplied.

        The panel is mounted upside down, so the SDK rotates on the way out.
        """
        framebuffer = blank()
        set_pixel(framebuffer, 0, 0, 1)
        robot.write_screen(framebuffer)
        assert get_pixel(fake_robot.framebuffer, 127, 63) == 1
        assert get_pixel(fake_robot.framebuffer, 0, 0) == 0

    def test_read_undoes_the_rotation(self, robot, fake_robot):
        stored = blank()
        set_pixel(stored, 127, 63, 1)
        fake_robot.framebuffer = bytearray(stored)
        assert get_pixel(robot.read_screen(), 0, 0) == 1

    def test_rotation_is_symmetric_on_the_wire(self, robot, fake_robot):
        framebuffer = bytes(range(256)) * 4
        robot.write_screen(framebuffer)
        assert bytes(fake_robot.framebuffer) == rotate180(framebuffer)

    def test_clear_screen(self, robot, fake_robot):
        fake_robot.framebuffer = bytearray(b"\xff" * FRAMEBUFFER_SIZE)
        robot.clear_screen()
        assert set(fake_robot.framebuffer) == {0}

    def test_wrong_size_is_refused_before_transmission(self, robot, fake_robot):
        with pytest.raises(ProtocolError, match="exactly 1024 bytes"):
            robot.write_screen(bytes(512))
        assert fake_robot.received == []

    def test_short_reply_is_refused(self, robot, fake_robot, monkeypatch):
        monkeypatch.setattr(
            fake_robot,
            "_reply_for",
            lambda *_: raw_frame(Command.READ_SCREEN, b"\x04" + bytes(100)),
            raising=True,
        )
        with pytest.raises(ProtocolError, match="expected 1025"):
            robot.read_screen()


class TestMixedCommands:
    def test_alternating_servo_and_screen_writes(self, robot, fake_robot):
        """Each exchange must complete before the next one starts.

        Interleaving the two command families is the documented way to crash the
        servo controller and drop the robot off the USB bus.
        """
        for index in range(6):
            robot.write_servos({Motor.HEAD: 1400 + index * 10})
            framebuffer = blank()
            set_pixel(framebuffer, index, index, 1)
            robot.write_screen(framebuffer)

        commands = [command for command, _ in fake_robot.received]
        assert commands == [Command.WRITE_SERVOS, Command.WRITE_SCREEN] * 6
        assert fake_robot.servos[Motor.HEAD] == 1450


class TestLifecycle:
    def test_context_manager_closes_the_transport(self, transport):
        with Eilik(transport=transport) as instance:
            assert instance.transport.is_open
        assert not transport.is_open

    def test_heartbeat(self, robot, fake_robot):
        robot.heartbeat()
        assert fake_robot.received[-1][0] == Command.ENVELOPE

    def test_repr_mentions_the_transport(self, robot):
        assert "SerialTransport" in repr(robot)

    def test_default_limits_are_the_verified_ranges(self, robot):
        assert robot.limits.ranges == ServoLimits.verified().ranges

"""End-to-end behaviour of the high-level API against a fake robot."""

from __future__ import annotations

import itertools
import time

import pytest

from eilik.errors import ProtocolError, ServoControllerFaultError, ServoRangeWarning
from eilik.protocol import HEARTBEAT_ENVELOPE_PREFIX, Command
from eilik.robot import Eilik, FirmwareInfo, _extract_text
from eilik.screen import FRAMEBUFFER_SIZE, blank, get_pixel, rotate180, set_pixel
from eilik.servo import Motor, ServoLimits, decode_servo_payload, linear

from .fake_robot import PING_PAYLOAD, device_nonce, raw_frame


class TestPing:
    def test_returns_firmware_info(self, robot):
        info = robot.ping()
        assert isinstance(info, FirmwareInfo)
        assert info.status == 0x94
        assert info.payload == PING_PAYLOAD

    def test_str_is_a_one_liner(self, robot):
        assert "\n" not in str(robot.ping())

    def test_documented_fields(self, robot):
        info = robot.ping()
        assert info.firmware_number == "4424"
        assert info.boot_firmware == "H090"
        assert info.identifier == 0x0001925B
        assert "firmware=4424" in str(info)

    def test_fields_are_none_when_the_layout_does_not_match(self):
        info = FirmwareInfo(status=0x94, payload=bytes(33), text="")
        assert info.firmware_number is None
        assert info.boot_firmware is None
        assert "firmware=" not in str(info)

    def test_short_payload_has_no_identifier(self):
        assert FirmwareInfo(status=0x94, payload=b"\x00" * 10, text="").identifier is None

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

    def test_all_zero_positions_are_reported_as_a_controller_fault(self, robot, fake_robot):
        """Zeros in every slot are the wedged-controller signature, not a reading."""
        fake_robot.servo_fault = True
        with pytest.raises(ServoControllerFaultError, match="Power-cycle") as excinfo:
            robot.read_servos()
        assert excinfo.value.positions == dict.fromkeys(Motor, 0)

    def test_writes_are_still_acknowledged_while_wedged(self, robot, fake_robot):
        """Which is why the acknowledgement alone cannot reveal the fault."""
        fake_robot.servo_fault = True
        assert robot.write_servos({Motor.BODY: 1500}) == {Motor.BODY: 1500}

    def test_a_single_zero_is_not_a_fault(self, robot, fake_robot, monkeypatch):
        reply = bytes.fromhex("04 010000 02dc05 03dc05 04dc05".replace(" ", ""))
        monkeypatch.setattr(
            fake_robot, "_reply_for", lambda *_: raw_frame(Command.READ_SERVOS, reply), raising=True
        )
        assert robot.read_servos()[Motor.ARM_RIGHT] == 0

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


def sent_positions(fake_robot) -> list[dict[Motor, int]]:
    """Return the positions carried by every 0xA2 frame the fake received."""
    return [
        decode_servo_payload(data)
        for command, data in fake_robot.received
        if command == Command.WRITE_SERVOS
    ]


class TestScreenOwnership:
    def test_write_screen_holds_the_screen_first(self, robot, fake_robot):
        robot.write_screen(blank())
        assert fake_robot.received == [
            (Command.WRITE_RUNNING_NUMBER, b"\x64"),
            (Command.WRITE_SCREEN, bytes(1024)),
        ]

    def test_hold_can_be_skipped(self, robot, fake_robot):
        robot.write_screen(blank(), hold=False)
        assert [command for command, _ in fake_robot.received] == [Command.WRITE_SCREEN]

    def test_release(self, robot, fake_robot):
        robot.release_screen()
        assert fake_robot.received == [(Command.WRITE_RUNNING_NUMBER, b"\x00")]

    def test_a_refused_hold_is_reported(self, robot, fake_robot, monkeypatch):
        monkeypatch.setattr(
            fake_robot,
            "_reply_for",
            lambda *_: raw_frame(Command.WRITE_RUNNING_NUMBER, b"\x00"),
            raising=True,
        )
        with pytest.raises(ProtocolError, match="screen hold failed"):
            robot.hold_screen()


class TestMove:
    def test_glides_to_the_target(self, robot, fake_robot):
        started = time.monotonic()
        assert robot.move({Motor.HEAD: 1700}, duration=0.2, rate=50) == {Motor.HEAD: 1700}
        assert time.monotonic() - started >= 0.18
        heads = [frame[Motor.HEAD] for frame in sent_positions(fake_robot)]
        assert len(heads) == 10
        assert heads == sorted(heads)
        assert heads[-1] == 1700
        assert fake_robot.servos[Motor.HEAD] == 1700

    def test_starts_from_the_position_read_back(self, robot, fake_robot):
        fake_robot.servos[Motor.BODY] = 1300
        robot.move({Motor.BODY: 1400}, duration=0.04, rate=100, easing=linear)
        assert fake_robot.received[0] == (Command.READ_SERVOS, b"")
        assert [frame[Motor.BODY] for frame in sent_positions(fake_robot)] == [
            1325,
            1350,
            1375,
            1400,
        ]

    def test_smoothstep_eases_in_and_out(self, robot, fake_robot):
        robot.move({Motor.ARM_LEFT: 1900}, duration=0.1, rate=100)
        positions = [1500] + [frame[Motor.ARM_LEFT] for frame in sent_positions(fake_robot)]
        steps = [b - a for a, b in itertools.pairwise(positions)]
        assert steps[0] < steps[len(steps) // 2] > steps[-1]

    def test_zero_duration_is_a_single_write(self, robot, fake_robot):
        robot.move({Motor.HEAD: 1600}, duration=0)
        assert sent_positions(fake_robot) == [{Motor.HEAD: 1600}]

    def test_unnamed_motors_are_not_sent(self, robot, fake_robot):
        robot.move({"head": 1550, 3: 1450}, duration=0.03, rate=100)
        assert all(set(frame) == {Motor.HEAD, Motor.BODY} for frame in sent_positions(fake_robot))

    def test_target_is_clamped(self, robot, fake_robot):
        assert robot.move({Motor.HEAD: 9999}, duration=0.03, rate=100) == {Motor.HEAD: 1800}
        assert max(frame[Motor.HEAD] for frame in sent_positions(fake_robot)) == 1800

    def test_warns_once_for_the_target_not_for_every_step(self, fake_robot, transport):
        instance = Eilik(transport=transport, limits=ServoLimits.widened(HEAD=(1000, 2000)))
        with pytest.warns(ServoRangeWarning) as record:
            instance.move({Motor.HEAD: 1950}, duration=0.1, rate=100)
        assert len(record) == 1
        assert fake_robot.servos[Motor.HEAD] == 1950

    def test_refuses_to_animate_a_wedged_controller(self, robot, fake_robot):
        fake_robot.servo_fault = True
        with pytest.raises(ServoControllerFaultError):
            robot.move({Motor.HEAD: 1600}, duration=0.1)
        assert sent_positions(fake_robot) == []

    @pytest.mark.parametrize(
        ("positions", "options"),
        [
            ({Motor.HEAD: 1600}, {"duration": -1}),
            ({Motor.HEAD: 1600}, {"rate": 0}),
            ({}, {}),
            ({"tail": 1500}, {}),
        ],
    )
    def test_bad_arguments_send_nothing(self, robot, fake_robot, positions, options):
        with pytest.raises(ValueError):
            robot.move(positions, **options)
        assert fake_robot.received == []


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
        assert (
            commands
            == [
                Command.WRITE_SERVOS,
                Command.WRITE_RUNNING_NUMBER,  # the screen hold before each frame
                Command.WRITE_SCREEN,
            ]
            * 6
        )
        assert fake_robot.servos[Motor.HEAD] == 1450


class TestLifecycle:
    def test_context_manager_closes_the_transport(self, transport):
        with Eilik(transport=transport) as instance:
            assert instance.transport.is_open
        assert not transport.is_open

    def test_heartbeat(self, robot, fake_robot):
        robot.heartbeat()
        assert fake_robot.received[-1] == (Command.ENVELOPE, HEARTBEAT_ENVELOPE_PREFIX + b"\xff")

    def test_heartbeat_skips_frames_that_are_not_its_reply(self, robot, fake_robot):
        fake_robot.inject_before_reply = [raw_frame(Command.READ_SERVOS, b"\x00")]
        robot.heartbeat()

    def test_heartbeat_reply_must_echo_the_subcommand(self, robot, fake_robot, monkeypatch):
        monkeypatch.setattr(
            fake_robot,
            "_reply_for",
            lambda *_: raw_frame(Command.ENVELOPE, device_nonce() + b"\x03"),
            raising=True,
        )
        with pytest.raises(ProtocolError, match="does not echo"):
            robot.heartbeat()

    def test_repr_mentions_the_transport(self, robot):
        assert "SerialTransport" in repr(robot)

    def test_default_limits_are_the_verified_ranges(self, robot):
        assert robot.limits.ranges == ServoLimits.verified().ranges

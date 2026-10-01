"""The simulated robot: servo travel, the documented crash, faults and quirks."""

from __future__ import annotations

import os
import select
import time
import tty

import pytest

from eilik.canvas import Canvas
from eilik.errors import EilikConnectionError, PortNotFoundError, ServoControllerFaultError
from eilik.protocol import Command, decode_frame, encode_frame
from eilik.robot import Eilik
from eilik.screen import rotate180
from eilik.servo import Motor
from eilik.simulator import (
    IDLE_TAKEOVER,
    SLEW_RATES,
    SimulatedEilik,
    describe_frame,
    idle_face,
    raw_frame,
    render,
)

#: A legacy single-motor servo frame from the official tooling: motor 1 to 2000.
LEGACY_SERVO_TX = bytes.fromhex("aaaaaa140061fc39e457fc03010101d007000000000041")


class RawLink:
    """A bare file descriptor on the simulator's port, with no SDK in between.

    It can do what the SDK refuses to: send without waiting for the
    acknowledgement, or send a destructive command.
    """

    def __init__(self, port: str) -> None:
        """Open ``port`` raw, without locking it."""
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)

    def send(self, payload: bytes) -> None:
        os.write(self.fd, payload)

    def read(self, timeout: float = 0.5) -> bytes:
        received = bytearray()
        deadline = time.monotonic() + timeout
        while (remaining := deadline - time.monotonic()) > 0:
            ready, _, _ = select.select([self.fd], [], [], remaining)
            if not ready:
                break
            try:
                chunk = os.read(self.fd, 4096)
            except OSError:
                break
            if not chunk:
                break
            received.extend(chunk)
            if len(received) >= 7 and len(received) >= int.from_bytes(received[3:5], "little") + 3:
                break
        return bytes(received)

    def close(self) -> None:
        os.close(self.fd)


@pytest.fixture
def sim():
    with SimulatedEilik() as instance:
        yield instance


@pytest.fixture
def raw(sim):
    link = RawLink(sim.port)
    yield link
    link.close()


class TestAsARobot:
    def test_the_sdk_drives_it_unchanged(self, sim):
        canvas = Canvas()
        canvas.text(0, 0, "Bonjour")
        with Eilik(port=sim.port) as robot:
            assert robot.ping().firmware_number == "4424"
            robot.heartbeat()
            robot.write_screen(canvas)
            assert robot.read_screen() == bytes(canvas)
        assert sim.screen() == bytes(canvas)

    def test_changes_counts_visible_updates(self, sim):
        before = sim.changes
        with Eilik(port=sim.port) as robot:
            robot.clear_screen()
            robot.write_servos({Motor.HEAD: 1600})
            robot.read_servos()
        assert sim.changes == before + 2

    def test_repr(self, sim):
        assert sim.port in repr(sim)
        assert "running" in repr(sim)


class TestServoTravel:
    def test_servos_take_time_to_arrive(self, sim):
        with Eilik(port=sim.port) as robot:
            robot.write_servos({Motor.ARM_RIGHT: 2000})
            midway = robot.read_servos()[Motor.ARM_RIGHT]
            assert 1500 <= midway < 2000
            time.sleep(500 / SLEW_RATES[Motor.ARM_RIGHT] + 0.05)
            assert robot.read_servos()[Motor.ARM_RIGHT] == 2000

    def test_body_and_head_are_slower_than_the_arms(self, sim):
        with Eilik(port=sim.port) as robot:
            robot.write_servos({Motor.ARM_LEFT: 1800, Motor.HEAD: 1800})
            time.sleep(0.12)
            positions = sim.positions()
        assert positions[Motor.ARM_LEFT] == 1800
        assert 1500 < positions[Motor.HEAD] < 1800

    def test_a_new_target_starts_from_where_the_servo_is(self, sim):
        with Eilik(port=sim.port) as robot:
            robot.write_servos({Motor.BODY: 1800})
            time.sleep(0.05)
            robot.write_servos({Motor.BODY: 1500})
            assert sim.positions()[Motor.BODY] < 1600

    def test_move_arrives(self, sim):
        with Eilik(port=sim.port) as robot:
            robot.move({Motor.HEAD: 1700, Motor.ARM_LEFT: 1200}, duration=0.3)
            time.sleep(0.2)
            assert robot.read_servos()[Motor.HEAD] == 1700

    def test_instant_mode(self):
        with SimulatedEilik(slew=False) as sim, Eilik(port=sim.port) as robot:
            robot.write_servos({Motor.HEAD: 1750})
            assert robot.read_servos()[Motor.HEAD] == 1750


class TestCrash:
    """Interleaving servo and screen frames without waiting for the ACK."""

    @pytest.mark.parametrize(
        ("first", "second"),
        [
            (Command.WRITE_SCREEN, Command.WRITE_SERVOS),
            (Command.WRITE_SERVOS, Command.WRITE_SCREEN),
        ],
    )
    def test_back_to_back_servo_and_screen_frames_crash_it(self, sim, raw, first, second):
        payloads = {Command.WRITE_SCREEN: bytes(1024), Command.WRITE_SERVOS: b"\x01\x04\xdc\x05"}
        raw.send(encode_frame(first, payloads[first]) + encode_frame(second, payloads[second]))
        deadline = time.monotonic() + 2
        while not sim.crashed and time.monotonic() < deadline:
            time.sleep(0.01)
        assert sim.crashed
        assert sim.servo_fault
        assert "without waiting for the acknowledgement" in sim.violations[0]
        assert "crashed" in repr(sim)

    def test_a_connected_sdk_sees_the_robot_drop_off_the_bus(self, sim, raw):
        with Eilik(port=sim.port, timeout=0.5) as robot:
            robot.ping()
            raw.send(
                encode_frame(Command.WRITE_SCREEN, bytes(1024))
                + encode_frame(Command.WRITE_SERVOS, b"\x01\x04\xdc\x05")
            )
            deadline = time.monotonic() + 2
            while not sim.crashed and time.monotonic() < deadline:
                time.sleep(0.01)
            with pytest.raises(EilikConnectionError):
                robot.ping()

    def test_the_port_disappears_like_a_device_node(self, sim, raw):
        """As /dev/ttyACM0 does when the robot leaves the bus."""
        raw.send(
            encode_frame(Command.WRITE_SERVOS, b"\x01\x04\xdc\x05")
            + encode_frame(Command.WRITE_SCREEN, bytes(1024))
        )
        deadline = time.monotonic() + 2
        while not sim.crashed and time.monotonic() < deadline:
            time.sleep(0.01)
        with pytest.raises(PortNotFoundError):
            Eilik(port=sim.port)

    def test_back_to_back_screen_frames_are_fine(self, sim, raw):
        """The documented crash needs the mix; screen frames alone are stable."""
        frame = encode_frame(Command.WRITE_SCREEN, bytes(1024))
        raw.send(frame + frame)
        time.sleep(0.2)
        assert not sim.crashed
        assert [command for command, _ in sim.received] == [Command.WRITE_SCREEN] * 2

    def test_waiting_for_the_ack_is_fine(self, sim):
        canvas = Canvas()
        with Eilik(port=sim.port) as robot:
            for index in range(10):
                robot.write_servos({Motor.HEAD: 1400 + 20 * index})
                canvas.pixel(index, index)
                robot.write_screen(canvas)
        assert not sim.crashed
        assert sim.violations == []

    def test_strict_mode_can_be_turned_off(self):
        with SimulatedEilik(strict=False) as lenient:
            link = RawLink(lenient.port)
            try:
                link.send(
                    encode_frame(Command.WRITE_SCREEN, bytes(1024))
                    + encode_frame(Command.WRITE_SERVOS, b"\x01\x04\xdc\x05")
                )
                time.sleep(0.2)
                assert not lenient.crashed
            finally:
                link.close()


class TestWedgedController:
    def test_reads_zero_and_ignores_writes(self):
        with SimulatedEilik(servo_fault=True) as sim, Eilik(port=sim.port) as robot:
            robot.write_servos({Motor.HEAD: 1700})  # still acknowledged
            assert sim.servos[Motor.HEAD] == 1500  # but nothing moves
            with pytest.raises(ServoControllerFaultError):
                robot.read_servos()

    def test_power_cycle_clears_it(self):
        with SimulatedEilik(servo_fault=True, slew=False) as sim, Eilik(port=sim.port) as robot:
            sim.power_cycle()
            robot.write_servos({Motor.HEAD: 1700})
            assert robot.read_servos()[Motor.HEAD] == 1700

    def test_the_screen_keeps_working(self):
        canvas = Canvas()
        canvas.rect(0, 0, 10, 10, fill=True)
        with SimulatedEilik(servo_fault=True) as sim, Eilik(port=sim.port) as robot:
            robot.write_screen(canvas)
            assert robot.read_screen() == bytes(canvas)


class TestFirmwareBehaviour:
    @pytest.mark.parametrize("opcode", [0x02, 0x04, 0x42])
    def test_destructive_commands_are_reported_and_ignored(self, sim, raw, opcode):
        raw.send(raw_frame(opcode, b"\x00"))
        assert raw.read(timeout=0.15) == b""
        assert f"0x{opcode:02X}" in sim.violations[0]
        assert "bricked" in sim.violations[0]

    def test_legacy_servo_command_moves_and_echoes(self):
        with SimulatedEilik(slew=False) as sim:
            link = RawLink(sim.port)
            try:
                link.send(LEGACY_SERVO_TX)
                reply = decode_frame(link.read())
            finally:
                link.close()
            assert sim.servos[Motor.ARM_RIGHT] == 2000
        assert reply.command == Command.ENVELOPE
        assert reply.data[5:] == b"\x03"
        assert reply.data[0] == reply.data[4]

    def test_read_running_number_answers_tagged_a4(self, raw):
        raw.send(raw_frame(0xA5))
        reply = decode_frame(raw.read())
        assert (reply.command, reply.data) == (0xA4, bytes.fromhex("0400ff00ff"))

    def test_write_running_number_is_acknowledged(self, sim, raw):
        raw.send(raw_frame(0xA6, b"\x03"))
        assert decode_frame(raw.read()).data == b"\x01"
        assert sim.changes == 0

    def test_unknown_commands_get_no_reply(self, raw):
        raw.send(raw_frame(0x20))
        assert raw.read(timeout=0.15) == b""

    def test_bad_checksum_gets_no_reply(self, sim, raw):
        frame = bytearray(encode_frame(Command.PING))
        frame[-1] ^= 0xFF
        raw.send(bytes(frame))
        assert raw.read(timeout=0.15) == b""
        assert sim.received == []

    def test_noise_before_a_frame_is_skipped(self, raw):
        # Noise ending in 0xAA could pair with the frame's own magic into a
        # plausible length; the robot never sends noise, so neither end
        # guesses its way past that.
        raw.send(b"\x00\xaa\xaa noise \x7f" + encode_frame(Command.PING))
        assert decode_frame(raw.read()).command == Command.PING


class TestLifecycle:
    def test_close_is_idempotent(self):
        sim = SimulatedEilik()
        sim.close()
        sim.close()

    def test_closing_after_unplug_does_not_close_a_reused_descriptor(self, tmp_path):
        """A descriptor number freed by unplug() can be handed to the next file."""
        sim = SimulatedEilik()
        sim.unplug()
        reused = os.open(tmp_path / "other", os.O_CREAT | os.O_RDWR)
        try:
            sim.close()
            os.fstat(reused)  # still open
        finally:
            os.close(reused)


class TestDescribeFrame:
    @pytest.mark.parametrize(
        ("command", "data", "expected"),
        [
            (Command.PING, b"", "0x01 ping"),
            (Command.WRITE_SCREEN, bytes(1024), "0xA4 write screen, 1024 bytes"),
            (Command.ENVELOPE, bytes.fromhex("e4c6f1ca83ff"), "0x61 heartbeat"),
            (0x42, b"", "0x42 format_sd"),
            (0x7B, b"", "0x7B unknown"),
        ],
    )
    def test_descriptions(self, command, data, expected):
        assert describe_frame(command, data) == expected


class TestScreenOwnership:
    """The robot's own face, the screen hold, and the two firmware behaviours."""

    @staticmethod
    def wait_past_takeover() -> None:
        time.sleep(IDLE_TAKEOVER * 3)

    @staticmethod
    def picture() -> bytes:
        canvas = Canvas()
        canvas.rect(0, 0, 128, 64)
        canvas.text(30, 28, "HOST")
        return bytes(canvas)

    def test_the_robot_shows_its_own_face_at_first(self, sim):
        assert sim.screen_owner() == "robot"
        assert any(sim.screen())
        with Eilik(port=sim.port) as robot:
            assert any(robot.read_screen())  # 0xA3 reads back what is displayed

    def test_the_face_blinks_and_glances(self):
        open_eyes, blink, glance = idle_face(1.0), idle_face(0.05), idle_face(4.0)
        assert len({open_eyes, blink, glance}) == 3
        lit = [bin(byte).count("1") for byte in open_eyes]
        assert sum(lit) > sum(bin(byte).count("1") for byte in blink)

    def test_the_sdk_keeps_its_frame_on_screen(self, sim):
        with Eilik(port=sim.port) as robot:
            robot.write_screen(self.picture())
            self.wait_past_takeover()
            assert sim.screen() == self.picture()
            assert sim.screen_owner() == "host"

    def test_a_frame_written_without_the_hold_is_painted_over(self, sim, raw):
        raw.send(encode_frame(Command.WRITE_SCREEN, rotate180(self.picture())))
        decode_frame(raw.read())
        self.wait_past_takeover()
        assert sim.screen() != self.picture()
        assert sim.screen_owner() == "robot"

    def test_holding_afterwards_does_not_bring_it_back(self, sim, raw):
        raw.send(encode_frame(Command.WRITE_SCREEN, rotate180(self.picture())))
        decode_frame(raw.read())
        self.wait_past_takeover()
        with Eilik(port=sim.port) as robot:
            robot.hold_screen()
        assert sim.screen() != self.picture()

    def test_release_brings_the_face_back(self, sim):
        with Eilik(port=sim.port) as robot:
            robot.write_screen(self.picture())
            robot.release_screen()
        assert sim.screen_owner() == "robot"
        assert not sim.screen_held

    def test_firmware_where_writing_takes_the_screen(self):
        with SimulatedEilik(hold_required=False) as lenient:
            link = RawLink(lenient.port)
            try:
                link.send(encode_frame(Command.WRITE_SCREEN, rotate180(self.picture())))
                decode_frame(link.read())
            finally:
                link.close()
            self.wait_past_takeover()
            assert lenient.screen() == self.picture()

    def test_render_says_who_owns_the_screen(self, sim):
        assert "screen: the robot's own face" in render(sim)
        with Eilik(port=sim.port) as robot:
            robot.write_screen(self.picture())
        assert "screen: the host's (held)" in render(sim)

    @pytest.mark.parametrize(
        ("data", "expected"),
        [
            (b"\x64", "0xA6 screen hold"),
            (b"\x00", "0xA6 screen release"),
            (b"\x07", "0xA6 running number 7"),
        ],
    )
    def test_frame_descriptions(self, data, expected):
        assert describe_frame(Command.WRITE_RUNNING_NUMBER, data) == expected

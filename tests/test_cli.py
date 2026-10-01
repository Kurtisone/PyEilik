"""The ``eilik`` command line, against the fake robot."""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time

import pytest

from eilik.canvas import Canvas, text_size
from eilik.cli import EXIT_SERVO_FAULT, main
from eilik.errors import PortNotFoundError, ServoControllerFaultError
from eilik.image import load_png
from eilik.protocol import Command, encode_frame
from eilik.robot import Eilik
from eilik.screen import HEIGHT, WIDTH, rotate180
from eilik.servo import Motor
from eilik.simulator import SimulatedEilik, render
from eilik.transport import default_port


def run(fake_robot, *args: str) -> int:
    return main(["--port", fake_robot.port, *args])


def on_screen(fake_robot) -> Canvas:
    """What the fake robot displays, the right way up."""
    return Canvas(rotate180(bytes(fake_robot.framebuffer)))


@pytest.fixture
def picture(tmp_path):
    """A PNG with a lit square in the left half."""
    canvas = Canvas()
    canvas.rect(10, 10, 20, 20, fill=True)
    path = tmp_path / "square.png"
    canvas.save_png(str(path), scale=1)
    return path


class TestText:
    def test_writes_centred_text(self, fake_robot):
        assert run(fake_robot, "text", "Hi") == 0
        expected = Canvas()
        # "Hi" fits at scale 3: 33x21, centred.
        expected.text((WIDTH - 33) // 2, (HEIGHT - 21) // 2, "Hi", scale=3)
        assert on_screen(fake_robot) == expected

    def test_escaped_newline_and_explicit_position(self, fake_robot):
        assert run(fake_robot, "text", r"a\nb", "--x", "0", "--y", "0", "--scale", "1") == 0
        expected = Canvas()
        expected.text(0, 0, "a\nb")
        assert on_screen(fake_robot) == expected

    def test_long_text_falls_back_to_a_smaller_scale(self, fake_robot):
        message = "x" * 21  # too wide for scale 2, just fits at scale 1
        assert text_size(message, 2)[0] > WIDTH >= text_size(message, 1)[0]
        assert run(fake_robot, "text", message) == 0
        width, height = text_size(message, 1)
        expected = Canvas()
        expected.text((WIDTH - width) // 2, (HEIGHT - height) // 2, message)
        assert on_screen(fake_robot) == expected

    def test_invert(self, fake_robot):
        assert run(fake_robot, "text", "I", "--invert") == 0
        assert on_screen(fake_robot).get_pixel(0, 0) == 1

    def test_preview_needs_no_robot(self, capsys):
        assert main(["--port", "/dev/does-not-exist", "text", "Hi", "--preview"]) == 0
        art = capsys.readouterr().out.splitlines()
        assert len(art) == HEIGHT // 2  # two pixel rows per line
        assert all(len(line) == WIDTH for line in art)
        assert any("\u2588" in line for line in art)

    @pytest.mark.parametrize("scale", ["0", "-2", "big"])
    def test_bad_scale_is_a_usage_error(self, fake_robot, scale):
        with pytest.raises(SystemExit) as excinfo:
            run(fake_robot, "text", "x", "--scale", scale)
        assert excinfo.value.code == 2


class TestShow:
    def test_shows_a_png(self, fake_robot, picture):
        assert run(fake_robot, "show", str(picture)) == 0
        screen = on_screen(fake_robot)
        assert screen.get_pixel(15, 15) == 1
        assert screen.get_pixel(60, 15) == 0

    def test_options_reach_the_converter(self, fake_robot, picture):
        assert run(fake_robot, "show", str(picture), "--invert", "--fit", "stretch") == 0
        screen = on_screen(fake_robot)
        assert screen.get_pixel(15, 15) == 0
        assert screen.get_pixel(60, 15) == 1

    def test_preview(self, picture, capsys):
        assert main(["show", str(picture), "--dither", "--preview"]) == 0
        art = capsys.readouterr().out.splitlines()
        assert art[7][15] == "\u2588"  # pixels (15, 14) and (15, 15), inside the square
        assert art[7][60] == " "

    def test_missing_file(self, fake_robot, tmp_path, capsys):
        assert run(fake_robot, "show", str(tmp_path / "nope.png")) == 1
        assert "nope.png" in capsys.readouterr().err

    def test_not_a_png(self, fake_robot, tmp_path, capsys):
        path = tmp_path / "fake.png"
        path.write_text("hello")
        assert run(fake_robot, "show", str(path)) == 1
        assert "not a PNG" in capsys.readouterr().err

    def test_bad_threshold_is_a_usage_error(self, picture):
        with pytest.raises(SystemExit) as excinfo:
            main(["show", str(picture), "--threshold", "300", "--preview"])
        assert excinfo.value.code == 2


class TestScreenCommands:
    def test_clear(self, fake_robot):
        fake_robot.framebuffer = bytearray(b"\xff" * 1024)
        assert run(fake_robot, "clear") == 0
        assert set(fake_robot.framebuffer) == {0}

    def test_capture(self, fake_robot, tmp_path, capsys):
        stored = Canvas()
        stored.pixel(0, 0)
        fake_robot.framebuffer = bytearray(rotate180(bytes(stored)))
        output = tmp_path / "shot.png"
        assert run(fake_robot, "capture", str(output), "--scale", "1", "--ascii") == 0
        assert load_png(output).pixel(0, 0) == 255
        assert capsys.readouterr().out.startswith("#.")


class TestServoCommands:
    def test_servos(self, fake_robot, capsys):
        fake_robot.servos[Motor.HEAD] = 1620
        assert run(fake_robot, "servos") == 0
        assert "HEAD=1620" in capsys.readouterr().out

    def test_move_by_name_and_id(self, fake_robot, capsys):
        assert run(fake_robot, "move", "head=1650", "3=1400", "--duration", "0") == 0
        assert fake_robot.servos[Motor.HEAD] == 1650
        assert fake_robot.servos[Motor.BODY] == 1400
        assert "HEAD=1650" in capsys.readouterr().out

    def test_move_is_clamped(self, fake_robot):
        assert run(fake_robot, "move", "ARM-LEFT=5000", "--duration", "0") == 0
        assert fake_robot.servos[Motor.ARM_LEFT] == 2000

    def test_center(self, fake_robot):
        fake_robot.servos[Motor.BODY] = 1300
        assert run(fake_robot, "center", "--duration", "0.05") == 0
        assert fake_robot.servos == dict.fromkeys(Motor, 1500)

    @pytest.mark.parametrize("spec", ["HEAD:1600", "TAIL=1500", "HEAD=up", "9=1500"])
    def test_bad_position_is_a_usage_error(self, fake_robot, spec):
        with pytest.raises(SystemExit) as excinfo:
            run(fake_robot, "move", spec)
        assert excinfo.value.code == 2

    @pytest.mark.parametrize("duration", ["-1", "nan", "soon"])
    def test_bad_duration_is_a_usage_error(self, fake_robot, duration):
        with pytest.raises(SystemExit) as excinfo:
            run(fake_robot, "center", "--duration", duration)
        assert excinfo.value.code == 2
        assert fake_robot.received == []

    def test_a_motor_named_twice_is_a_usage_error(self, fake_robot):
        with pytest.raises(SystemExit) as excinfo:
            run(fake_robot, "move", "HEAD=1600", "4=1700")
        assert excinfo.value.code == 2

    def test_wedged_controller_has_its_own_exit_status(self, fake_robot, capsys):
        fake_robot.servo_fault = True
        assert run(fake_robot, "servos") == EXIT_SERVO_FAULT
        assert "Power-cycle" in capsys.readouterr().err


class TestConnection:
    def test_unopenable_port(self, capsys):
        assert main(["--port", "/dev/does-not-exist", "clear"]) == 1
        assert "does-not-exist" in capsys.readouterr().err

    def test_busy_port(self, fake_robot, transport, capsys):
        assert transport.is_open
        assert run(fake_robot, "clear") == 1
        assert "already in use" in capsys.readouterr().err

    def test_python_dash_m(self):
        result = subprocess.run(
            [sys.executable, "-m", "eilik", "--version"],
            capture_output=True,
            text=True,
            check=False,
        )
        assert result.returncode == 0
        assert result.stdout.startswith("pyeilik ")


class TestEnvironmentPort:
    def test_eilik_port_is_used_when_no_port_is_given(self, fake_robot, monkeypatch):
        monkeypatch.setenv("EILIK_PORT", fake_robot.port)
        assert default_port() == fake_robot.port
        with Eilik() as robot:
            assert robot.transport.port == fake_robot.port
        assert main(["servos"]) == 0

    def test_an_explicit_port_wins(self, fake_robot, monkeypatch):
        monkeypatch.setenv("EILIK_PORT", "/dev/does-not-exist")
        assert run(fake_robot, "servos") == 0

    def test_the_missing_robot_message_mentions_it(self, monkeypatch):
        monkeypatch.delenv("EILIK_PORT", raising=False)
        monkeypatch.setattr("eilik.transport._acm_devices", list)
        with pytest.raises(PortNotFoundError, match="EILIK_PORT"):
            default_port()


class SimulateRun:
    """``eilik simulate --log`` running in a thread, for the length of a test."""

    def __init__(self, link, *extra: str, seconds: float = 3.0) -> None:
        """Start the simulator and wait for its link to appear."""
        self.link = link
        self.status: list[int] = []
        self.thread = threading.Thread(
            target=lambda: self.status.append(
                main(["simulate", "--log", "--link", str(link), "--for", str(seconds), *extra])
            )
        )
        self.thread.start()
        self.wait_for(link.is_symlink)

    @staticmethod
    def wait_for(condition, timeout: float = 3.0) -> None:
        deadline = time.monotonic() + timeout
        while not condition():
            assert time.monotonic() < deadline, "timed out"
            time.sleep(0.02)

    def join(self) -> int:
        self.thread.join(timeout=10)
        return self.status[0]


class TestSimulate:
    def test_commands_reach_the_simulator_through_the_link(self, tmp_path, capsys, monkeypatch):
        link = tmp_path / "sim"
        session = SimulateRun(link, "--instant", seconds=1.0)
        monkeypatch.setenv("EILIK_PORT", str(link))
        assert main(["text", "Hi"]) == 0
        assert main(["move", "HEAD=1650", "--duration", "0"]) == 0
        assert session.join() == 0
        out = capsys.readouterr().out
        assert "in another terminal: export EILIK_PORT=" in out
        assert "0xA4 write screen, 1024 bytes" in out
        assert "0xA2 write servos" in out
        assert not link.exists() and not link.is_symlink()  # cleaned up

    def test_a_crash_reboots_it_wedged_on_a_new_port(self, tmp_path, capsys):
        link = tmp_path / "sim"
        session = SimulateRun(link, seconds=2.0)
        first_port = str(link.readlink())
        fd = os.open(link, os.O_RDWR | os.O_NOCTTY)
        try:
            os.write(
                fd,
                encode_frame(Command.WRITE_SCREEN, bytes(1024))
                + encode_frame(Command.WRITE_SERVOS, b"\x01\x04\xdc\x05"),
            )
        finally:
            os.close(fd)
        session.wait_for(lambda: link.is_symlink() and str(link.readlink()) != first_port)
        with Eilik(port=str(link)) as robot, pytest.raises(ServoControllerFaultError):
            robot.read_servos()
        assert session.join() == 0
        out = capsys.readouterr().out
        assert "crashed and dropped off the bus" in out
        assert "! 0xA2 arrived while 0xA4 was still being processed" in out

    def test_refuses_to_replace_a_regular_file(self, tmp_path, capsys):
        precious = tmp_path / "notes.txt"
        precious.write_text("keep me")
        assert main(["simulate", "--link", str(precious), "--for", "0"]) == 1
        assert precious.read_text() == "keep me"
        assert "not a symlink" in capsys.readouterr().err

    def test_draws_on_a_terminal(self, tmp_path, capsys, monkeypatch):
        monkeypatch.setattr(sys.stdout, "isatty", lambda: True)
        assert main(["simulate", "--link", str(tmp_path / "sim"), "--for", "0.2"]) == 0
        out = capsys.readouterr().out
        assert out.startswith("\x1b[?1049h")  # alternate screen
        assert "Ctrl-C to stop" in out
        assert out.endswith("\x1b[?1049l")  # terminal restored


class TestRender:
    def test_layout(self):
        with SimulatedEilik(slew=False) as sim:
            picture = render(sim, "title here")
        lines = picture.split("\n")
        assert "title here" in lines[0]
        assert all(len(line) == WIDTH + 2 for line in lines[:34])
        assert [line.split()[0] for line in lines[34:38]] == [motor.name for motor in Motor]
        assert "running" in lines[38]

    def test_reports_faults_and_violations(self):
        with SimulatedEilik(servo_fault=True) as sim:
            sim.violations.append("something bad")
            picture = render(sim)
        assert "WEDGED" in picture
        assert " ! something bad" in picture
        assert "HEAD         0" in picture

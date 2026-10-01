"""The bring-up probe, run against the simulated robot."""

from __future__ import annotations

import pytest

import probe
from eilik.protocol import Command
from eilik.transport import SerialTransport


def run_probe(fake_robot, capsys, *extra: str) -> tuple[int, str]:
    code = probe.main(["--port", fake_robot.port, "--no-png", "--timeout", "0.5", *extra])
    return code, capsys.readouterr().out


def test_healthy_robot(fake_robot, capsys):
    code, out = run_probe(fake_robot, capsys)
    assert code == 0
    assert "firmware: 4424" in out
    assert "echoed" in out
    assert "ARM_RIGHT=1500" in out
    assert "all requested commands completed" in out


def test_a_silent_heartbeat_is_not_fatal(fake_robot, capsys, monkeypatch):
    """Some firmware ignores the heartbeat until the official app has run."""
    answer = fake_robot._reply_for

    def ignore_heartbeat(command, data):
        return None if command == Command.ENVELOPE else answer(command, data)

    monkeypatch.setattr(fake_robot, "_reply_for", ignore_heartbeat)
    code, out = run_probe(fake_robot, capsys)
    assert code == 0
    assert "no reply; some firmware versions" in out
    assert "ARM_RIGHT=1500" in out


def test_wedged_servo_controller(fake_robot, capsys):
    fake_robot.servo_fault = True
    code, out = run_probe(fake_robot, capsys, "--move", "--yes")
    assert code == 3
    assert "FAULT" in out
    assert "skipping the test movement" in out
    assert "screen read" in out  # the display still works, so it is still read


def test_movement_returns_to_the_start(fake_robot, capsys):
    fake_robot.servos[next(iter(fake_robot.servos))] = 1400
    start = dict(fake_robot.servos)
    code, _ = run_probe(fake_robot, capsys, "--move", "--yes")
    assert code == 0
    assert fake_robot.servos == start


def test_busy_port(fake_robot, capsys):
    with SerialTransport(port=fake_robot.port):
        code, out = run_probe(fake_robot, capsys)
    assert code == 1
    assert "already in use" in out


@pytest.mark.parametrize("flag", ["--list"])
def test_listing_ports_needs_no_robot(capsys, flag):
    assert probe.main([flag]) in (0, 1)  # 1 when the machine has no serial port
    assert "serial ports" in capsys.readouterr().out

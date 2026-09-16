"""Shared fixtures."""

from __future__ import annotations

import pytest

from eilik.robot import Eilik
from eilik.transport import SerialTransport

from .fake_robot import FakeEilik


@pytest.fixture
def fake_robot():
    """Yield a fake Eilik listening on a pty."""
    robot = FakeEilik()
    try:
        yield robot
    finally:
        robot.close()


@pytest.fixture
def transport(fake_robot):
    """Yield a transport connected to the fake robot."""
    link = SerialTransport(port=fake_robot.port, timeout=2.0)
    try:
        yield link
    finally:
        link.close()


@pytest.fixture
def robot(transport):
    """Yield a high-level Eilik connected to the fake robot."""
    with Eilik(transport=transport) as instance:
        yield instance

"""The robot the test suite talks to: the simulator, with instant servos.

Instant servos and a screen without the robot's own animated face keep
write-then-read assertions exact. Strict sequencing stays on, so a transport
regression that let a servo frame and a screen frame interleave would crash
the fake and fail the tests, as it would the robot.
"""

from __future__ import annotations

from eilik.simulator import PING_PAYLOAD, SimulatedEilik, device_nonce, raw_frame

__all__ = ["PING_PAYLOAD", "FakeEilik", "device_nonce", "raw_frame"]


class FakeEilik(SimulatedEilik):
    """A simulator whose servos move instantly and whose screen keeps what is written."""

    def __init__(self) -> None:
        """Start a simulator without servo travel time."""
        super().__init__(slew=False, idle_face=False)

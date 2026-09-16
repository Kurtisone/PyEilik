"""Exception and warning types raised by the Eilik SDK."""

from __future__ import annotations

__all__ = [
    "AmbiguousPortError",
    "BlacklistedCommandError",
    "ChecksumError",
    "EilikError",
    "EilikTimeoutError",
    "FrameError",
    "PortNotFoundError",
    "ProtocolError",
    "ServoRangeWarning",
    "UnsupportedCommandError",
]


class EilikError(Exception):
    """Base class for every error raised by this package."""


class ProtocolError(EilikError):
    """A frame could not be encoded or decoded."""


class FrameError(ProtocolError):
    """A frame is structurally invalid (bad magic, bad length, truncated)."""


class ChecksumError(ProtocolError):
    """A received frame carried a checksum that does not match its body.

    The firmware silently drops frames whose checksum is wrong, so a checksum
    mismatch on the receive path means the link corrupted the data rather than
    that the robot rejected the request.
    """

    def __init__(self, expected: int, actual: int) -> None:
        """Record the expected and actual checksum bytes."""
        super().__init__(f"checksum mismatch: expected 0x{expected:02X}, got 0x{actual:02X}")
        self.expected = expected
        self.actual = actual


class BlacklistedCommandError(EilikError):
    """A destructive command was refused before it could reach the wire.

    This is a safety refusal, not a protocol failure: the opcode is perfectly
    valid as far as the firmware is concerned, which is precisely the problem.
    """

    def __init__(self, command: int, reason: str) -> None:
        """Record the refused opcode and why it is refused."""
        super().__init__(
            f"command 0x{command:02X} is blacklisted and will never be sent by this SDK: {reason}"
        )
        self.command = command
        self.reason = reason


class UnsupportedCommandError(EilikError):
    """An opcode outside the SDK's allowlist was passed to the transport."""

    def __init__(self, command: int) -> None:
        """Record the unsupported opcode."""
        super().__init__(
            f"command 0x{command:02X} is not part of the supported command set; "
            "add it to eilik.protocol.Command after verifying it is non-destructive"
        )
        self.command = command


class EilikTimeoutError(EilikError, TimeoutError):
    """The robot did not answer within the allotted time."""


class PortNotFoundError(EilikError):
    """No serial port matching an Eilik robot could be found."""


class AmbiguousPortError(EilikError):
    """Several candidate serial ports were found and none could be preferred."""

    def __init__(self, candidates: list[str]) -> None:
        """Record the candidate device paths."""
        super().__init__(
            "several candidate serial ports found, pass one explicitly: " + ", ".join(candidates)
        )
        self.candidates = list(candidates)


class ServoRangeWarning(UserWarning):
    """A requested servo position fell outside the empirically verified range."""

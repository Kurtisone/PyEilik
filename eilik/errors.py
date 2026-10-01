"""Exception and warning types raised by the Eilik SDK."""

from __future__ import annotations

__all__ = [
    "AmbiguousPortError",
    "BlacklistedCommandError",
    "ChecksumError",
    "EilikConnectionError",
    "EilikError",
    "EilikTimeoutError",
    "FrameError",
    "ImageError",
    "PortBusyError",
    "PortNotFoundError",
    "ProtocolError",
    "ServoControllerFaultError",
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


class ImageError(EilikError, ValueError):
    """A picture could not be decoded or does not have a usable shape."""


class EilikTimeoutError(EilikError, TimeoutError):
    """The robot did not answer within the allotted time."""


class EilikConnectionError(EilikError, ConnectionError):
    """The serial link failed underneath an exchange.

    The usual cause is the robot dropping off the USB bus, which the kernel
    reports as ``ENXIO``. The documented trigger is a servo command and a screen
    command interleaving without waiting for the acknowledgement in between; the
    robot then reboots and re-enumerates, so the port has to be reopened.
    """

    def __init__(self, port: str, cause: BaseException) -> None:
        """Record the port and the low-level error that broke the link."""
        super().__init__(
            f"lost the serial link on {port}: {cause}. If the robot dropped off the USB bus, "
            "it reboots and comes back; reopen the port. If servo reads then come back as all "
            "zeros, power-cycle it with the switch on the body"
        )
        self.port = port
        self.cause = cause


class PortBusyError(EilikError):
    """Something else already holds the robot's serial port.

    The port is opened with an exclusive lock because two programs driving the
    robot at once can interleave a servo frame with a screen frame, which is the
    documented way to crash its servo controller.
    """

    def __init__(self, port: str) -> None:
        """Record the busy port."""
        super().__init__(
            f"{port} is already in use, by another program or another connection in this one; "
            f"two connections driving the robot at once can interleave frames and crash its "
            f"servo controller. Check with: fuser -v {port}"
        )
        self.port = port


class ServoControllerFaultError(EilikError):
    """The servo controller reported every position as zero.

    That is the fault signature of a wedged servo controller, typically after
    the robot has crashed from interleaved commands: 0xA2 is still acknowledged
    and 0xA1 still answers with the right motor ids, but every position reads
    zero and nothing moves. The display keeps working, so the rest of the robot
    looks healthy.

    Reconnecting does not clear it. Eilik has an internal battery, so unplugging
    the USB cable leaves the controller running and still wedged; it takes a
    power cycle with the switch on the body.
    """

    def __init__(self, positions: dict) -> None:
        """Record the all-zero reading."""
        super().__init__(
            "every servo position reads zero, the signature of a wedged servo controller. "
            "Power-cycle the robot with the switch on its body; unplugging USB is not enough "
            "because it runs on its internal battery"
        )
        self.positions = dict(positions)


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

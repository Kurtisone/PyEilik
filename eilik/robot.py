"""High-level API for driving an Eilik robot."""

from __future__ import annotations

import logging
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from . import screen as screen_module
from .errors import ProtocolError
from .protocol import Command, encode_heartbeat
from .servo import (
    Motor,
    ServoLimits,
    decode_servo_payload,
    encode_servo_payload,
    neutral_positions,
)
from .transport import DEFAULT_BAUDRATE, SerialTransport

__all__ = ["Eilik", "FirmwareInfo"]

_log = logging.getLogger(__name__)

#: Status byte the firmware returns in a 0xA2 / 0xA4 acknowledgement on success.
STATUS_OK = 0x01


@dataclass(frozen=True)
class FirmwareInfo:
    """What a 0x01 ping reply tells us.

    Only what is needed to confirm the link is parsed. The remainder of the
    payload is kept verbatim in :attr:`payload` so callers can dig further
    without the SDK having to guess at field offsets it has not verified.

    Attributes:
        status: Leading byte of the reply's data field.
        payload: The rest of the data field, 33 bytes on observed firmware.
        text: Printable ASCII runs of at least four characters found in the
            payload, joined by spaces. Best effort, for display only.
    """

    status: int
    payload: bytes
    text: str

    def __str__(self) -> str:
        """Return a one-line summary."""
        return f"status=0x{self.status:02X} bytes={len(self.payload)} text={self.text or '-'!r}"


def _extract_text(payload: bytes, minimum_run: int = 4) -> str:
    """Return printable ASCII runs of at least ``minimum_run`` characters."""
    runs, current = [], bytearray()
    for byte in payload:
        if 0x20 <= byte < 0x7F:
            current.append(byte)
            continue
        if len(current) >= minimum_run:
            runs.append(current.decode("ascii"))
        current.clear()
    if len(current) >= minimum_run:
        runs.append(current.decode("ascii"))
    return " ".join(runs)


class Eilik:
    """A connected Eilik robot.

    Every exchange waits for the firmware's acknowledgement before returning, so
    consecutive calls are automatically serialised. That matters: sending a
    screen command while a servo command is still being processed crashes the
    servo controller and the robot drops off the USB bus with ``ENXIO``.

    Example:
        >>> with Eilik() as robot:  # doctest: +SKIP
        ...     print(robot.ping())
        ...     robot.write_servos({Motor.HEAD: 1600})
        ...     framebuffer = robot.read_screen()

    Args:
        port: Device path; ``None`` auto-detects a single ``/dev/ttyACM*``.
        baudrate: Nominal line rate, see :mod:`eilik.transport`.
        timeout: Seconds to wait for a reply.
        limits: Servo position limits. Defaults to the verified ranges.
        transport: An already-built transport, mainly for tests.
    """

    def __init__(
        self,
        port: str | None = None,
        baudrate: int = DEFAULT_BAUDRATE,
        timeout: float = 1.0,
        limits: ServoLimits | None = None,
        transport: SerialTransport | None = None,
    ) -> None:
        """Open the link to the robot."""
        self.transport = transport or SerialTransport(
            port=port, baudrate=baudrate, timeout=timeout
        )
        self.limits = limits or ServoLimits.verified()

    # -- lifecycle ---------------------------------------------------------

    def close(self) -> None:
        """Close the serial link."""
        self.transport.close()

    def __enter__(self) -> Eilik:
        """Return self, for use as a context manager."""
        return self

    def __exit__(self, *exc_info: object) -> None:
        """Close the link on exit."""
        self.close()

    def __repr__(self) -> str:
        """Return a short description of the connection."""
        return f"Eilik({self.transport!r})"

    # -- read-only commands ------------------------------------------------

    def ping(self, timeout: float | None = None) -> FirmwareInfo:
        """Identify the firmware and confirm the link is alive.

        Args:
            timeout: Seconds to wait for the reply.

        Returns:
            The decoded reply.

        Raises:
            EilikTimeoutError: If the robot did not answer.
            ProtocolError: If the reply carried no data.
        """
        frame = self.transport.request(Command.PING, timeout=timeout)
        if not frame.data:
            raise ProtocolError("ping reply carried no data")
        return FirmwareInfo(
            status=frame.data[0],
            payload=frame.data[1:],
            text=_extract_text(frame.data[1:]),
        )

    def heartbeat(self, timeout: float | None = None) -> None:
        """Send a keep-alive heartbeat and wait for its acknowledgement.

        Args:
            timeout: Seconds to wait for the reply.

        Raises:
            EilikTimeoutError: If the robot did not answer.
        """
        # Built through the encoder so the nested sub-command is validated, then
        # handed to the transport, which re-checks the outer opcode.
        frame = encode_heartbeat()
        deadline_timeout = timeout if timeout is not None else self.transport.timeout
        self.transport.send_frame(frame)
        self.transport.read_frame(timeout=deadline_timeout)

    def read_servos(self, timeout: float | None = None) -> dict[Motor, int]:
        """Read the current position of every servo.

        Args:
            timeout: Seconds to wait for the reply.

        Returns:
            Motor to pulse width.

        Raises:
            EilikTimeoutError: If the robot did not answer.
            ProtocolError: If the payload is malformed.
        """
        frame = self.transport.request(Command.READ_SERVOS, timeout=timeout)
        return decode_servo_payload(frame.data)

    def read_screen(self, timeout: float | None = None) -> bytes:
        """Read the display framebuffer.

        Args:
            timeout: Seconds to wait for the reply. Screen reads move 1032
                bytes, so allow more than for a servo read on a slow link.

        Returns:
            A 1024-byte framebuffer, already rotated the right way up.

        Raises:
            EilikTimeoutError: If the robot did not answer.
            ProtocolError: If the reply is not 1024 bytes of framebuffer.
        """
        frame = self.transport.request(Command.READ_SCREEN, timeout=timeout)
        # The payload is a leading status byte followed by the framebuffer.
        if len(frame.data) != screen_module.FRAMEBUFFER_SIZE + 1:
            raise ProtocolError(
                f"screen read returned {len(frame.data)} bytes of payload, expected "
                f"{screen_module.FRAMEBUFFER_SIZE + 1}"
            )
        return screen_module.rotate180(frame.data[1:])

    # -- write commands ----------------------------------------------------

    def write_servos(
        self,
        positions: Mapping[object, int],
        timeout: float | None = None,
    ) -> dict[Motor, int]:
        """Move up to four servos in a single frame.

        Positions are clamped to the configured limits before transmission, and
        a :class:`~eilik.errors.ServoRangeWarning` is raised for any position
        that ends up outside the empirically verified range.

        Args:
            positions: Motor (a :class:`Motor`, an id or a name) to pulse width.
            timeout: Seconds to wait for the acknowledgement.

        Returns:
            The positions actually sent, after clamping.

        Raises:
            ValueError: If no motors, more than four, or an unknown motor.
            EilikTimeoutError: If the robot did not acknowledge.
            ProtocolError: If the acknowledgement reports a failure.
        """
        payload = encode_servo_payload(positions, self.limits)
        frame = self.transport.request(Command.WRITE_SERVOS, payload, timeout=timeout)
        self._check_status(frame.data, "servo write")
        return decode_servo_payload(payload)

    def write_screen(self, framebuffer: Sequence[int], timeout: float | None = None) -> None:
        """Replace the display contents.

        Args:
            framebuffer: A 1024-byte buffer the right way up; the 180-degree
                rotation the panel needs is applied here.
            timeout: Seconds to wait for the acknowledgement.

        Raises:
            ProtocolError: If the buffer is not 1024 bytes, or the robot
                reported a failure.
            EilikTimeoutError: If the robot did not acknowledge.
        """
        rotated = screen_module.rotate180(bytes(framebuffer))
        frame = self.transport.request(Command.WRITE_SCREEN, rotated, timeout=timeout)
        self._check_status(frame.data, "screen write")

    @staticmethod
    def _check_status(data: bytes, what: str) -> None:
        """Raise if an acknowledgement payload does not report success.

        Raises:
            ProtocolError: If the payload is empty or the status is not
                :data:`STATUS_OK`.
        """
        if not data:
            raise ProtocolError(f"{what} acknowledgement carried no status byte")
        if data[0] != STATUS_OK:
            raise ProtocolError(f"{what} failed with status 0x{data[0]:02X}")

    # -- convenience -------------------------------------------------------

    def center(self, timeout: float | None = None) -> dict[Motor, int]:
        """Return every motor to its neutral position.

        Args:
            timeout: Seconds to wait for the acknowledgement.

        Returns:
            The positions sent.
        """
        return self.write_servos(neutral_positions(), timeout=timeout)

    def clear_screen(self, timeout: float | None = None) -> None:
        """Blank the display.

        Args:
            timeout: Seconds to wait for the acknowledgement.
        """
        self.write_screen(screen_module.blank(), timeout=timeout)

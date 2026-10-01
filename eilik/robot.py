"""High-level API for driving an Eilik robot."""

from __future__ import annotations

import logging
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass

from . import screen as screen_module
from .canvas import Canvas
from .errors import ProtocolError, ServoControllerFaultError
from .protocol import HEARTBEAT_ENVELOPE_PREFIX, SUBCOMMAND_HEARTBEAT, Command
from .servo import (
    Motor,
    ServoLimits,
    decode_servo_payload,
    encode_servo_payload,
    neutral_positions,
    resolve_positions,
    smoothstep,
)
from .transport import DEFAULT_BAUDRATE, SerialTransport

__all__ = ["Eilik", "FirmwareInfo"]

_log = logging.getLogger(__name__)

#: Default number of position updates per second during :meth:`Eilik.move`.
#: The servos cannot follow much more, and the reference player sends motion at
#: 10 Hz; 20 keeps short moves smooth while leaving the link mostly idle.
DEFAULT_MOVE_RATE = 20.0

#: Status byte the firmware returns in a 0xA2 / 0xA4 acknowledgement on success.
STATUS_OK = 0x01


def _ascii_field(payload: bytes, start: int, end: int) -> str | None:
    """Return ``payload[start:end]`` as text if it is all printable, else None."""
    field = payload[start:end]
    if len(field) != end - start or not all(0x21 <= byte < 0x7F for byte in field):
        return None
    return field.decode("ascii")


@dataclass(frozen=True)
class FirmwareInfo:
    """What a 0x01 ping reply tells us.

    The payload is kept verbatim in :attr:`payload`. On the firmware documented
    in the community protocol reference it looks like this, with the field names
    taken from the manufacturer's own table::

        offset 1..4    "4424"       probably firmware_number
        offset 7..10   "H090"       probably boot_firmware
        offset 11..14  u32 LE       looks like an identifier

    That layout has been seen on one device but not pinned down, so the
    properties exposing it are best effort: each returns None rather than
    guessing when its bytes do not look like what was observed. Compare these
    identifiers first when the robot behaves differently from someone else's,
    since some commands differ between firmware versions.

    Attributes:
        status: Leading byte of the reply's data field (0x94 when observed).
        payload: The rest of the data field, 33 bytes on observed firmware.
        text: Printable ASCII runs of at least four characters found in the
            payload, joined by spaces. Best effort, for display only.
    """

    status: int
    payload: bytes
    text: str

    @property
    def firmware_number(self) -> str | None:
        """Probable firmware number, ``"4424"`` on the documented device."""
        return _ascii_field(self.payload, 1, 5)

    @property
    def boot_firmware(self) -> str | None:
        """Probable boot firmware, ``"H090"`` on the documented device."""
        return _ascii_field(self.payload, 7, 11)

    @property
    def identifier(self) -> int | None:
        """The u32 at payload offset 11 that looks like an identifier."""
        if len(self.payload) < 15:
            return None
        return int.from_bytes(self.payload[11:15], "little")

    def __str__(self) -> str:
        """Return a one-line summary."""
        parts = [f"status=0x{self.status:02X}"]
        if self.firmware_number is not None:
            parts.append(f"firmware={self.firmware_number}")
        if self.boot_firmware is not None:
            parts.append(f"boot={self.boot_firmware}")
        parts.append(f"bytes={len(self.payload)}")
        parts.append(f"text={self.text or '-'!r}")
        return " ".join(parts)


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
        self.transport = transport or SerialTransport(port=port, baudrate=baudrate, timeout=timeout)
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
        """Send a heartbeat and wait for the robot to echo it.

        This is the cheapest link check there is. It is not a keep-alive: the
        protocol has no session and the link does not go stale, so an idle
        connection needs nothing sent on it.

        Args:
            timeout: Seconds to wait for the reply.

        Raises:
            EilikTimeoutError: If the robot did not answer.
            ProtocolError: If the reply does not echo the heartbeat.
        """
        # Through request(), like every other exchange, so the transport lock is
        # held from the write until the matching reply has been read.
        frame = self.transport.request(
            Command.ENVELOPE,
            HEARTBEAT_ENVELOPE_PREFIX + bytes([SUBCOMMAND_HEARTBEAT]),
            timeout=timeout,
        )
        # The reply carries a fresh nonce of the firmware's own, then the echo.
        expected_size = len(HEARTBEAT_ENVELOPE_PREFIX) + 1
        if len(frame.data) != expected_size or frame.data[-1] != SUBCOMMAND_HEARTBEAT:
            raise ProtocolError(f"heartbeat reply does not echo 0xFF: {frame!r}")

    def read_servos(self, timeout: float | None = None) -> dict[Motor, int]:
        """Read the current position of every servo.

        Args:
            timeout: Seconds to wait for the reply.

        Returns:
            Motor to pulse width.

        Raises:
            EilikTimeoutError: If the robot did not answer.
            ProtocolError: If the payload is malformed.
            ServoControllerFaultError: If every position reads zero, which is
                the signature of a wedged servo controller rather than a real
                reading. It needs a power cycle, not a reconnect.
        """
        frame = self.transport.request(Command.READ_SERVOS, timeout=timeout)
        positions = decode_servo_payload(frame.data)
        if positions and not any(positions.values()):
            raise ServoControllerFaultError(positions)
        return positions

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
            The positions actually sent, after clamping. The acknowledgement
            only proves the frame arrived intact (the firmware acknowledges a
            non-existent motor id just the same), so read the positions back
            with :meth:`read_servos` to confirm a move.

        Raises:
            ValueError: If no motors, more than four, or an unknown motor.
            EilikTimeoutError: If the robot did not acknowledge.
            ProtocolError: If the acknowledgement reports a failure.
        """
        payload = encode_servo_payload(positions, self.limits)
        frame = self.transport.request(Command.WRITE_SERVOS, payload, timeout=timeout)
        self._check_status(frame.data, "servo write")
        return decode_servo_payload(payload)

    def move(
        self,
        positions: Mapping[object, int],
        duration: float = 0.5,
        rate: float = DEFAULT_MOVE_RATE,
        easing: Callable[[float], float] = smoothstep,
        timeout: float | None = None,
    ) -> dict[Motor, int]:
        """Glide servos to new positions over ``duration`` seconds.

        The movement starts from the positions read back with
        :meth:`read_servos`, so it refuses to run on a wedged servo controller
        rather than animate a robot that will not move. Targets are clamped
        once, up front, which is also when any
        :class:`~eilik.errors.ServoRangeWarning` is raised. The intermediate
        positions then go out ``rate`` times a second on a fixed schedule, so a
        slow exchange does not stretch the movement, and each waits for its
        acknowledgement like any other write.

        Motors not named in ``positions`` are not sent and hold still.

        Args:
            positions: Motor (a :class:`Motor`, an id or a name) to target.
            duration: Seconds the movement should take; 0 sends the targets in
                a single frame.
            rate: Position updates per second.
            easing: Maps progress in 0..1 to the fraction of the way covered.
                :func:`~eilik.servo.smoothstep` (the default) starts and stops
                gently; :func:`~eilik.servo.linear` keeps a constant speed.
            timeout: Seconds to wait for each reply.

        Returns:
            The target positions, after clamping.

        Raises:
            ValueError: If ``duration`` is negative, ``rate`` is not positive,
                or ``positions`` is invalid as for :meth:`write_servos`.
            ServoControllerFaultError: If the servo controller is wedged.
            EilikTimeoutError: If the robot stops answering.
        """
        if duration < 0:
            raise ValueError(f"duration must not be negative, got {duration}")
        if rate <= 0:
            raise ValueError(f"rate must be positive, got {rate}")

        targets = resolve_positions(positions, self.limits)
        start = self.read_servos(timeout=timeout)
        steps = max(1, round(duration * rate))
        interval = duration / steps
        began = time.monotonic()

        for step in range(1, steps + 1):
            delay = began + step * interval - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            fraction = easing(step / steps) if step < steps else 1.0
            frame = {
                motor: round(
                    start.get(motor, target) + (target - start.get(motor, target)) * fraction
                )
                for motor, target in targets.items()
            }
            # The targets were checked above; the steps between them and the
            # starting point need no warning of their own.
            payload = encode_servo_payload(frame, self.limits, warn=False)
            reply = self.transport.request(Command.WRITE_SERVOS, payload, timeout=timeout)
            self._check_status(reply.data, "servo write")
        return targets

    def write_screen(
        self, framebuffer: Sequence[int] | Canvas, timeout: float | None = None
    ) -> None:
        """Replace the display contents.

        Args:
            framebuffer: A 1024-byte buffer the right way up, or a
                :class:`~eilik.canvas.Canvas`; the 180-degree rotation the panel
                needs is applied here.
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

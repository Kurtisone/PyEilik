"""Serial transport: port discovery, custom baud rate, framing and ACK waiting.

About the baud rate
-------------------

The robot enumerates as a USB CDC-ACM device. The 125000 baud figure that the
protocol documentation quotes is nominal: CDC-ACM carries a line-coding request
to the device but the bytes themselves travel at USB speed, so the termios rate
has no effect on throughput. It is set anyway, to match what the vendor tooling
does, but failing to set it is not fatal.

125000 is not one of the standard POSIX rates. On Linux the kernel exposes
arbitrary rates through the ``TCSETS2`` ioctl with the ``BOTHER`` flag, and
pyserial >= 3.0 already uses that path, so no out-of-tree hack is needed (unlike
macOS, where the vendor SDK reaches for ``IOSSIOSPEED``). This module verifies
the rate actually took by reading it back with ``TCGETS2``, and re-applies it
directly if pyserial's attempt did not stick.
"""

from __future__ import annotations

import array
import contextlib
import errno
import fcntl
import logging
import sys
import termios
import threading
import time
from collections.abc import Iterator
from pathlib import Path

import serial
from serial.tools import list_ports

from .errors import (
    AmbiguousPortError,
    ChecksumError,
    EilikConnectionError,
    EilikTimeoutError,
    FrameError,
    PortBusyError,
    PortNotFoundError,
)
from .protocol import (
    MAGIC,
    MAX_FRAME_SIZE,
    MIN_LENGTH_FIELD,
    Frame,
    decode_frame,
    encode_frame,
    ensure_frame_allowed,
)

__all__ = [
    "DEFAULT_BAUDRATE",
    "SerialTransport",
    "autodetect_port",
    "list_candidate_ports",
]

_log = logging.getLogger(__name__)

#: Nominal line rate. See the module docstring: the value is not load-bearing.
DEFAULT_BAUDRATE = 125000

#: Directory and glob pattern matching the CDC-ACM nodes the robot shows up on.
ACM_DIRECTORY = Path("/dev")
ACM_PATTERN = "ttyACM*"
ACM_GLOB = f"{ACM_DIRECTORY}/{ACM_PATTERN}"

#: Seconds to wait between a reply and the next transmission. The servo
#: controller crashes and drops off the USB bus if a screen command lands while
#: it is still processing a servo command, so frames are strictly serialised and
#: separated by this pause on top of waiting for the reply.
DEFAULT_INTER_FRAME_DELAY = 0.005

#: ``TCGETS2`` / ``TCSETS2`` ioctl numbers and the ``BOTHER`` cflag, used for the
#: direct termios2 path. These are ABI constants on Linux, but they are not
#: exposed by the :mod:`termios` module, so they are spelled out here.
_TCGETS2 = 0x802C542A
_TCSETS2 = 0x402C542B
_BOTHER = 0o010000
_CBAUD = 0o010017

#: Index of ``c_cflag`` / ``c_ispeed`` / ``c_ospeed`` in ``struct termios2``
#: viewed as an array of C ints.
_CFLAG_INDEX = 2
_ISPEED_INDEX = 9
_OSPEED_INDEX = 10

#: errno values meaning something else holds the port: a failed exclusive
#: ``flock`` reports EWOULDBLOCK (EAGAIN on Linux), a busy device EBUSY.
_BUSY_ERRNOS = frozenset({errno.EAGAIN, errno.EWOULDBLOCK, errno.EBUSY})

#: What the underlying serial port raises when the device goes away mid-use:
#: pyserial's own exception (an OSError), plain OSError (ENXIO, EIO) and, from
#: the input-flush path, termios.error.
_LINK_ERRORS = (OSError, termios.error)


def _acm_devices() -> list[str]:
    """Return the CDC-ACM device paths present on this machine, sorted."""
    try:
        return sorted(str(device) for device in ACM_DIRECTORY.glob(ACM_PATTERN))
    except OSError:
        return []


def list_candidate_ports() -> list[str]:
    """Return the serial device paths that could be an Eilik, best first.

    CDC-ACM nodes are listed first because that is what the robot enumerates as;
    anything else pyserial knows about follows, so that a machine using a
    different naming scheme still offers something to pick from.
    """
    acm = _acm_devices()
    others = [port.device for port in list_ports.comports() if port.device not in acm]
    return acm + sorted(others)


def describe_ports() -> list[str]:
    """Return one human-readable line per known serial port, for diagnostics."""
    known = {port.device: port for port in list_ports.comports()}
    lines = []
    for device in list_candidate_ports():
        info = known.get(device)
        if info is None:
            lines.append(f"{device}  (no USB metadata)")
            continue
        has_ids = info.vid is not None and info.pid is not None
        vid_pid = f"{info.vid:04x}:{info.pid:04x}" if has_ids else "-"
        lines.append(f"{device}  {vid_pid}  {info.description}")
    return lines


def autodetect_port() -> str:
    """Return the single obvious Eilik serial port.

    Returns:
        The device path.

    Raises:
        PortNotFoundError: If no CDC-ACM node is present.
        AmbiguousPortError: If several are, in which case the caller has to pick
            one; :func:`describe_ports` produces the listing to choose from.
    """
    candidates = _acm_devices()
    if not candidates:
        raise PortNotFoundError(
            f"no device matching {ACM_GLOB}; plug the robot in and check `dmesg | tail`"
        )
    if len(candidates) > 1:
        raise AmbiguousPortError(candidates)
    return candidates[0]


def _read_termios2_speed(fd: int) -> int | None:
    """Return the output speed currently configured on ``fd``, or None.

    None means the query is unavailable (non-Linux, or a device that does not
    implement the ioctl), which is not an error: the baud rate is nominal.
    """
    if not sys.platform.startswith("linux"):
        return None
    buf = array.array("i", [0] * 64)
    try:
        fcntl.ioctl(fd, _TCGETS2, buf, True)
    except OSError:
        return None
    return buf[_OSPEED_INDEX]


def _apply_termios2_speed(fd: int, baudrate: int) -> bool:
    """Set a non-standard baud rate on ``fd`` via ``TCSETS2``/``BOTHER``.

    This is the low-level fallback for the case where pyserial's own custom-rate
    handling is unavailable or does not stick. It is the standard kernel
    mechanism, not a vendor-specific hack.

    Args:
        fd: An open file descriptor for the tty.
        baudrate: The rate to request.

    Returns:
        True if the rate was applied and read back correctly.
    """
    if not sys.platform.startswith("linux"):
        return False
    buf = array.array("i", [0] * 64)
    try:
        fcntl.ioctl(fd, _TCGETS2, buf, True)
        buf[_CFLAG_INDEX] = (buf[_CFLAG_INDEX] & ~_CBAUD) | _BOTHER
        buf[_ISPEED_INDEX] = baudrate
        buf[_OSPEED_INDEX] = baudrate
        fcntl.ioctl(fd, _TCSETS2, buf, True)
    except OSError as exc:
        _log.debug("termios2 baud fallback failed: %s", exc)
        return False
    return _read_termios2_speed(fd) == baudrate


class SerialTransport:
    """Frame-level serial link to the robot.

    The transport owns the destructive-command guard on the write path and the
    "one frame in flight at a time" discipline that keeps the servo controller
    from crashing. Every public method is safe to call from multiple threads:
    :meth:`request` holds a lock across the whole send/receive exchange.

    Args:
        port: Device path. ``None`` runs :func:`autodetect_port`.
        baudrate: Nominal line rate; see the module docstring.
        timeout: Default seconds to wait for a reply.
        inter_frame_delay: Minimum pause between consecutive transmissions.
        exclusive: Take an exclusive lock on the port, so that a second program
            cannot open the same robot and interleave its frames with ours.

    Raises:
        PortNotFoundError: If ``port`` is None and no robot is attached.
        AmbiguousPortError: If ``port`` is None and several candidates exist.
        PortBusyError: If another connection already holds the port.
        serial.SerialException: For any other failure to open the port, such
            as missing permissions.
    """

    def __init__(
        self,
        port: str | None = None,
        baudrate: int = DEFAULT_BAUDRATE,
        timeout: float = 1.0,
        inter_frame_delay: float = DEFAULT_INTER_FRAME_DELAY,
        exclusive: bool = True,
    ) -> None:
        """Open the serial port and configure the line rate."""
        self.port = port or autodetect_port()
        self.baudrate = baudrate
        self.timeout = timeout
        self.inter_frame_delay = inter_frame_delay
        self._lock = threading.RLock()
        self._last_write = 0.0
        self._rx = bytearray()

        try:
            self._serial = self._open(baudrate, exclusive)
        except ValueError:
            # pyserial refuses the rate up front on platforms without custom-rate
            # support. Open at a standard rate and set the real one by hand; on
            # CDC-ACM the rate is nominal anyway, so this still works.
            _log.debug("pyserial rejected %d baud, opening at 115200 and using termios2", baudrate)
            self._serial = self._open(115200, exclusive)
            _apply_termios2_speed(self._serial.fileno(), baudrate)

        self._verify_baudrate()
        self.reset_input()

    def _open(self, baudrate: int, exclusive: bool) -> serial.Serial:
        """Open the port 8N1 at ``baudrate``, translating a busy port.

        Raises:
            PortBusyError: If another connection holds the port.
        """
        try:
            return serial.Serial(
                self.port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                write_timeout=self.timeout,
                exclusive=exclusive,
            )
        except serial.SerialException as exc:
            if exc.errno in _BUSY_ERRNOS:
                raise PortBusyError(self.port) from exc
            raise

    @contextlib.contextmanager
    def _translate_link_errors(self) -> Iterator[None]:
        """Turn a failure of the underlying port into :class:`EilikConnectionError`.

        Using the transport after :meth:`close` is a caller error rather than a
        lost link, so pyserial's own error for it passes through unchanged.
        """
        try:
            yield
        except serial.PortNotOpenError:
            raise
        except _LINK_ERRORS as exc:
            raise EilikConnectionError(self.port, exc) from exc

    def _verify_baudrate(self) -> None:
        """Read the rate back and re-apply it directly if it did not stick."""
        actual = _read_termios2_speed(self._serial.fileno())
        if actual is None or actual == self.baudrate:
            return
        _log.debug("baud rate reads back as %s, re-applying %s", actual, self.baudrate)
        if not _apply_termios2_speed(self._serial.fileno(), self.baudrate):
            # Not fatal: CDC-ACM ignores the line rate, so a mismatch here does
            # not stop the robot from answering.
            _log.warning(
                "could not set %d baud on %s (reads back as %s); continuing, as the rate is "
                "nominal on CDC-ACM",
                self.baudrate,
                self.port,
                actual,
            )

    @property
    def actual_baudrate(self) -> int | None:
        """The rate the kernel reports for this port, or None if unavailable."""
        return _read_termios2_speed(self._serial.fileno())

    @property
    def is_open(self) -> bool:
        """Whether the underlying serial port is open."""
        return bool(self._serial.is_open)

    def close(self) -> None:
        """Close the serial port. Safe to call more than once."""
        with self._lock:
            if self._serial.is_open:
                self._serial.close()

    def __enter__(self) -> SerialTransport:
        """Return self, for use as a context manager."""
        return self

    def __exit__(self, *exc_info: object) -> None:
        """Close the port on exit."""
        self.close()

    def reset_input(self) -> None:
        """Discard any bytes already buffered, in the kernel and in this reader.

        Raises:
            EilikConnectionError: If the device has gone away.
        """
        with self._lock:
            with self._translate_link_errors():
                self._serial.reset_input_buffer()
            self._rx.clear()

    # -- transmit ----------------------------------------------------------

    def send_frame(self, frame: bytes) -> None:
        """Write a pre-encoded frame, re-validating it against the guard.

        The bytes are decoded in full and put through the same
        :func:`~eilik.protocol.ensure_frame_allowed` check as
        :func:`~eilik.protocol.encode_frame`, so hand-assembled bytes cannot
        smuggle a destructive command past the blacklist by bypassing the
        encoder: not as the opcode, not nested inside a 0x61 envelope, and not
        as a second frame appended after a harmless one.

        Args:
            frame: Exactly one complete frame, magic and checksum included.

        Raises:
            FrameError: If the bytes are not exactly one well-formed frame with
                a valid checksum.
            BlacklistedCommandError: If the opcode or a nested sub-command is
                destructive or refused.
            UnsupportedCommandError: If either is outside the allowlist.
            EilikConnectionError: If the write fails because the device has
                gone away.
        """
        try:
            decoded = decode_frame(bytes(frame))
        except (ChecksumError, FrameError) as exc:
            raise FrameError(
                f"refusing to transmit bytes that are not a well-formed frame: {exc}"
            ) from exc
        ensure_frame_allowed(decoded.command, decoded.data)

        with self._lock:
            pause = self.inter_frame_delay - (time.monotonic() - self._last_write)
            if pause > 0:
                time.sleep(pause)
            with self._translate_link_errors():
                self._serial.write(decoded.raw)
                self._serial.flush()
            self._last_write = time.monotonic()

    # -- receive -----------------------------------------------------------

    def _try_parse(self) -> Frame | None:
        """Pull one frame out of the receive buffer, resynchronising as needed.

        Bytes are only consumed once a frame has been fully validated. A magic
        prefix that turns out to be noise -- an implausible length field, or a
        checksum that does not match -- advances the scan by a single byte
        instead of discarding a whole frame's worth, so a real frame that
        happens to follow garbage ending in 0xAA is still recovered.

        Returns:
            The next valid frame, or None if more bytes are needed.
        """
        buf = self._rx
        while True:
            index = buf.find(MAGIC)
            if index < 0:
                # Keep only what could still be the start of a magic prefix.
                del buf[: max(0, len(buf) - (len(MAGIC) - 1))]
                return None
            if index:
                del buf[:index]

            if len(buf) < len(MAGIC) + 2:
                return None

            length = int.from_bytes(buf[3:5], "little")
            if length < MIN_LENGTH_FIELD or length + 3 > MAX_FRAME_SIZE:
                _log.debug("implausible length %d after magic, resyncing", length)
                del buf[:1]
                continue

            total = length + 3
            if len(buf) < total:
                return None

            try:
                frame = decode_frame(bytes(buf[:total]))
            except (ChecksumError, FrameError) as exc:
                # The firmware drops bad frames without answering, so damaged
                # data is noise to be skipped rather than an error to report.
                _log.debug("discarding damaged frame: %s", exc)
                del buf[:1]
                continue

            del buf[:total]
            return frame

    def read_frame(self, timeout: float | None = None) -> Frame:
        """Read one frame, resynchronising on the magic prefix.

        Damaged frames are skipped rather than raised: the firmware itself
        silently ignores frames whose checksum is wrong, so corruption shows up
        as an absent reply, not as an error. If nothing valid arrives before the
        deadline, a timeout is raised.

        Args:
            timeout: Seconds to wait. Defaults to the transport's timeout.

        Returns:
            The decoded frame.

        Raises:
            EilikTimeoutError: If no complete, valid frame arrived in time.
            EilikConnectionError: If the device has gone away.
        """
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)

        with self._lock:
            while True:
                frame = self._try_parse()
                if frame is not None:
                    return frame

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise EilikTimeoutError(
                        f"timed out waiting for a frame ({len(self._rx)} bytes buffered)"
                    )
                with self._translate_link_errors():
                    self._serial.timeout = remaining
                    chunk = self._serial.read(max(1, self._serial.in_waiting))
                if chunk:
                    self._rx.extend(chunk)

    # -- request/response --------------------------------------------------

    def request(
        self,
        command: int,
        data: bytes = b"",
        expect: int | None = None,
        timeout: float | None = None,
    ) -> Frame:
        """Send a command and wait for the matching reply.

        Frames that arrive but do not carry the expected opcode (an unsolicited
        heartbeat, a late reply to an earlier command) are discarded and the wait
        continues, as are frames that fail their checksum, since a corrupt frame
        only means the link garbled it.

        The lock is held for the whole exchange, so no other thread can slip a
        frame in between the request and its reply. That is what keeps a screen
        write from landing while a servo write is still being processed, which
        crashes the servo controller and drops the robot off the USB bus.

        Args:
            command: The opcode to send.
            data: The data field.
            expect: Opcode to wait for. Defaults to ``command``, since the
                firmware echoes the opcode it is acknowledging.
            timeout: Seconds to wait for the reply.

        Returns:
            The matching reply frame.

        Raises:
            EilikTimeoutError: If no matching reply arrived. The firmware
                silently drops frames whose checksum is wrong, so a timeout is
                also how a corrupted transmission shows up.
            EilikConnectionError: If the device has gone away.
        """
        expect = command if expect is None else expect
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)

        with self._lock:
            self.reset_input()
            self.send_frame(encode_frame(command, data))

            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    frame = self.read_frame(timeout=remaining)
                except EilikTimeoutError:
                    break
                if frame.command == expect:
                    return frame
                _log.debug("ignoring unsolicited frame %r while awaiting 0x%02X", frame, expect)

        raise EilikTimeoutError(
            f"no reply to command 0x{command:02X} within "
            f"{self.timeout if timeout is None else timeout:.3f}s; the firmware drops frames "
            "with a bad checksum without answering, so this is also how corruption surfaces"
        )

    def __repr__(self) -> str:
        """Return a short description of the link."""
        state = "open" if self.is_open else "closed"
        return f"SerialTransport({self.port!r}, {self.baudrate} baud, {state})"

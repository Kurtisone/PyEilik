"""A fake Eilik that speaks the real protocol over a pty pair.

This lets the transport and the high-level API be exercised end to end -- real
serial file descriptors, real framing, real resynchronisation -- without any
hardware attached.
"""

from __future__ import annotations

import contextlib
import os
import pty
import secrets
import select
import threading
import tty

from eilik.protocol import MAGIC, Command, checksum
from eilik.screen import FRAMEBUFFER_SIZE
from eilik.servo import Motor

#: Payload the firmware returns after the leading status byte of a ping reply,
#: modelled on the documented one: "4424" and "H090" at offsets 1 and 7, a u32
#: at offset 11. The reference elides the tail, so this one is zero-filled.
PING_PAYLOAD = (
    bytes.fromhex("da") + b"4424" + bytes.fromhex("0e00") + b"H090" + bytes.fromhex("5b9201000d00")
).ljust(33, b"\x00")


def device_nonce() -> bytes:
    """Return a fresh nonce shaped like the firmware's: first byte == last byte."""
    head = secrets.token_bytes(4)
    return head + head[:1]


def raw_frame(command: int, data: bytes = b"") -> bytes:
    """Build a frame without going through the SDK's safety guard.

    Tests use this to hand-assemble frames the encoder would refuse, so the
    transport's own guard can be exercised independently.
    """
    body = (len(data) + 4).to_bytes(2, "little") + bytes([command]) + data
    return MAGIC + body + bytes([checksum(body)])


class FakeEilik:
    """A scriptable robot on the far end of a pty.

    Attributes:
        port: Device path the transport should open.
        servos: Current servo positions, updated by 0xA2 writes.
        framebuffer: Current screen contents as the robot stores them, i.e.
            rotated relative to what the SDK caller supplies.
        received: Every frame the fake decoded, as ``(command, data)``.
    """

    def __init__(self) -> None:
        """Create the pty pair and start the responder thread."""
        self._master, self._slave = pty.openpty()
        tty.setraw(self._slave)
        self.port = os.ttyname(self._slave)

        self.servos: dict[Motor, int] = dict.fromkeys(Motor, 1500)
        self.framebuffer = bytearray(FRAMEBUFFER_SIZE)
        self.received: list[tuple] = []

        #: Frames to emit before the next real reply, to simulate unsolicited
        #: traffic such as a heartbeat arriving mid-exchange.
        self.inject_before_reply: list[bytes] = []
        #: When set, corrupt the next reply's checksum.
        self.corrupt_next_reply = False
        #: When set, swallow the next request without replying.
        self.drop_next_request = False
        #: When set, behave like a wedged servo controller: writes are still
        #: acknowledged, but every position reads back as zero.
        self.servo_fault = False

        self._stop = threading.Event()
        # Written to on shutdown, to wake the responder out of select().
        self._wake_read, self._wake_write = os.pipe()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    # -- lifecycle ---------------------------------------------------------

    def _stop_responder(self) -> None:
        """Stop the responder thread and wait for it to let go of the master."""
        self._stop.set()
        with contextlib.suppress(OSError):
            os.write(self._wake_write, b"\x00")
        self._thread.join(timeout=2)

    def close(self) -> None:
        """Stop the responder and release the pty."""
        self._stop_responder()
        for fd in (self._master, self._slave, self._wake_read, self._wake_write):
            with contextlib.suppress(OSError):
                os.close(fd)

    def unplug(self) -> None:
        """Simulate the robot dropping off the bus: the far end of the pty hangs up.

        The responder is stopped before the master is closed, because a thread
        still blocked on the descriptor would keep the pty open.
        """
        self._stop_responder()
        with contextlib.suppress(OSError):
            os.close(self._master)

    def __enter__(self) -> FakeEilik:
        """Return self, for use as a context manager."""
        return self

    def __exit__(self, *exc_info: object) -> None:
        """Shut the fake down on exit."""
        self.close()

    # -- wire handling -----------------------------------------------------

    def _read_exact(self, count: int) -> bytes | None:
        """Read exactly ``count`` bytes from the master, or None if shut down."""
        chunks = bytearray()
        while len(chunks) < count:
            if self._stop.is_set():
                return None
            try:
                # Wait on the wake pipe too, so a stop request is noticed
                # without waiting for the next byte from the transport.
                ready, _, _ = select.select([self._master, self._wake_read], [], [])
                if self._master not in ready:
                    continue
                chunk = os.read(self._master, count - len(chunks))
            except (OSError, ValueError):
                return None
            if not chunk:
                return None
            chunks.extend(chunk)
        return bytes(chunks)

    def _serve(self) -> None:
        """Read frames from the master fd and answer them until shut down."""
        window = bytearray()
        while not self._stop.is_set():
            byte = self._read_exact(1)
            if byte is None:
                return
            window.extend(byte)
            if len(window) > len(MAGIC):
                del window[0]
            if bytes(window) != MAGIC:
                continue
            # Reset the sync window, otherwise the next frame's leading 0xAA
            # completes this stale window and the length is read from the wrong
            # offset.
            window.clear()

            header = self._read_exact(2)
            if header is None:
                return
            length = int.from_bytes(header, "little")
            body = self._read_exact(length - 2)
            if body is None:
                return

            command, data, received_checksum = body[0], body[1:-1], body[-1]
            if checksum(header + body[:-1]) != received_checksum:
                # The real firmware silently ignores frames with a bad checksum.
                continue

            self.received.append((command, data))
            reply = self._reply_for(command, data)
            if reply is None:
                continue
            for extra in self.inject_before_reply:
                self._write_all(extra)
            self.inject_before_reply = []
            if self.corrupt_next_reply:
                self.corrupt_next_reply = False
                reply = reply[:-1] + bytes([reply[-1] ^ 0xFF])
            self._write_all(reply)

    def _write_all(self, payload: bytes) -> None:
        """Write every byte to the master fd, tolerating partial writes."""
        view = memoryview(payload)
        while view:
            try:
                written = os.write(self._master, view)
            except OSError:
                return
            view = view[written:]

    def _reply_for(self, command: int, data: bytes) -> bytes | None:
        """Return the frame the firmware would answer with, or None for silence."""
        if self.drop_next_request:
            self.drop_next_request = False
            return None

        if command == Command.PING:
            return raw_frame(Command.PING, b"\x94" + PING_PAYLOAD)

        if command == Command.ENVELOPE:
            # Echo the sub-command behind a nonce of the firmware's own.
            return raw_frame(Command.ENVELOPE, device_nonce() + data[-1:])

        if command == Command.READ_SERVOS and self.servo_fault:
            payload = bytearray([len(self.servos)])
            for motor in sorted(self.servos):
                payload.extend((int(motor), 0, 0))
            return raw_frame(Command.READ_SERVOS, bytes(payload))

        if command == Command.READ_SERVOS:
            payload = bytearray([len(self.servos)])
            for motor in sorted(self.servos):
                payload.append(int(motor))
                payload.extend(self.servos[motor].to_bytes(2, "little"))
            return raw_frame(Command.READ_SERVOS, bytes(payload))

        if command == Command.WRITE_SERVOS:
            count = data[0]
            for index in range(count):
                offset = 1 + index * 3
                motor = Motor(data[offset])
                self.servos[motor] = int.from_bytes(data[offset + 1 : offset + 3], "little")
            return raw_frame(Command.WRITE_SERVOS, b"\x01")

        if command == Command.READ_SCREEN:
            return raw_frame(Command.READ_SCREEN, b"\x04" + bytes(self.framebuffer))

        if command == Command.WRITE_SCREEN:
            self.framebuffer = bytearray(data)
            return raw_frame(Command.WRITE_SCREEN, b"\x01")

        return None

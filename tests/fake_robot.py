"""A fake Eilik that speaks the real protocol over a pty pair.

This lets the transport and the high-level API be exercised end to end -- real
serial file descriptors, real framing, real resynchronisation -- without any
hardware attached.
"""

from __future__ import annotations

import contextlib
import os
import pty
import threading
import tty

from eilik.protocol import MAGIC, Command, checksum
from eilik.screen import FRAMEBUFFER_SIZE
from eilik.servo import Motor

#: Payload the firmware returns after the leading status byte of a ping reply.
PING_PAYLOAD = bytes(range(0x10, 0x10 + 33))


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

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    # -- lifecycle ---------------------------------------------------------

    def close(self) -> None:
        """Stop the responder and release the pty."""
        self._stop.set()
        for fd in (self._master, self._slave):
            with contextlib.suppress(OSError):
                os.close(fd)
        self._thread.join(timeout=2)

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
                chunk = os.read(self._master, count - len(chunks))
            except OSError:
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
            return raw_frame(Command.ENVELOPE, b"\x01")

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

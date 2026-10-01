"""A simulated Eilik that speaks the real protocol over a pseudo-terminal.

The simulator opens a pty pair and answers on the master side the way the
firmware does, so anything that drives a real robot -- this SDK, the ``eilik``
command, ``probe.py``, another tool entirely -- can be pointed at
:attr:`SimulatedEilik.port` instead and run unchanged::

    with SimulatedEilik() as sim, Eilik(port=sim.port) as robot:
        robot.move({Motor.HEAD: 1650}, duration=0.5)
        print(sim.positions())

``eilik simulate`` runs one in a terminal and draws its screen and servos live.

It models what ``PROTOCOL.md`` documents about the real device, including the
parts that hurt:

* servos travel towards their target at the measured speeds instead of
  teleporting, so a read during a movement sees it in progress;
* a servo frame and a screen frame sent back to back without waiting for the
  acknowledgement crash it: it drops off the bus and its servo controller stays
  wedged (every position reads zero, nothing moves) until :meth:`power_cycle`;
* a frame with a bad checksum is ignored without a reply.

Unlike the real device, it refuses destructive commands: they are recorded in
:attr:`SimulatedEilik.violations` and otherwise ignored.
"""

from __future__ import annotations

import contextlib
import math
import os
import pty
import secrets
import select
import threading
import time
import tty
from collections.abc import Mapping
from types import MappingProxyType

from .canvas import Canvas
from .protocol import (
    BLACKLISTED_COMMANDS,
    MAGIC,
    MAX_FRAME_SIZE,
    MIN_LENGTH_FIELD,
    SCREEN_HOLD,
    SCREEN_RELEASE,
    SUBCOMMAND_HEARTBEAT,
    Command,
    checksum,
)
from .screen import FRAMEBUFFER_SIZE, WIDTH, rotate180, to_blocks
from .servo import NEUTRAL_POSITION, VERIFIED_RANGES, Motor

__all__ = [
    "PING_PAYLOAD",
    "SLEW_RATES",
    "SimulatedEilik",
    "describe_frame",
    "device_nonce",
    "idle_face",
    "raw_frame",
    "render",
]

#: Payload of a ping reply after its status byte, modelled on the documented
#: one: "4424" and "H090" at offsets 1 and 7, a u32 at offset 11. The reference
#: elides the tail, so this one is zero-filled.
PING_PAYLOAD = (
    bytes.fromhex("da") + b"4424" + bytes.fromhex("0e00") + b"H090" + bytes.fromhex("5b9201000d00")
).ljust(33, b"\x00")

#: How fast each joint moves, in units per second. These are the lower bounds
#: measured on a real robot; the true ceilings are higher and unknown.
SLEW_RATES: Mapping[Motor, float] = MappingProxyType(
    {Motor.ARM_RIGHT: 3000.0, Motor.ARM_LEFT: 3000.0, Motor.BODY: 1450.0, Motor.HEAD: 1450.0}
)

# Opcodes the firmware answers that this SDK never sends.
_READ_RUNNING_NUMBER = 0xA5
_WRITE_RUNNING_NUMBER = 0xA6
_LEGACY_SERVO = 0x03  # sub-command inside 0x61

_MOTOR_IDS = frozenset(int(motor) for motor in Motor)

#: How soon the robot's idle animation repaints over a frame written without
#: the screen hold, on firmware that needs it (strognoff/eilik-sdk: ~50 ms).
IDLE_TAKEOVER = 0.05

#: The pair of commands that crash the robot when interleaved.
_CRASHING_PAIR = frozenset({int(Command.WRITE_SERVOS), int(Command.WRITE_SCREEN)})


def device_nonce() -> bytes:
    """Return a fresh nonce shaped like the firmware's: first byte == last byte."""
    head = secrets.token_bytes(4)
    return head + head[:1]


def idle_face(seconds: float) -> bytes:
    """Return the simulator's stand-in for the robot's own face, ``seconds`` in.

    Two eyes that blink every four seconds and glance left and right. It is
    drawn here, not copied from the robot: it only shows where the robot's
    own animation would be on screen.
    """
    canvas = Canvas()
    glance = (0, -6, 0, 6)[int(seconds // 3) % 4]
    blinking = seconds % 4.0 < 0.15
    for centre in (42 + glance, 86 + glance):
        if blinking:
            canvas.rect(centre - 10, 31, 21, 3, fill=True)
            continue
        canvas.circle(centre, 27, 10, fill=True)
        canvas.circle(centre, 37, 10, fill=True)
        canvas.rect(centre - 10, 27, 21, 11, fill=True)
    return bytes(canvas)


def raw_frame(command: int, data: bytes = b"") -> bytes:
    """Build a frame without going through the SDK's safety guard.

    The simulator replies with it, and tests use it to hand-assemble frames the
    encoder would refuse.
    """
    body = (len(data) + 4).to_bytes(2, "little") + bytes([command]) + data
    return MAGIC + body + bytes([checksum(body)])


class SimulatedEilik:
    """A virtual robot on the far end of a pty.

    Args:
        slew: Move servos at :data:`SLEW_RATES` rather than instantly.
        strict: Crash, like the real robot, when a servo frame and a screen
            frame are sent back to back without waiting for the
            acknowledgement in between.
        servo_fault: Start with a wedged servo controller.
        idle_face: Show the robot's own animated face whenever the host does
            not own the screen, as the robot does. Off, the screen simply
            keeps whatever was last written.
        hold_required: Model firmware on which a frame written without the
            screen hold (running number 100) is painted over by the idle
            animation within :data:`IDLE_TAKEOVER`. Off models the robot
            ``PROTOCOL.md`` describes, where writing a frame takes the screen.

    Attributes:
        port: Device path to open, e.g. ``Eilik(port=sim.port)``.
        servos: Where each servo is heading, or resting. See :meth:`positions`
            for where they are right now.
        framebuffer: The last frame the host wrote, as the robot stores it
            (rotated). :meth:`screen` returns what is actually on screen, the
            right way up.
        screen_held: Whether the host holds the screen (running number 100).
        received: Every intact frame received, as ``(command, data)``.
        violations: Human-readable reports of anything that would have hurt a
            real robot: destructive commands, crashing sequences.
        crashed: Whether the robot has crashed and dropped off the bus.
        servo_fault: Whether the servo controller is wedged.
        changes: Incremented whenever the screen or a servo target changes, so
            a display can tell when to redraw.
        inject_before_reply: Frames to emit before the next reply, to simulate
            unsolicited traffic.
        corrupt_next_reply: Corrupt the next reply's checksum.
        drop_next_request: Swallow the next request without replying.
    """

    def __init__(
        self,
        *,
        slew: bool = True,
        strict: bool = True,
        servo_fault: bool = False,
        idle_face: bool = True,
        hold_required: bool = True,
    ) -> None:
        """Create the pty pair and start answering on it."""
        self._master, self._slave = pty.openpty()
        tty.setraw(self._slave)
        self.port = os.ttyname(self._slave)

        self.slew = slew
        self.strict = strict
        self.idle_face = idle_face
        self.hold_required = hold_required
        self.screen_held = False
        # When the host's current frame was written; None once it is gone.
        self._shown_at: float | None = None
        self._born = time.monotonic()
        self.servos: dict[Motor, int] = dict.fromkeys(Motor, NEUTRAL_POSITION)
        self.framebuffer = bytearray(FRAMEBUFFER_SIZE)
        self.received: list[tuple[int, bytes]] = []
        self.violations: list[str] = []
        self.crashed = False
        self.servo_fault = servo_fault
        self.changes = 0

        self.inject_before_reply: list[bytes] = []
        self.corrupt_next_reply = False
        self.drop_next_request = False

        # motor -> (position when the current movement started, start time)
        self._motion: dict[Motor, tuple[float, float]] = {}
        self._state_lock = threading.Lock()
        self._stop = threading.Event()
        # Written to on shutdown, to wake the responder out of select().
        self._wake_read, self._wake_write = os.pipe()
        self._thread = threading.Thread(target=self._serve, name="eilik-simulator", daemon=True)
        self._thread.start()

    # -- lifecycle -------------------------------------------------------------

    def _stop_responder(self) -> None:
        """Stop the responder thread and wait for it to let go of the master."""
        self._stop.set()
        with contextlib.suppress(OSError):
            os.write(self._wake_write, b"\x00")
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=2)

    def _close_fd(self, name: str) -> None:
        """Close the descriptor held in attribute ``name``, at most once.

        The number of a closed descriptor can be handed straight to the next
        file opened, so closing it a second time could close someone else's.
        """
        fd = getattr(self, name)
        setattr(self, name, -1)
        if fd >= 0:
            with contextlib.suppress(OSError):
                os.close(fd)

    def close(self) -> None:
        """Stop answering and release the pty. Safe to call more than once."""
        self._stop_responder()
        for name in ("_master", "_slave", "_wake_read", "_wake_write"):
            self._close_fd(name)

    def unplug(self) -> None:
        """Drop off the bus: the far end of the pty hangs up."""
        self._stop_responder()
        self._close_fd("_master")

    def power_cycle(self) -> None:
        """Clear a wedged servo controller, as the switch on the body does.

        A crashed simulator stays off the bus; start a new one to reconnect.
        """
        self.servo_fault = False

    def __enter__(self) -> SimulatedEilik:
        """Return self, for use as a context manager."""
        return self

    def __exit__(self, *exc_info: object) -> None:
        """Shut down on exit."""
        self.close()

    def __repr__(self) -> str:
        """Return a short description."""
        state = "crashed" if self.crashed else "wedged" if self.servo_fault else "running"
        return f"SimulatedEilik({self.port!r}, {state}, {len(self.received)} frames)"

    # -- observable state ----------------------------------------------------

    def screen(self) -> bytes:
        """Return what is on screen right now, the right way up."""
        now = time.monotonic()
        if not self.idle_face or self._host_owns_screen(now):
            return rotate180(bytes(self.framebuffer))
        return idle_face(now - self._born)

    def screen_owner(self) -> str:
        """Return ``"host"`` or ``"robot"``: whose picture is on screen."""
        return "host" if self._host_owns_screen(time.monotonic()) else "robot"

    def _host_owns_screen(self, now: float) -> bool:
        """Whether the host's frame is still showing, as opposed to the robot's face."""
        if self._shown_at is None:
            return False
        return self.screen_held or now - self._shown_at < IDLE_TAKEOVER

    def positions(self) -> dict[Motor, int]:
        """Return where each servo physically is right now."""
        now = time.monotonic()
        with self._state_lock:
            return {motor: self._position(motor, now) for motor in self.servos}

    def _position(self, motor: Motor, now: float) -> int:
        """Return a servo's position at ``now``; the state lock must be held."""
        target = self.servos[motor]
        if not self.slew or motor not in self._motion:
            return target
        start, started = self._motion[motor]
        travelled = SLEW_RATES[motor] * (now - started)
        if travelled >= abs(target - start):
            return target
        return round(start + math.copysign(travelled, target - start))

    def _set_target(self, motor: Motor, target: int) -> None:
        """Start moving ``motor`` towards ``target``."""
        now = time.monotonic()
        with self._state_lock:
            self._motion[motor] = (self._position(motor, now), now)
            self.servos[motor] = target
        self.changes += 1

    # -- wire handling -------------------------------------------------------

    def _read_some(self, timeout: float | None = None) -> bytes | None:
        """Return the bytes available within ``timeout``, b"" if none, None to stop."""
        try:
            ready, _, _ = select.select([self._master, self._wake_read], [], [], timeout)
            if self._stop.is_set():
                return None
            if self._master not in ready:
                return b""
            data = os.read(self._master, 4096)
        except (OSError, ValueError):
            return None
        return data or None

    def _serve(self) -> None:
        """Read frames from the master and answer them until stopped."""
        buffer = bytearray()
        while not self._stop.is_set():
            data = self._read_some()
            if data is None:
                return
            buffer.extend(data)
            while (frame := self._take_frame(buffer)) is not None:
                if self.strict and self._crashes_after(frame[0], buffer):
                    return
                self._handle(*frame)

    @staticmethod
    def _take_frame(buffer: bytearray) -> tuple[int, bytes] | None:
        """Remove and return the next intact frame, skipping noise and damage."""
        while True:
            index = buffer.find(MAGIC)
            if index < 0:
                del buffer[: max(0, len(buffer) - (len(MAGIC) - 1))]
                return None
            del buffer[:index]
            if len(buffer) < 5:
                return None
            length = int.from_bytes(buffer[3:5], "little")
            if length < MIN_LENGTH_FIELD or length + 3 > MAX_FRAME_SIZE:
                del buffer[:1]
                continue
            if len(buffer) < length + 3:
                return None
            frame = bytes(buffer[: length + 3])
            del buffer[: length + 3]
            if checksum(frame[3:-1]) == frame[-1]:
                return frame[5], frame[6:-1]
            # The firmware ignores a frame with a bad checksum, without replying.

    def _crashes_after(self, command: int, buffer: bytearray) -> bool:
        """Crash if the next frame already arrived and the pair is the fatal one.

        The host can only have sent the next frame before this one's
        acknowledgement if it did not wait for it.
        """
        if command not in _CRASHING_PAIR:
            return False
        deadline = time.monotonic() + 0.005
        while len(buffer) < 6 and time.monotonic() < deadline:
            more = self._read_some(timeout=0.001)
            if more is None:
                return False
            buffer.extend(more)
        start = buffer.find(MAGIC)
        if start < 0 or len(buffer) < start + 6:
            return False
        following = buffer[start + 5]
        if {command, following} != _CRASHING_PAIR:
            return False
        self.violations.append(
            f"0x{following:02X} arrived while 0x{command:02X} was still being processed, "
            "without waiting for the acknowledgement: the real robot crashes, drops off "
            "the USB bus (ENXIO) and can leave its servo controller wedged"
        )
        self.crashed = True
        self.servo_fault = True
        self.changes += 1
        self._stop.set()
        self._close_fd("_master")
        return True

    def _handle(self, command: int, data: bytes) -> None:
        """Record a frame, act on it, and send the reply if there is one."""
        self.received.append((command, data))
        reply = self._reply_for(command, data)
        if reply is None:
            return
        for extra in self.inject_before_reply:
            self._write_all(extra)
        self.inject_before_reply = []
        if self.corrupt_next_reply:
            self.corrupt_next_reply = False
            reply = reply[:-1] + bytes([reply[-1] ^ 0xFF])
        self._write_all(reply)

    def _write_all(self, payload: bytes) -> None:
        """Write every byte to the master, tolerating partial writes."""
        view = memoryview(payload)
        while view:
            try:
                written = os.write(self._master, view)
            except OSError:
                return
            view = view[written:]

    # -- the firmware --------------------------------------------------------

    def _reply_for(self, command: int, data: bytes) -> bytes | None:
        """Act on a command and return the firmware's reply, or None for silence."""
        if self.drop_next_request:
            self.drop_next_request = False
            return None

        if command in BLACKLISTED_COMMANDS:
            self.violations.append(
                f"received destructive command 0x{command:02X} "
                f"({BLACKLISTED_COMMANDS[command]}); a real robot could have been bricked. "
                "The simulator ignored it"
            )
            return None

        if command == Command.PING:
            return raw_frame(Command.PING, b"\x94" + PING_PAYLOAD)
        if command == Command.ENVELOPE:
            return self._envelope(data)
        if command == Command.READ_SERVOS:
            return raw_frame(Command.READ_SERVOS, self._servo_report())
        if command == Command.WRITE_SERVOS:
            self._write_servos(data)
            return raw_frame(Command.WRITE_SERVOS, b"\x01")
        if command == Command.READ_SCREEN:
            # It reads back what is displayed, the robot's own face included.
            return raw_frame(Command.READ_SCREEN, b"\x04" + rotate180(self.screen()))
        if command == Command.WRITE_SCREEN:
            if len(data) == FRAMEBUFFER_SIZE:
                self.framebuffer = bytearray(data)
                self._shown_at = time.monotonic()
                if not self.hold_required:
                    self.screen_held = True  # writing takes the screen
                self.changes += 1
            return raw_frame(Command.WRITE_SCREEN, b"\x01")
        if command == _READ_RUNNING_NUMBER:
            # No handler on the documented firmware: it answers tagged 0xA4.
            return raw_frame(Command.WRITE_SCREEN, bytes.fromhex("0400ff00ff"))
        if command == _WRITE_RUNNING_NUMBER:
            self._running_number(data[0] if len(data) == 1 else None)
            return raw_frame(_WRITE_RUNNING_NUMBER, b"\x01")  # acknowledged, whatever the value
        return None  # no handler, no reply

    def _running_number(self, value: int | None) -> None:
        """Apply a 0xA6 running number: 100 holds the screen, 0 releases it."""
        if value == SCREEN_HOLD:
            if not self._host_owns_screen(time.monotonic()):
                self._shown_at = None  # a frame already painted over stays gone
            self.screen_held = True
        elif value == SCREEN_RELEASE:
            self.screen_held = False
            self._shown_at = None
            self.changes += 1
        # Other values: acknowledged without any modelled effect.

    def _envelope(self, data: bytes) -> bytes | None:
        """Answer a 0x61 frame: a five-byte nonce, then a sub-command."""
        if len(data) < 6:
            return None
        subcommand, body = data[5], data[6:]
        if subcommand == SUBCOMMAND_HEARTBEAT:
            return raw_frame(Command.ENVELOPE, device_nonce() + bytes([SUBCOMMAND_HEARTBEAT]))
        if subcommand == _LEGACY_SERVO:
            # 03 01 <motor> 01 <lo> <hi> [tail]: one motor per frame, unclamped.
            if len(body) >= 5 and body[1] in _MOTOR_IDS and not self.servo_fault:
                self._set_target(Motor(body[1]), int.from_bytes(body[3:5], "little"))
            return raw_frame(Command.ENVELOPE, device_nonce() + bytes([_LEGACY_SERVO]))
        return None

    def _servo_report(self) -> bytes:
        """Return a 0xA1 payload; all zeros while the controller is wedged."""
        positions = self.positions()
        payload = bytearray([len(positions)])
        for motor in sorted(positions):
            position = 0 if self.servo_fault else positions[motor]
            payload.append(int(motor))
            payload.extend(position.to_bytes(2, "little"))
        return bytes(payload)

    def _write_servos(self, data: bytes) -> None:
        """Apply a 0xA2 payload. A wedged controller acknowledges but ignores it."""
        if not data or self.servo_fault:
            return
        for index in range(data[0]):
            entry = data[1 + 3 * index : 4 + 3 * index]
            if len(entry) == 3 and entry[0] in _MOTOR_IDS:
                self._set_target(Motor(entry[0]), int.from_bytes(entry[1:], "little"))


# -- display -------------------------------------------------------------------

#: Positions shown by the servo gauges, wide enough for every verified range.
_GAUGE_RANGE = (1000, 2000)
_GAUGE_WIDTH = 41


def describe_frame(command: int, data: bytes) -> str:
    """Return a short human-readable description of a received frame."""
    if command == Command.ENVELOPE and len(data) == 6:
        subcommand = {SUBCOMMAND_HEARTBEAT: "heartbeat", _LEGACY_SERVO: "legacy servo"}
        return f"0x61 {subcommand.get(data[5], f'envelope, sub-command 0x{data[5]:02X}')}"
    if command == Command.WRITE_RUNNING_NUMBER and len(data) == 1:
        meaning = {SCREEN_HOLD: "screen hold", SCREEN_RELEASE: "screen release"}
        return f"0xA6 {meaning.get(data[0], f'running number {data[0]}')}"
    try:
        name = Command(command).name.lower().replace("_", " ")
    except ValueError:
        name = BLACKLISTED_COMMANDS.get(command, "unknown").split(" - ")[0]
    size = f", {len(data)} bytes" if data else ""
    return f"0x{command:02X} {name}{size}"


def _gauge(position: int, target: int) -> str:
    """Draw a position on a horizontal scale, with the target if it differs."""
    low, high = _GAUGE_RANGE
    span = _GAUGE_WIDTH - 1

    def slot(value: int) -> int:
        return round((min(max(value, low), high) - low) * span / (high - low))

    cells = ["\u2500"] * _GAUGE_WIDTH
    cells[slot(NEUTRAL_POSITION)] = "\u253c"
    if target != position:
        cells[slot(target)] = "\u25cb"
    cells[slot(position)] = "\u25cf"
    return "\u251c" + "".join(cells) + "\u2524"


def render(sim: SimulatedEilik, title: str = "Eilik simulator") -> str:
    """Draw the simulator's screen, servos and status as text.

    Args:
        sim: The simulator to draw.
        title: Shown in the top border, e.g. the port to connect to.

    Returns:
        A block of text 130 columns wide, ready to print.
    """
    border = "\u2500" * WIDTH
    heading = f"\u2500 {title} "
    lines = ["\u256d" + heading + border[len(heading) :] + "\u256e"]
    lines += ["\u2502" + row + "\u2502" for row in to_blocks(sim.screen()).split("\n")]
    lines.append("\u2570" + border + "\u256f")

    positions = sim.positions()
    for motor in Motor:
        position, target = positions[motor], sim.servos[motor]
        shown = 0 if sim.servo_fault else position
        low, high = VERIFIED_RANGES[motor]
        moving = f" \u2192 {target}" if target != position and not sim.servo_fault else ""
        lines.append(
            f" {motor.name:<9} {shown:>4}  {_gauge(position, target)}  "
            f"verified {low}-{high}{moving}"
        )

    if sim.crashed:
        state = "CRASHED: off the bus"
    elif sim.servo_fault:
        state = "servo controller WEDGED: positions read 0 (power-cycle to clear)"
    else:
        state = "running"
    last = describe_frame(*sim.received[-1]) if sim.received else "none yet"
    owner = sim.screen_owner()
    screen = "the robot's own face" if owner == "robot" else "the host's"
    if owner == "host":
        screen += " (held)" if sim.screen_held else " (until the robot repaints it)"
    lines.append(f" {state} \u00b7 screen: {screen}")
    lines.append(f" {len(sim.received)} frames \u00b7 last {last}")
    lines += [f" ! {violation}" for violation in sim.violations[-3:]]
    return "\n".join(lines)

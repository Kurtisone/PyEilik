"""Frame encoding and decoding for the Eilik serial protocol.

Every frame, in both directions, uses the same layout::

    offset      size        field
    0           3           AA AA AA            magic
    3           2           length (u16 LE)     total frame size minus 3
    5           1           command id
    6           length - 4  data
    length + 2  1           checksum

The checksum covers the bytes from the ``length`` field up to and including the
last data byte; the checksum byte itself is excluded. A frame whose checksum is
wrong is silently discarded by the firmware, which makes the absence of a reply
the integrity signal on the transmit path.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from enum import IntEnum
from types import MappingProxyType

from .errors import BlacklistedCommandError, ChecksumError, FrameError, UnsupportedCommandError

__all__ = [
    "BLACKLISTED_COMMANDS",
    "HEADER_SIZE",
    "HEARTBEAT_ENVELOPE_PREFIX",
    "MAGIC",
    "MAX_FRAME_SIZE",
    "Command",
    "Frame",
    "checksum",
    "decode_frame",
    "encode_frame",
    "encode_heartbeat",
    "ensure_command_allowed",
]

#: Start-of-frame marker.
MAGIC = b"\xaa\xaa\xaa"

#: Bytes before the data field: magic (3) + length (2) + command (1).
HEADER_SIZE = 6

#: Smallest legal value of the ``length`` field: command byte + checksum + the
#: two bytes of the length field itself, i.e. a frame carrying no data.
MIN_LENGTH_FIELD = 4

#: Largest frame we are willing to buffer. The biggest legitimate frame is the
#: 1032-byte screen read; anything larger means the length field was corrupted,
#: and without this cap a garbled length would make the reader block for a very
#: long time.
MAX_FRAME_SIZE = 4096


class Command(IntEnum):
    """Opcodes this SDK is allowed to emit.

    This enum doubles as the transmit allowlist: :func:`ensure_command_allowed`
    rejects anything that is not a member. Adding a member is the only way to
    let a new opcode reach the wire, which forces a deliberate review of whether
    the opcode is destructive.
    """

    PING = 0x01
    """Firmware identification. Read-only."""

    ENVELOPE = 0x61
    """Wrapper carrying a sub-command; only the heartbeat sub-command is used."""

    READ_SERVOS = 0xA1
    """Read the angles of the four servos. Read-only."""

    WRITE_SERVOS = 0xA2
    """Write servo angles, up to four motors per frame."""

    READ_SCREEN = 0xA3
    """Read the 1024-byte framebuffer. Read-only."""

    WRITE_SCREEN = 0xA4
    """Write the 1024-byte framebuffer."""


# ---------------------------------------------------------------------------
# Safety: destructive opcodes that must never be emitted
# ---------------------------------------------------------------------------
#
# The firmware happily accepts the opcodes below. They rewrite flash, overwrite
# on-device content, or reformat the SD card. A malformed or mistimed frame on
# any of them can leave the robot unbootable with no recovery path over the
# serial link, so this SDK refuses to build or transmit them at all.
#
# DO NOT DELETE ENTRIES FROM THIS MAPPING, and do not add these opcodes to
# `Command`. Both the encoder (`encode_frame`) and the transport's raw write
# path funnel through `ensure_command_allowed`, so there is no supported way to
# reach these opcodes even by hand-assembling bytes and calling a low-level
# function. If a future refactor introduces an opcode dispatch table, build it
# from `Command`, never by relaxing the guard below.
BLACKLISTED_COMMANDS: Mapping[int, str] = MappingProxyType(
    {
        0x02: "confirm_upgrade - commits a staged firmware upgrade",
        0x03: "content_update - overwrites on-device content",
        0x04: "firmware_flash - writes firmware to flash",
        0x05: "firmware_flash_direct - writes firmware to flash without staging",
        0x31: "write_specified - writes arbitrary data to the SD card",
        0x41: "reinit_sd - reinitialises the SD card",
        0x42: "format_sd - formats the SD card",
    }
)

#: Fixed prefix of the 0x61 envelope payload, observed on every heartbeat frame.
HEARTBEAT_ENVELOPE_PREFIX = b"\xe4\xc6\xf1\xca\x83"

#: Sub-command byte carried inside the 0x61 envelope for a heartbeat.
SUBCOMMAND_HEARTBEAT = 0xFF

#: Envelope sub-commands this SDK will emit. The envelope opcode nests a second
#: opcode inside its payload, so the allowlist has to apply one level deeper as
#: well, otherwise a destructive sub-command could ride inside a "safe" 0x61.
SAFE_ENVELOPE_SUBCOMMANDS = frozenset({SUBCOMMAND_HEARTBEAT})


def checksum(data: bytes) -> int:
    """Return the Eilik checksum byte for ``data``.

    Args:
        data: The bytes covered by the checksum, i.e. the ``length`` field, the
            command byte and the data field, concatenated.

    Returns:
        The checksum byte, in ``0..255``.
    """
    return 255 - (sum(data) % 256)


def ensure_command_allowed(command: int) -> int:
    """Validate that ``command`` may be transmitted, or raise.

    This is the single choke point for the destructive-command blacklist. Both
    :func:`encode_frame` and the transport's raw write path call it, so an
    opcode cannot reach the serial port without passing through here.

    Args:
        command: The opcode to check.

    Returns:
        ``command`` unchanged, so the call can be inlined in an expression.

    Raises:
        BlacklistedCommandError: If the opcode is known to be destructive.
        UnsupportedCommandError: If the opcode is not a :class:`Command` member.
    """
    if command in BLACKLISTED_COMMANDS:
        raise BlacklistedCommandError(command, BLACKLISTED_COMMANDS[command])
    if command not in _ALLOWED_COMMANDS:
        raise UnsupportedCommandError(command)
    return command


_ALLOWED_COMMANDS = frozenset(int(member) for member in Command)


def _ensure_envelope_payload_allowed(data: bytes) -> None:
    """Validate the sub-command nested inside a 0x61 envelope payload.

    Raises:
        UnsupportedCommandError: If the payload is malformed or carries a
            sub-command outside :data:`SAFE_ENVELOPE_SUBCOMMANDS`.
        BlacklistedCommandError: If the nested sub-command is blacklisted.
    """
    expected_len = len(HEARTBEAT_ENVELOPE_PREFIX) + 1
    if len(data) != expected_len or not data.startswith(HEARTBEAT_ENVELOPE_PREFIX):
        raise UnsupportedCommandError(Command.ENVELOPE)
    subcommand = data[-1]
    if subcommand in BLACKLISTED_COMMANDS:
        raise BlacklistedCommandError(subcommand, BLACKLISTED_COMMANDS[subcommand])
    if subcommand not in SAFE_ENVELOPE_SUBCOMMANDS:
        raise UnsupportedCommandError(subcommand)


@dataclass(frozen=True)
class Frame:
    """A decoded protocol frame.

    Attributes:
        command: The opcode carried by the frame.
        data: The data field, without the command byte or the checksum.
        raw: The complete on-the-wire bytes, magic and checksum included.
    """

    command: int
    data: bytes
    raw: bytes

    def __len__(self) -> int:
        """Return the on-the-wire size of the frame in bytes."""
        return len(self.raw)

    def __repr__(self) -> str:
        """Return a short, hex-oriented representation."""
        try:
            name = Command(self.command).name
        except ValueError:
            name = "UNKNOWN"
        preview = self.data[:16].hex(" ")
        if len(self.data) > 16:
            preview += f" ... (+{len(self.data) - 16} bytes)"
        return f"Frame(command=0x{self.command:02X} {name}, data=[{preview}])"


def encode_frame(command: int, data: bytes = b"") -> bytes:
    """Build a complete frame for ``command``.

    Args:
        command: The opcode. Validated against the blacklist and allowlist.
        data: The data field, without the command byte or the checksum.

    Returns:
        The complete frame, ready to be written to the serial port.

    Raises:
        BlacklistedCommandError: If the opcode is destructive.
        UnsupportedCommandError: If the opcode is outside the allowlist, or if
            the opcode is the 0x61 envelope and the nested sub-command is not
            allowed.
        FrameError: If the resulting frame would exceed :data:`MAX_FRAME_SIZE`.
    """
    ensure_command_allowed(command)
    if command == Command.ENVELOPE:
        _ensure_envelope_payload_allowed(data)

    length = len(data) + MIN_LENGTH_FIELD
    if length + 3 > MAX_FRAME_SIZE:
        raise FrameError(f"frame of {length + 3} bytes exceeds the {MAX_FRAME_SIZE}-byte cap")

    body = length.to_bytes(2, "little") + bytes([command]) + data
    return MAGIC + body + bytes([checksum(body)])


def decode_frame(raw: bytes) -> Frame:
    """Parse and validate a complete frame.

    Args:
        raw: Exactly one frame, magic and checksum included. Trailing bytes are
            rejected rather than ignored, so that a framing bug cannot pass
            silently.

    Returns:
        The decoded :class:`Frame`.

    Raises:
        FrameError: If the magic, the length field or the overall size is wrong.
        ChecksumError: If the trailing checksum does not match the body.
    """
    if len(raw) < HEADER_SIZE + 1:
        raise FrameError(f"frame of {len(raw)} bytes is shorter than the minimum of 7")
    if not raw.startswith(MAGIC):
        raise FrameError(f"bad magic: {raw[:3].hex(' ')}")

    length = int.from_bytes(raw[3:5], "little")
    if length < MIN_LENGTH_FIELD:
        raise FrameError(f"length field {length} is below the minimum of {MIN_LENGTH_FIELD}")

    expected_size = length + 3
    if len(raw) != expected_size:
        raise FrameError(
            f"length field announces {expected_size} bytes but {len(raw)} were supplied"
        )

    body = raw[3:-1]
    expected = checksum(body)
    actual = raw[-1]
    if expected != actual:
        raise ChecksumError(expected, actual)

    return Frame(command=raw[5], data=bytes(raw[HEADER_SIZE:-1]), raw=bytes(raw))


def encode_heartbeat() -> bytes:
    """Build the heartbeat frame.

    The heartbeat is the 0xFF sub-command wrapped in a 0x61 envelope. It is
    read-only and is the cheapest way to keep the link alive.

    Returns:
        The complete heartbeat frame.
    """
    return encode_frame(
        Command.ENVELOPE, HEARTBEAT_ENVELOPE_PREFIX + bytes([SUBCOMMAND_HEARTBEAT])
    )

"""Frame encoding, decoding and checksum, against the documented golden vectors."""

from __future__ import annotations

import pytest

from eilik.errors import BlacklistedCommandError, ChecksumError, FrameError, UnsupportedCommandError
from eilik.protocol import (
    BLACKLISTED_COMMANDS,
    MAX_FRAME_SIZE,
    RUNNING_NUMBERS,
    SCREEN_HOLD,
    SCREEN_RELEASE,
    Command,
    checksum,
    decode_frame,
    encode_frame,
    encode_heartbeat,
)

from .fake_robot import raw_frame

# Golden vectors captured from a real robot.
HEARTBEAT_TX = bytes.fromhex("aaaaaa0a0061e4c6f1ca83ffad")
PING_TX = bytes.fromhex("aaaaaa04000 1fa".replace(" ", ""))
READ_SERVOS_TX = bytes.fromhex("aaaaaa0400a15a")
READ_SCREEN_TX = bytes.fromhex("aaaaaa0400a358")
WRITE_SERVOS_ACK = bytes.fromhex("aaaaaa0500a20157")
READ_SERVOS_REPLY = bytes.fromhex("aaaaaa1100a10401000002000003000004 00003f".replace(" ", ""))

# Frames sent by a real device, from the community protocol reference
# (PROTOCOL.md in github.com/aklto/PyEilik). Each heartbeat reply carries a
# different nonce whose first and last bytes match.
HEARTBEAT_RX = [
    bytes.fromhex("aaaaaa0a0061cda02e0bcdff22"),
    bytes.fromhex("aaaaaa0a0061666a8d1666ffbc"),
    bytes.fromhex("aaaaaa0a0061fc39e457fcff29"),
]
LEGACY_SERVO_ACK = bytes.fromhex("aaaaaa0a00611c0e46451c03c0")
WRITE_SCREEN_ACK = bytes.fromhex("aaaaaa0500a40155")

# A legacy single-motor servo frame from the official tooling: motor 1 to 2000.
LEGACY_SERVO_TX = bytes.fromhex("aaaaaa140061fc39e457fc03010101d007000000000041")


class TestChecksum:
    """The checksum is 255 minus the low byte of the sum."""

    def test_empty(self):
        assert checksum(b"") == 255

    def test_ping_body(self):
        # 0x04 0x00 0x01 -> 255 - 5 = 0xFA
        assert checksum(bytes.fromhex("040001")) == 0xFA

    def test_wraps_modulo_256(self):
        assert checksum(b"\xff\xff") == 255 - ((255 + 255) % 256)

    @pytest.mark.parametrize(
        "frame",
        [
            HEARTBEAT_TX,
            PING_TX,
            READ_SERVOS_TX,
            READ_SCREEN_TX,
            WRITE_SERVOS_ACK,
            READ_SERVOS_REPLY,
            *HEARTBEAT_RX,
            LEGACY_SERVO_ACK,
            WRITE_SCREEN_ACK,
            LEGACY_SERVO_TX,
        ],
        ids=[
            "heartbeat",
            "ping",
            "read_servos",
            "read_screen",
            "servo_ack",
            "servo_reply",
            "heartbeat_rx_1",
            "heartbeat_rx_2",
            "heartbeat_rx_3",
            "legacy_servo_ack",
            "screen_ack",
            "legacy_servo",
        ],
    )
    def test_golden_vectors_are_self_consistent(self, frame):
        """Every golden vector carries the checksum this implementation computes."""
        assert checksum(frame[3:-1]) == frame[-1]


class TestEncode:
    """Encoding reproduces the golden vectors byte for byte."""

    def test_ping(self):
        assert encode_frame(Command.PING) == PING_TX

    def test_read_servos(self):
        assert encode_frame(Command.READ_SERVOS) == READ_SERVOS_TX

    def test_read_screen(self):
        assert encode_frame(Command.READ_SCREEN) == READ_SCREEN_TX

    def test_heartbeat(self):
        assert encode_heartbeat() == HEARTBEAT_TX

    def test_length_field_is_total_minus_three(self):
        frame = encode_frame(Command.WRITE_SERVOS, b"\x01\x01\xdc\x05")
        assert int.from_bytes(frame[3:5], "little") == len(frame) - 3

    def test_screen_write_is_1031_bytes(self):
        frame = encode_frame(Command.WRITE_SCREEN, bytes(1024))
        assert len(frame) == 1031
        assert frame[3:5] == bytes.fromhex("0404")

    def test_oversized_frame_is_refused(self):
        with pytest.raises(FrameError, match="exceeds"):
            encode_frame(Command.WRITE_SCREEN, bytes(MAX_FRAME_SIZE))


class TestDecode:
    """Decoding validates structure and checksum."""

    def test_servo_ack(self):
        frame = decode_frame(WRITE_SERVOS_ACK)
        assert frame.command == Command.WRITE_SERVOS
        assert frame.data == b"\x01"

    def test_servo_reply(self):
        frame = decode_frame(READ_SERVOS_REPLY)
        assert frame.command == Command.READ_SERVOS
        assert frame.data == bytes.fromhex("04010000020000030000040000")
        assert len(frame) == 20

    def test_roundtrip(self):
        payload = b"\x02\x01\xdc\x05\x04\x78\x05"
        assert decode_frame(encode_frame(Command.WRITE_SERVOS, payload)).data == payload

    def test_corrupt_checksum_is_rejected(self):
        damaged = WRITE_SERVOS_ACK[:-1] + bytes([WRITE_SERVOS_ACK[-1] ^ 0x01])
        with pytest.raises(ChecksumError) as excinfo:
            decode_frame(damaged)
        assert excinfo.value.expected == WRITE_SERVOS_ACK[-1]

    def test_corrupt_body_is_rejected(self):
        damaged = bytearray(READ_SERVOS_REPLY)
        damaged[8] ^= 0xFF
        with pytest.raises(ChecksumError):
            decode_frame(bytes(damaged))

    def test_bad_magic(self):
        with pytest.raises(FrameError, match="bad magic"):
            decode_frame(b"\xab\xab\xab\x04\x00\x01\xfa")

    def test_truncated(self):
        with pytest.raises(FrameError, match="shorter than"):
            decode_frame(PING_TX[:5])

    def test_trailing_bytes_are_rejected(self):
        with pytest.raises(FrameError, match="announces"):
            decode_frame(PING_TX + b"\x00")

    def test_length_below_minimum(self):
        with pytest.raises(FrameError, match="below the minimum"):
            decode_frame(b"\xaa\xaa\xaa\x01\x00\x01\x00")

    def test_repr_names_the_command(self):
        assert "READ_SERVOS" in repr(decode_frame(READ_SERVOS_REPLY))


class TestRealDeviceCaptures:
    """Frames a real robot sent decode with this implementation's checksum."""

    @pytest.mark.parametrize("frame", HEARTBEAT_RX, ids=["cda02e", "666a8d", "fc39e4"])
    def test_heartbeat_replies(self, frame):
        decoded = decode_frame(frame)
        assert decoded.command == Command.ENVELOPE
        nonce, echo = decoded.data[:5], decoded.data[5:]
        assert echo == b"\xff"
        assert nonce[0] == nonce[4]

    def test_heartbeat_replies_do_not_reuse_the_transmitted_nonce(self):
        """The nonce changes in every frame the device sends; it is not a token."""
        nonces = {decode_frame(frame).data[:5] for frame in HEARTBEAT_RX}
        assert len(nonces) == len(HEARTBEAT_RX)
        assert decode_frame(HEARTBEAT_TX).data[:5] not in nonces

    def test_legacy_servo_ack_echoes_the_subcommand(self):
        assert decode_frame(LEGACY_SERVO_ACK).data[5:] == b"\x03"

    def test_screen_write_ack(self):
        frame = decode_frame(WRITE_SCREEN_ACK)
        assert (frame.command, frame.data) == (Command.WRITE_SCREEN, b"\x01")

    def test_legacy_servo_frame_is_well_formed(self):
        """It decodes, so only the guard stands between it and the wire."""
        assert decode_frame(LEGACY_SERVO_TX).data[5] == 0x03


class TestSafetyGuard:
    """Destructive opcodes cannot be encoded, under any spelling."""

    def test_every_documented_destructive_opcode_is_listed(self):
        assert set(BLACKLISTED_COMMANDS) == {0x02, 0x03, 0x04, 0x05, 0x31, 0x41, 0x42}

    @pytest.mark.parametrize("opcode", sorted(BLACKLISTED_COMMANDS))
    def test_blacklisted_opcodes_cannot_be_encoded(self, opcode):
        with pytest.raises(BlacklistedCommandError) as excinfo:
            encode_frame(opcode)
        assert excinfo.value.command == opcode
        assert BLACKLISTED_COMMANDS[opcode].split(" ")[0] in str(excinfo.value)

    def test_blacklist_and_allowlist_are_disjoint(self):
        assert not set(BLACKLISTED_COMMANDS) & {int(member) for member in Command}

    def test_unknown_opcode_is_refused(self):
        with pytest.raises(UnsupportedCommandError):
            encode_frame(0x7B)

    def test_blacklist_is_immutable(self):
        with pytest.raises(TypeError):
            BLACKLISTED_COMMANDS[0x42] = "oops"  # type: ignore[index]

    def test_envelope_rejects_a_smuggled_destructive_subcommand(self):
        """A blacklisted opcode nested in a 0x61 envelope is caught too."""
        payload = bytes.fromhex("e4c6f1ca83") + bytes([0x42])
        with pytest.raises(BlacklistedCommandError) as excinfo:
            encode_frame(Command.ENVELOPE, payload)
        assert excinfo.value.command == 0x42

    def test_envelope_rejects_an_unknown_subcommand(self):
        payload = bytes.fromhex("e4c6f1ca83") + bytes([0x7B])
        with pytest.raises(UnsupportedCommandError):
            encode_frame(Command.ENVELOPE, payload)

    def test_envelope_rejects_a_malformed_payload(self):
        with pytest.raises(UnsupportedCommandError):
            encode_frame(Command.ENVELOPE, b"\x00\x01\x02")

    def test_envelope_refuses_the_unclamped_legacy_servo_command(self):
        """Inside 0x61, 0x03 is the legacy servo command, which skips clamping."""
        data = decode_frame(LEGACY_SERVO_TX).data
        with pytest.raises(BlacklistedCommandError, match="legacy servo") as excinfo:
            encode_frame(Command.ENVELOPE, data)
        assert excinfo.value.command == 0x03

    def test_envelope_allows_only_the_reference_heartbeat(self):
        """A heartbeat with another nonce works on the robot, but is not needed."""
        with pytest.raises(UnsupportedCommandError):
            encode_frame(Command.ENVELOPE, bytes.fromhex("cda02e0bcd") + b"\xff")

    def test_envelope_rejects_trailing_bytes_after_the_heartbeat(self):
        with pytest.raises(UnsupportedCommandError):
            encode_frame(Command.ENVELOPE, bytes.fromhex("e4c6f1ca83ff00"))

    def test_raw_frame_helper_can_still_build_one(self):
        """The test helper deliberately bypasses the guard.

        That is what makes the transport-level guard worth testing separately.
        """
        assert raw_frame(0x42)[5] == 0x42


class TestRunningNumbers:
    """0xA6 goes out with the two documented values only."""

    @pytest.mark.parametrize("value", [SCREEN_HOLD, SCREEN_RELEASE])
    def test_documented_values_are_allowed(self, value):
        assert encode_frame(Command.WRITE_RUNNING_NUMBER, bytes([value]))[6] == value

    @pytest.mark.parametrize("data", [b"\x05", b"\x01", b"\xff", b"", b"\x64\x00"])
    def test_anything_else_is_refused(self, data):
        with pytest.raises(UnsupportedCommandError, match="no known effect"):
            encode_frame(Command.WRITE_RUNNING_NUMBER, data)

    def test_the_values_are_documented(self):
        assert set(RUNNING_NUMBERS) == {0, 100}
        assert all(RUNNING_NUMBERS.values())

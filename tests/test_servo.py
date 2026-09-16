"""Servo addressing, clamping and payload (de)serialisation."""

from __future__ import annotations

import pytest

from eilik.errors import ProtocolError, ServoRangeWarning
from eilik.servo import (
    MAX_MOTORS_PER_FRAME,
    NEUTRAL_POSITION,
    VERIFIED_RANGES,
    Motor,
    ServoLimits,
    decode_servo_payload,
    describe,
    encode_servo_payload,
    neutral_positions,
)


class TestMotorTable:
    """Motor ids match the documented table."""

    def test_ids(self):
        assert (Motor.ARM_RIGHT, Motor.ARM_LEFT, Motor.BODY, Motor.HEAD) == (1, 2, 3, 4)

    def test_every_motor_has_a_verified_range(self):
        assert set(VERIFIED_RANGES) == set(Motor)

    def test_arms_are_wider_than_body_and_head(self):
        assert VERIFIED_RANGES[Motor.ARM_RIGHT] == (1000, 2000)
        assert VERIFIED_RANGES[Motor.HEAD] == (1200, 1800)

    def test_neutral_is_inside_every_verified_range(self):
        for motor, (low, high) in VERIFIED_RANGES.items():
            assert low <= NEUTRAL_POSITION <= high, motor


class TestClamping:
    """Default limits clamp silently; widened limits warn."""

    @pytest.mark.parametrize(
        ("motor", "requested", "expected"),
        [
            (Motor.ARM_RIGHT, 500, 1000),
            (Motor.ARM_RIGHT, 9999, 2000),
            (Motor.HEAD, 0, 1200),
            (Motor.HEAD, 65535, 1800),
            (Motor.BODY, 1500, 1500),
        ],
    )
    def test_clamps_to_the_verified_range(self, motor, requested, expected):
        assert ServoLimits.verified().clamp(motor, requested) == expected

    def test_clamping_inward_does_not_warn(self):
        """Clamping inward must stay silent.

        Pulling a position back into the verified range is the safe direction.
        """
        import warnings

        with warnings.catch_warnings():
            warnings.simplefilter("error")
            assert ServoLimits.verified().clamp(Motor.HEAD, 9999) == 1800

    def test_widened_limits_warn_outside_the_verified_range(self):
        limits = ServoLimits.widened(HEAD=(1000, 2000))
        with pytest.warns(ServoRangeWarning, match="outside the verified range 1200-1800"):
            assert limits.clamp(Motor.HEAD, 1950) == 1950

    def test_widened_limits_still_clamp_to_their_own_bounds(self):
        limits = ServoLimits.widened(HEAD=(1000, 2000))
        with pytest.warns(ServoRangeWarning):
            assert limits.clamp(Motor.HEAD, 5000) == 2000

    def test_widened_limits_do_not_warn_inside_the_verified_range(self):
        import warnings

        limits = ServoLimits.widened(HEAD=(1000, 2000))
        with warnings.catch_warnings():
            warnings.simplefilter("error")
            assert limits.clamp(Motor.HEAD, 1500) == 1500

    def test_widened_leaves_other_motors_at_verified_ranges(self):
        limits = ServoLimits.widened(HEAD=(1000, 2000))
        assert limits.ranges[Motor.BODY] == VERIFIED_RANGES[Motor.BODY]

    def test_widening_does_not_mutate_the_shared_verified_table(self):
        ServoLimits.widened(HEAD=(1000, 2000))
        assert VERIFIED_RANGES[Motor.HEAD] == (1200, 1800)

    @pytest.mark.parametrize("bad", [(2000, 1000), (0, 70000)])
    def test_invalid_widened_ranges_are_refused(self, bad):
        with pytest.raises(ValueError):
            ServoLimits.widened(HEAD=bad)

    def test_unknown_motor_name_is_refused(self):
        with pytest.raises(KeyError):
            ServoLimits.widened(ELBOW=(1000, 2000))

    @pytest.mark.parametrize("bad", [1500.5, "1500", None, True])
    def test_non_integer_positions_are_refused(self, bad):
        with pytest.raises(ValueError, match="must be an int"):
            ServoLimits.verified().clamp(Motor.BODY, bad)


class TestEncodePayload:
    """0xA2 payloads are ``count`` followed by little-endian triplets."""

    def test_single_motor(self):
        assert encode_servo_payload({Motor.ARM_RIGHT: 1500}) == bytes.fromhex("0101dc05")

    def test_multiple_motors_are_sorted_by_id(self):
        payload = encode_servo_payload({Motor.HEAD: 1400, Motor.ARM_RIGHT: 1500})
        assert payload == bytes.fromhex("0201dc05047805")

    def test_all_four_motors_fit(self):
        payload = encode_servo_payload(neutral_positions())
        assert payload[0] == MAX_MOTORS_PER_FRAME
        assert len(payload) == 1 + 4 * 3

    def test_accepts_names_and_ids(self):
        by_enum = encode_servo_payload({Motor.HEAD: 1600})
        assert encode_servo_payload({"head": 1600}) == by_enum
        assert encode_servo_payload({4: 1600}) == by_enum

    def test_positions_are_clamped_before_encoding(self):
        assert encode_servo_payload({Motor.HEAD: 9999}) == bytes.fromhex("01040807")

    def test_empty_mapping_is_refused(self):
        with pytest.raises(ValueError, match="no motor positions"):
            encode_servo_payload({})

    def test_duplicate_motor_is_refused(self):
        with pytest.raises(ValueError, match="more than once"):
            encode_servo_payload({Motor.HEAD: 1500, "head": 1600})

    @pytest.mark.parametrize("bad", [0, 9, "elbow", 1.5])
    def test_unknown_motor_is_refused(self, bad):
        with pytest.raises(ValueError):
            encode_servo_payload({bad: 1500})


class TestDecodePayload:
    """0xA1 replies round-trip back to a motor mapping."""

    def test_golden_reply(self):
        data = bytes.fromhex("04010000020000030000040000")
        assert decode_servo_payload(data) == dict.fromkeys(Motor, 0)

    def test_little_endian_positions(self):
        assert decode_servo_payload(bytes.fromhex("0101dc05")) == {Motor.ARM_RIGHT: 1500}

    def test_roundtrip(self):
        positions = {Motor.ARM_LEFT: 1100, Motor.BODY: 1700}
        assert decode_servo_payload(encode_servo_payload(positions)) == positions

    def test_unknown_motor_ids_are_skipped(self):
        """A firmware revision reporting a fifth motor must not break reads."""
        data = bytes.fromhex("02" + "01dc05" + "09ff00")
        assert decode_servo_payload(data) == {Motor.ARM_RIGHT: 1500}

    def test_empty_payload_is_refused(self):
        with pytest.raises(ProtocolError, match="empty"):
            decode_servo_payload(b"")

    def test_count_mismatch_is_refused(self):
        with pytest.raises(ProtocolError, match="announces 4 motors"):
            decode_servo_payload(bytes.fromhex("0401dc05"))


def test_describe_is_ordered_by_motor_id():
    assert describe({Motor.HEAD: 1400, Motor.ARM_RIGHT: 1500}) == "ARM_RIGHT=1500, HEAD=1400"

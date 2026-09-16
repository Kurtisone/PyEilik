"""Servo addressing, position limits and payload (de)serialisation.

Positions are ``uint16`` little-endian pulse widths. 1500 is the neutral
position for every motor.

The ranges in :data:`VERIFIED_RANGES` are the ones that have actually been
exercised on hardware. Driving a servo past its verified range risks pushing it
against a mechanical stop, which the firmware will not protect against, so the
SDK clamps to these ranges by default.
"""

from __future__ import annotations

import warnings
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from enum import IntEnum
from types import MappingProxyType

from .errors import ProtocolError, ServoRangeWarning

__all__ = [
    "MAX_MOTORS_PER_FRAME",
    "NEUTRAL_POSITION",
    "VERIFIED_RANGES",
    "Motor",
    "ServoLimits",
    "decode_servo_payload",
    "encode_servo_payload",
]


class Motor(IntEnum):
    """The four addressable servos.

    Left and right are given from the robot's own point of view, so
    :attr:`ARM_RIGHT` is the arm on your left when the robot faces you.
    """

    ARM_RIGHT = 1
    """Right arm of the robot (on the observer's left when facing it)."""

    ARM_LEFT = 2
    """Left arm of the robot (on the observer's right when facing it)."""

    BODY = 3
    """Torso rotation."""

    HEAD = 4
    """Head rotation."""


#: Pulse width that centres every motor.
NEUTRAL_POSITION = 1500

#: The firmware accepts at most four motor triplets in a single 0xA2 frame.
MAX_MOTORS_PER_FRAME = 4

#: Bytes per motor in a 0xA1/0xA2 payload: id, position low, position high.
BYTES_PER_MOTOR = 3

#: Ranges that have been exercised on real hardware.
#:
#: The arms have been swept across 1000-2000. The body and head have only been
#: driven through small excursions, so their verified range is deliberately
#: narrow; widening it means measuring the mechanical stops first.
VERIFIED_RANGES: Mapping[Motor, tuple[int, int]] = MappingProxyType(
    {
        Motor.ARM_RIGHT: (1000, 2000),
        Motor.ARM_LEFT: (1000, 2000),
        Motor.BODY: (1200, 1800),
        Motor.HEAD: (1200, 1800),
    }
)

#: Positions are encoded as ``uint16``, so nothing outside this range can be
#: represented on the wire regardless of the configured limits.
ENCODABLE_RANGE = (0, 0xFFFF)


@dataclass(frozen=True)
class ServoLimits:
    """Position limits applied before a 0xA2 frame is built.

    By default the limits are :data:`VERIFIED_RANGES` and clamping is silent,
    because clamping to a verified range is the safe outcome. Supplying custom
    ranges is allowed, but any position that ends up outside the verified range
    raises a :class:`~eilik.errors.ServoRangeWarning`, so widening the limits is
    always visible in the logs.

    Attributes:
        ranges: Inclusive ``(minimum, maximum)`` per motor.
    """

    ranges: Mapping[Motor, tuple[int, int]] = field(default_factory=lambda: VERIFIED_RANGES)

    @classmethod
    def verified(cls) -> ServoLimits:
        """Return the default limits, i.e. the empirically verified ranges."""
        return cls(VERIFIED_RANGES)

    @classmethod
    def widened(cls, **overrides: tuple[int, int]) -> ServoLimits:
        """Return limits based on the verified ranges with some motors relaxed.

        Args:
            **overrides: Motor name to ``(minimum, maximum)``, for example
                ``ServoLimits.widened(HEAD=(1100, 1900))``.

        Returns:
            A new :class:`ServoLimits`.

        Raises:
            KeyError: If a name does not match a :class:`Motor` member.
            ValueError: If a range is inverted or not encodable as ``uint16``.
        """
        ranges: dict[Motor, tuple[int, int]] = dict(VERIFIED_RANGES)
        for name, bounds in overrides.items():
            motor = Motor[name]
            low, high = bounds
            if low > high:
                raise ValueError(f"inverted range for {motor.name}: {bounds!r}")
            if low < ENCODABLE_RANGE[0] or high > ENCODABLE_RANGE[1]:
                raise ValueError(f"range for {motor.name} is not encodable as uint16: {bounds!r}")
            ranges[motor] = (int(low), int(high))
        return cls(MappingProxyType(ranges))

    def clamp(self, motor: Motor, position: int) -> int:
        """Clamp ``position`` for ``motor`` and warn when leaving safe ground.

        Args:
            motor: The motor the position is destined for.
            position: The requested pulse width.

        Returns:
            The position actually safe to transmit.

        Raises:
            ValueError: If ``position`` is not an integer, or if the clamped
                result cannot be encoded as ``uint16``.

        Warns:
            ServoRangeWarning: If the requested position was outside the
                verified range for this motor, whether or not the configured
                limits allowed it through.
        """
        if isinstance(position, bool) or not isinstance(position, int):
            raise ValueError(f"position for {motor.name} must be an int, got {position!r}")

        low, high = self.ranges[motor]
        clamped = min(max(position, low), high)

        verified_low, verified_high = VERIFIED_RANGES[motor]
        if not verified_low <= clamped <= verified_high:
            warnings.warn(
                f"{motor.name} position {clamped} is outside the verified range "
                f"{verified_low}-{verified_high}; this risks driving the servo into a "
                f"mechanical stop",
                ServoRangeWarning,
                stacklevel=3,
            )

        if not ENCODABLE_RANGE[0] <= clamped <= ENCODABLE_RANGE[1]:
            raise ValueError(f"position {clamped} for {motor.name} is not encodable as uint16")
        return clamped


def _coerce_motor(key: object) -> Motor:
    """Convert a motor name, id or :class:`Motor` into a :class:`Motor`.

    Raises:
        ValueError: If ``key`` does not designate a known motor.
    """
    if isinstance(key, Motor):
        return key
    if isinstance(key, str):
        try:
            return Motor[key.upper()]
        except KeyError:
            raise ValueError(f"unknown motor name {key!r}") from None
    if isinstance(key, int) and not isinstance(key, bool):
        try:
            return Motor(key)
        except ValueError:
            raise ValueError(f"unknown motor id {key}") from None
    raise ValueError(f"cannot interpret {key!r} as a motor")


def encode_servo_payload(
    positions: Mapping[object, int],
    limits: ServoLimits | None = None,
) -> bytes:
    """Build the data field of a 0xA2 frame.

    Args:
        positions: Motor (as a :class:`Motor`, an id or a name) to pulse width.
        limits: Limits to apply. Defaults to :meth:`ServoLimits.verified`.

    Returns:
        ``<count> <id, position_lo, position_hi> * count``.

    Raises:
        ValueError: If ``positions`` is empty, holds more than
            :data:`MAX_MOTORS_PER_FRAME` entries, names an unknown motor, or
            names the same motor twice.
    """
    limits = limits or ServoLimits.verified()

    resolved: dict[Motor, int] = {}
    for key, position in positions.items():
        motor = _coerce_motor(key)
        if motor in resolved:
            raise ValueError(f"motor {motor.name} specified more than once")
        resolved[motor] = limits.clamp(motor, position)

    if not resolved:
        raise ValueError("no motor positions supplied")
    if len(resolved) > MAX_MOTORS_PER_FRAME:
        raise ValueError(
            f"at most {MAX_MOTORS_PER_FRAME} motors fit in one frame, got {len(resolved)}"
        )

    payload = bytearray([len(resolved)])
    for motor in sorted(resolved):
        payload.append(int(motor))
        payload.extend(resolved[motor].to_bytes(2, "little"))
    return bytes(payload)


def decode_servo_payload(data: bytes) -> dict[Motor, int]:
    """Parse the data field of a 0xA1 reply.

    Args:
        data: ``<count> <id, position_lo, position_hi> * count``.

    Returns:
        Motor to pulse width, for every motor the robot reported. Motors the
        firmware reports with an id outside :class:`Motor` are skipped rather
        than raising, so a firmware revision that grows a fifth motor does not
        break reads.

    Raises:
        ProtocolError: If the payload is empty or its length does not match the
            announced motor count.
    """
    if not data:
        raise ProtocolError("empty servo payload")

    count = data[0]
    expected = 1 + count * BYTES_PER_MOTOR
    if len(data) != expected:
        raise ProtocolError(
            f"servo payload announces {count} motors ({expected} bytes) "
            f"but carries {len(data)} bytes"
        )

    positions: dict[Motor, int] = {}
    for index in range(count):
        offset = 1 + index * BYTES_PER_MOTOR
        motor_id = data[offset]
        position = int.from_bytes(data[offset + 1 : offset + 3], "little")
        try:
            positions[Motor(motor_id)] = position
        except ValueError:
            continue
    return positions


def describe(positions: Mapping[Motor, int]) -> str:
    """Return a one-line human-readable summary of a servo position mapping."""
    return ", ".join(f"{motor.name}={positions[motor]}" for motor in sorted(positions))


def neutral_positions(motors: Iterable[Motor] | None = None) -> dict[Motor, int]:
    """Return ``{motor: NEUTRAL_POSITION}`` for ``motors`` (default: all four)."""
    return dict.fromkeys(motors if motors is not None else Motor, NEUTRAL_POSITION)

"""Pure-Python driver for the Energize Lab Eilik desktop robot on Linux.

The robot enumerates as a USB CDC-ACM serial device. This package implements the
community-reverse-engineered frame protocol directly in Python, so no vendor
library or platform-specific baud-rate hack is required.

Only non-destructive commands are implemented. The opcodes that rewrite
firmware or the SD card are listed in
:data:`eilik.protocol.BLACKLISTED_COMMANDS` and are refused at every point where
bytes can reach the serial port.

Example:
    >>> from eilik import Eilik, Motor
    >>> with Eilik() as robot:  # doctest: +SKIP
    ...     print(robot.ping())
    ...     robot.write_servos({Motor.HEAD: 1600})
"""

from .errors import (
    AmbiguousPortError,
    BlacklistedCommandError,
    ChecksumError,
    EilikConnectionError,
    EilikError,
    EilikTimeoutError,
    FrameError,
    PortBusyError,
    PortNotFoundError,
    ProtocolError,
    ServoControllerFaultError,
    ServoRangeWarning,
    UnsupportedCommandError,
)
from .protocol import (
    BLACKLISTED_COMMANDS,
    Command,
    Frame,
    checksum,
    decode_frame,
    encode_frame,
    encode_heartbeat,
)
from .robot import Eilik, FirmwareInfo
from .screen import (
    FRAMEBUFFER_SIZE,
    HEIGHT,
    WIDTH,
    blank,
    get_pixel,
    rotate180,
    save_png,
    set_pixel,
)
from .servo import NEUTRAL_POSITION, VERIFIED_RANGES, Motor, ServoLimits
from .transport import DEFAULT_BAUDRATE, SerialTransport, autodetect_port, list_candidate_ports

__version__ = "0.1.0"

__all__ = [
    "BLACKLISTED_COMMANDS",
    "DEFAULT_BAUDRATE",
    "FRAMEBUFFER_SIZE",
    "HEIGHT",
    "NEUTRAL_POSITION",
    "VERIFIED_RANGES",
    "WIDTH",
    "AmbiguousPortError",
    "BlacklistedCommandError",
    "ChecksumError",
    "Command",
    "Eilik",
    "EilikConnectionError",
    "EilikError",
    "EilikTimeoutError",
    "FirmwareInfo",
    "Frame",
    "FrameError",
    "Motor",
    "PortBusyError",
    "PortNotFoundError",
    "ProtocolError",
    "SerialTransport",
    "ServoControllerFaultError",
    "ServoLimits",
    "ServoRangeWarning",
    "UnsupportedCommandError",
    "__version__",
    "autodetect_port",
    "blank",
    "checksum",
    "decode_frame",
    "encode_frame",
    "encode_heartbeat",
    "get_pixel",
    "list_candidate_ports",
    "rotate180",
    "save_png",
    "set_pixel",
]

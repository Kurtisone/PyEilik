"""Framebuffer helpers for the 128x64 monochrome display.

The panel is an SSD1306-style display driven in page mode: the buffer holds
eight 128-byte pages, each byte covering eight vertically stacked pixels, least
significant bit on top::

    bit = (buffer[(y // 8) * 128 + x] >> (y % 8)) & 1

The panel itself is mounted upside down relative to that natural ordering, so
every buffer read and write goes through :func:`rotate180`. Functions in this
module take and return *logical* framebuffers, i.e. already the right way up;
:class:`~eilik.robot.Eilik` applies the rotation on the wire.
"""

from __future__ import annotations

import struct
import zlib
from collections.abc import Iterable, Sequence
from pathlib import Path

from .errors import ProtocolError

__all__ = [
    "FRAMEBUFFER_SIZE",
    "HEIGHT",
    "PAGES",
    "WIDTH",
    "blank",
    "check_size",
    "get_pixel",
    "rotate180",
    "save_png",
    "set_pixel",
    "to_ascii",
]

#: Display width in pixels.
WIDTH = 128

#: Display height in pixels.
HEIGHT = 64

#: Number of 8-pixel-tall pages stacked vertically.
PAGES = HEIGHT // 8

#: Size of a complete framebuffer in bytes.
FRAMEBUFFER_SIZE = WIDTH * PAGES

#: Lookup table reversing the bit order of a byte, used by :func:`rotate180`.
_BIT_REVERSE = bytes(int(format(value, "08b")[::-1], 2) for value in range(256))


def check_size(framebuffer: bytes) -> None:
    """Raise :class:`~eilik.errors.ProtocolError` unless the buffer is 1024 bytes.

    Args:
        framebuffer: The buffer to check.
    """
    if len(framebuffer) != FRAMEBUFFER_SIZE:
        raise ProtocolError(
            f"framebuffer must be exactly {FRAMEBUFFER_SIZE} bytes, got {len(framebuffer)}"
        )


def rotate180(framebuffer: bytes) -> bytes:
    """Rotate a framebuffer by 180 degrees.

    In page mode, rotating the image by half a turn maps the pixel at ``(x, y)``
    to ``(WIDTH - 1 - x, HEIGHT - 1 - y)``. Because a page byte stacks eight
    pixels along ``y``, that works out to reversing the byte order of the whole
    buffer and reversing the bits inside each byte, which is what this does.

    The operation is its own inverse, so the same call serves both the read and
    the write path.

    Args:
        framebuffer: A 1024-byte buffer.

    Returns:
        The rotated buffer.

    Raises:
        ProtocolError: If the buffer is not 1024 bytes.
    """
    check_size(framebuffer)
    return bytes(_BIT_REVERSE[byte] for byte in reversed(framebuffer))


def blank(value: int = 0) -> bytearray:
    """Return a fresh framebuffer filled with ``value`` (0 clears, 0xFF fills)."""
    return bytearray([value & 0xFF]) * FRAMEBUFFER_SIZE


def get_pixel(framebuffer: Sequence[int], x: int, y: int) -> int:
    """Return the pixel at ``(x, y)`` as 0 or 1.

    Args:
        framebuffer: A 1024-byte buffer.
        x: Column, ``0 <= x < 128``.
        y: Row, ``0 <= y < 64``.

    Returns:
        1 if the pixel is lit, 0 otherwise.

    Raises:
        IndexError: If the coordinates are off-screen.
    """
    if not (0 <= x < WIDTH and 0 <= y < HEIGHT):
        raise IndexError(f"({x}, {y}) is outside the {WIDTH}x{HEIGHT} display")
    return (framebuffer[(y // 8) * WIDTH + x] >> (y % 8)) & 1


def set_pixel(framebuffer: bytearray, x: int, y: int, value: int = 1) -> None:
    """Set the pixel at ``(x, y)`` in place.

    Args:
        framebuffer: A mutable 1024-byte buffer.
        x: Column, ``0 <= x < 128``.
        y: Row, ``0 <= y < 64``.
        value: Truthy to light the pixel, falsy to clear it.

    Raises:
        IndexError: If the coordinates are off-screen.
    """
    if not (0 <= x < WIDTH and 0 <= y < HEIGHT):
        raise IndexError(f"({x}, {y}) is outside the {WIDTH}x{HEIGHT} display")
    index = (y // 8) * WIDTH + x
    mask = 1 << (y % 8)
    if value:
        framebuffer[index] |= mask
    else:
        framebuffer[index] &= ~mask & 0xFF


def to_rows(framebuffer: Sequence[int]) -> list[list[int]]:
    """Return the framebuffer as ``HEIGHT`` rows of ``WIDTH`` pixel values."""
    return [[get_pixel(framebuffer, x, y) for x in range(WIDTH)] for y in range(HEIGHT)]


def to_ascii(framebuffer: Sequence[int], lit: str = "#", dark: str = ".") -> str:
    """Render the framebuffer as ASCII art, one character per pixel.

    Useful for eyeballing a screen read straight in a terminal, without needing
    an image viewer.
    """
    return "\n".join("".join(lit if px else dark for px in row) for row in to_rows(framebuffer))


def _png_chunk(tag: bytes, payload: bytes) -> bytes:
    """Return a length-prefixed, CRC-suffixed PNG chunk."""
    return (
        struct.pack(">I", len(payload))
        + tag
        + payload
        + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)
    )


def save_png(framebuffer: Sequence[int], path: str, scale: int = 4) -> None:
    """Write the framebuffer to an 8-bit greyscale PNG.

    Implemented with :mod:`zlib` and :mod:`struct` only, so reading the screen
    into a viewable image needs no imaging dependency on the target machine.

    Args:
        framebuffer: A 1024-byte buffer, already the right way up.
        path: Destination file path.
        scale: Integer magnification; the default of 4 makes a 128x64 panel
            legible at 512x256.

    Raises:
        ValueError: If ``scale`` is below 1.
        ProtocolError: If the buffer is not 1024 bytes.
    """
    if scale < 1:
        raise ValueError(f"scale must be >= 1, got {scale}")
    check_size(bytes(framebuffer))

    width, height = WIDTH * scale, HEIGHT * scale
    raw = bytearray()
    for row in to_rows(framebuffer):
        scaled = b"".join(bytes([0xFF if px else 0x00]) * scale for px in row)
        for _ in range(scale):
            raw.append(0)  # PNG filter type 0 (None)
            raw.extend(scaled)

    header = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)  # 8-bit greyscale
    png = (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", header)
        + _png_chunk(b"IDAT", zlib.compress(bytes(raw), 9))
        + _png_chunk(b"IEND", b"")
    )
    Path(path).write_bytes(png)


def from_rows(rows: Iterable[Iterable[int]]) -> bytearray:
    """Build a framebuffer from ``HEIGHT`` iterables of ``WIDTH`` pixel values.

    Raises:
        ValueError: If the shape is not 64 rows of 128 pixels.
    """
    framebuffer = blank()
    row_list = [list(row) for row in rows]
    if len(row_list) != HEIGHT or any(len(row) != WIDTH for row in row_list):
        raise ValueError(f"expected {HEIGHT} rows of {WIDTH} pixels")
    for y, row in enumerate(row_list):
        for x, value in enumerate(row):
            if value:
                set_pixel(framebuffer, x, y, 1)
    return framebuffer

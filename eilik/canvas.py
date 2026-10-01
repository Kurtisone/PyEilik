"""Drawing on the 128x64 display: pixels, lines, rectangles, circles and text.

A :class:`Canvas` wraps a logical framebuffer, the right way up, with the origin
in the top-left corner. Drawing is clipped at the edges, so a shape may extend
past the screen without raising. Hand the canvas straight to
:meth:`~eilik.robot.Eilik.write_screen`, which applies the panel's rotation on
the way out::

    canvas = Canvas()
    canvas.rect(0, 0, 128, 64)
    canvas.text(6, 24, "Bonjour !", scale=2)
    robot.write_screen(canvas)
"""

from __future__ import annotations

from collections.abc import Sequence

from . import font
from . import screen as screen_module

__all__ = ["Canvas", "text_size"]


def text_size(text: str, scale: int = 1) -> tuple[int, int]:
    """Return the ``(width, height)`` in pixels that :meth:`Canvas.text` covers.

    The spacing after the last character and below the last line is not
    counted, so the result can be used directly to centre text.

    Args:
        text: The text, possibly over several lines.
        scale: The integer magnification the text will be drawn at.

    Raises:
        ValueError: If ``scale`` is below 1.
    """
    _check_scale(scale)
    lines = text.split("\n")
    longest = max(len(line) for line in lines)
    width = max(0, longest * font.CELL_WIDTH - 1)
    height = len(lines) * font.CELL_HEIGHT - 1
    return width * scale, height * scale


def _check_scale(scale: int) -> None:
    """Raise ValueError unless ``scale`` is a positive integer."""
    if isinstance(scale, bool) or not isinstance(scale, int) or scale < 1:
        raise ValueError(f"scale must be a positive integer, got {scale!r}")


class Canvas:
    """A framebuffer with drawing operations.

    For every drawing method, a truthy ``value`` lights pixels and a falsy one
    clears them, so the same calls erase as well as draw.

    Args:
        framebuffer: Initial contents, 1024 bytes the right way up, e.g. from
            :meth:`~eilik.robot.Eilik.read_screen`. Copied, never modified.
            Defaults to a blank screen.

    Raises:
        ProtocolError: If ``framebuffer`` is not 1024 bytes.
    """

    width = screen_module.WIDTH
    height = screen_module.HEIGHT

    def __init__(self, framebuffer: Sequence[int] | None = None) -> None:
        """Start from ``framebuffer``, or from a blank screen."""
        if framebuffer is None:
            self.buffer = screen_module.blank()
        else:
            data = bytes(framebuffer)
            screen_module.check_size(data)
            self.buffer = bytearray(data)

    def __bytes__(self) -> bytes:
        """Return the framebuffer, ready for :meth:`~eilik.robot.Eilik.write_screen`."""
        return bytes(self.buffer)

    def __eq__(self, other: object) -> bool:
        """Two canvases are equal when they hold the same pixels."""
        if not isinstance(other, Canvas):
            return NotImplemented
        return self.buffer == other.buffer

    __hash__ = None  # type: ignore[assignment]  # mutable

    def __repr__(self) -> str:
        """Return a short description with the number of lit pixels."""
        lit = sum(bin(byte).count("1") for byte in self.buffer)
        return f"Canvas({self.width}x{self.height}, {lit} lit)"

    def copy(self) -> Canvas:
        """Return an independent copy."""
        return Canvas(self.buffer)

    # -- whole-screen operations ---------------------------------------------

    def clear(self, value: int = 0) -> None:
        """Set every pixel: cleared by default, lit if ``value`` is truthy."""
        fill = 0xFF if value else 0x00
        self.buffer[:] = bytes([fill]) * len(self.buffer)

    def invert(self) -> None:
        """Swap lit and dark pixels across the whole screen."""
        self.buffer[:] = bytes(byte ^ 0xFF for byte in self.buffer)

    # -- pixels --------------------------------------------------------------

    def pixel(self, x: int, y: int, value: int = 1) -> None:
        """Set one pixel. Coordinates off the screen are ignored."""
        if 0 <= x < self.width and 0 <= y < self.height:
            index = (y >> 3) * self.width + x
            mask = 1 << (y & 7)
            if value:
                self.buffer[index] |= mask
            else:
                self.buffer[index] &= ~mask & 0xFF

    def get_pixel(self, x: int, y: int) -> int:
        """Return the pixel at ``(x, y)`` as 0 or 1.

        Raises:
            IndexError: If the coordinates are off the screen.
        """
        return screen_module.get_pixel(self.buffer, x, y)

    # -- shapes --------------------------------------------------------------

    def _span(self, x0: int, x1: int, y: int, value: int) -> None:
        """Set the horizontal run from ``x0`` to ``x1`` inclusive on row ``y``."""
        if not 0 <= y < self.height:
            return
        for x in range(max(0, min(x0, x1)), min(self.width - 1, max(x0, x1)) + 1):
            self.pixel(x, y, value)

    def line(self, x0: int, y0: int, x1: int, y1: int, value: int = 1) -> None:
        """Draw a straight line between two points, both ends included."""
        # Bresenham, valid in every octant.
        dx, dy = abs(x1 - x0), -abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = dx + dy
        while True:
            self.pixel(x0, y0, value)
            if x0 == x1 and y0 == y1:
                return
            doubled = 2 * error
            if doubled >= dy:
                error += dy
                x0 += step_x
            if doubled <= dx:
                error += dx
                y0 += step_y

    def rect(
        self, x: int, y: int, width: int, height: int, value: int = 1, fill: bool = False
    ) -> None:
        """Draw a rectangle whose top-left corner is ``(x, y)``.

        Args:
            x: Left edge.
            y: Top edge.
            width: Width in pixels; nothing is drawn if it is not positive.
            height: Height in pixels; nothing is drawn if it is not positive.
            value: Light (truthy) or clear (falsy).
            fill: Fill the inside instead of drawing the outline only.
        """
        if width <= 0 or height <= 0:
            return
        right, bottom = x + width - 1, y + height - 1
        if fill:
            for row in range(y, bottom + 1):
                self._span(x, right, row, value)
            return
        self._span(x, right, y, value)
        self._span(x, right, bottom, value)
        for row in range(y + 1, bottom):
            self.pixel(x, row, value)
            self.pixel(right, row, value)

    def circle(self, cx: int, cy: int, radius: int, value: int = 1, fill: bool = False) -> None:
        """Draw a circle centred on ``(cx, cy)``.

        Args:
            cx: Centre column.
            cy: Centre row.
            radius: Radius in pixels; 0 draws a single pixel, negative nothing.
            value: Light (truthy) or clear (falsy).
            fill: Fill the disc instead of drawing the outline only.
        """
        if radius < 0:
            return
        # Midpoint circle: walk one octant and mirror it into the other seven.
        x, y = radius, 0
        error = 1 - radius
        while x >= y:
            if fill:
                for row, half in ((cy + y, x), (cy - y, x), (cy + x, y), (cy - x, y)):
                    self._span(cx - half, cx + half, row, value)
            else:
                for px, py in (
                    (x, y), (y, x), (-y, x), (-x, y), (-x, -y), (-y, -x), (y, -x), (x, -y),
                ):  # fmt: skip
                    self.pixel(cx + px, cy + py, value)
            y += 1
            if error < 0:
                error += 2 * y + 1
            else:
                x -= 1
                error += 2 * (y - x) + 1

    # -- text ----------------------------------------------------------------

    def text(self, x: int, y: int, text: str, value: int = 1, scale: int = 1) -> None:
        """Draw text with the built-in 5x7 font, top-left corner at ``(x, y)``.

        Each character advances 6 pixels and each line 8, times ``scale``. A
        newline starts a new line back at ``x``. Characters the font does not
        cover are drawn as a hollow box. Only the glyphs' lit pixels are drawn,
        so text can be laid over an image.

        Args:
            x: Left edge of the first character.
            y: Top edge of the first line.
            text: What to write; see :func:`text_size` to measure it first.
            value: Light (truthy) or clear (falsy) the glyph pixels.
            scale: Integer magnification; 2 gives 10x14 glyphs.

        Raises:
            ValueError: If ``scale`` is below 1.
        """
        _check_scale(scale)
        for line_index, line in enumerate(text.split("\n")):
            top = y + line_index * font.CELL_HEIGHT * scale
            for char_index, char in enumerate(line):
                left = x + char_index * font.CELL_WIDTH * scale
                self._glyph(left, top, font.glyph(char), value, scale)

    def _glyph(self, left: int, top: int, rows: tuple[int, ...], value: int, scale: int) -> None:
        """Draw one glyph's lit pixels, each as a ``scale`` x ``scale`` block."""
        for row_index, bits in enumerate(rows):
            for column in range(font.GLYPH_WIDTH):
                if not bits & (1 << (font.GLYPH_WIDTH - 1 - column)):
                    continue
                px, py = left + column * scale, top + row_index * scale
                for dy in range(scale):
                    for dx in range(scale):
                        self.pixel(px + dx, py + dy, value)

    # -- output --------------------------------------------------------------

    def to_ascii(self, lit: str = "#", dark: str = ".") -> str:
        """Render as ASCII art, one character per pixel, for a terminal preview."""
        return screen_module.to_ascii(self.buffer, lit=lit, dark=dark)

    def save_png(self, path: str, scale: int = 4) -> None:
        """Save as a PNG; see :func:`eilik.screen.save_png`."""
        screen_module.save_png(self.buffer, path, scale=scale)

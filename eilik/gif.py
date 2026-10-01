"""Animated GIF decoding, with no imaging dependency.

GIF is the format animations are usually found in, so it gets a decoder of its
own, built like the PNG one on the standard library alone. Frames come out
fully composited -- partial frames, transparency and the three disposal
methods applied -- as :class:`~eilik.image.GrayImage` luminance, each with how
long it stays on screen.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .errors import ImageError
from .image import MAX_PIXELS, GrayImage

__all__ = ["MINIMUM_DELAY", "GifFrame", "decode_gif", "load_gif"]

#: Shortest delay honoured, in seconds. Browsers stretch delays of 0 and 0.01 s
#: to about 0.1 s, and files are authored against that behaviour, so do the
#: same rather than play them at an unwatchable speed.
MINIMUM_DELAY = 0.02
_DEFAULT_DELAY = 0.1

#: Most frames accepted from one file, to bound decoding time and memory.
MAX_FRAMES = 10_000

# Row order of an interlaced image: (first row, step) for each of the 4 passes.
_INTERLACE_PASSES = ((0, 8), (4, 8), (2, 4), (1, 2))


@dataclass(frozen=True)
class GifFrame:
    """One composited frame of an animation.

    Attributes:
        image: The whole logical screen as it looks during this frame.
        delay: Seconds the frame stays on screen.
    """

    image: GrayImage
    delay: float


def load_gif(path: str | Path, background: int = 0) -> list[GifFrame]:
    """Read an animated (or still) GIF file.

    Args:
        path: The file to read.
        background: Luminance shown wherever no frame has drawn, and where
            frames are transparent. Black (unlit) by default.

    Raises:
        ImageError: If the file is not a GIF this module can decode.
        OSError: If the file cannot be read.
    """
    return decode_gif(Path(path).read_bytes(), background=background)


def decode_gif(data: bytes, background: int = 0) -> list[GifFrame]:
    """Decode GIF bytes into composited frames; see :func:`load_gif`."""
    if not 0 <= background <= 255:
        raise ValueError(f"background must be in 0..255, got {background}")
    if data[:6] not in (b"GIF87a", b"GIF89a"):
        raise ImageError("not a GIF file (bad signature)")
    if len(data) < 13:
        raise ImageError("truncated GIF: no logical screen descriptor")

    width = int.from_bytes(data[6:8], "little")
    height = int.from_bytes(data[8:10], "little")
    packed = data[10]
    if width < 1 or height < 1:
        raise ImageError(f"GIF dimensions must be positive, got {width}x{height}")
    if width * height > MAX_PIXELS:
        raise ImageError(f"{width}x{height} is larger than the {MAX_PIXELS}-pixel limit")

    position = 13
    global_palette = None
    if packed & 0x80:
        size = 3 * (2 << (packed & 0x07))
        global_palette = _luminances(data[position : position + size])
        position += size

    # None marks a pixel no frame has covered yet: it shows the background.
    screen: list[int | None] = [None] * (width * height)
    frames: list[GifFrame] = []
    control = _GraphicControl()

    while True:
        if position >= len(data):
            break  # a missing trailer is common enough to forgive
        introducer = data[position]
        position += 1
        if introducer == 0x3B:  # trailer
            break
        if introducer == 0x21:  # extension
            label = data[position] if position < len(data) else 0
            blocks, position = _sub_blocks(data, position + 1)
            if label == 0xF9 and blocks and len(blocks[0]) >= 4:
                control = _GraphicControl.parse(blocks[0])
            continue
        if introducer != 0x2C:
            raise ImageError(f"corrupt GIF: unexpected block 0x{introducer:02X}")

        image, position = _read_image(data, position, global_palette)
        if len(frames) >= MAX_FRAMES:
            raise ImageError(f"GIF has more than {MAX_FRAMES} frames")
        saved = list(screen) if control.disposal == 3 else None
        _draw(screen, width, height, image, control.transparent)
        frames.append(
            GifFrame(
                GrayImage(width, height, bytes(background if v is None else v for v in screen)),
                control.delay,
            )
        )
        _dispose(screen, width, height, image, control.disposal, saved)
        control = _GraphicControl()

    if not frames:
        raise ImageError("GIF contains no image")
    return frames


@dataclass(frozen=True)
class _GraphicControl:
    """The graphic control extension that applies to the next image."""

    disposal: int = 0
    transparent: int | None = None
    delay: float = _DEFAULT_DELAY

    @classmethod
    def parse(cls, block: bytes) -> _GraphicControl:
        flags = block[0]
        hundredths = int.from_bytes(block[1:3], "little")
        delay = hundredths / 100 if hundredths / 100 >= MINIMUM_DELAY else _DEFAULT_DELAY
        transparent = block[3] if flags & 0x01 else None
        return cls(disposal=(flags >> 2) & 0x07, transparent=transparent, delay=delay)


@dataclass(frozen=True)
class _Image:
    """One decoded image block: a rectangle of palette luminances."""

    left: int
    top: int
    width: int
    height: int
    indices: bytes
    palette: list[int]


def _luminances(table: bytes) -> list[int]:
    """Convert an RGB colour table to Rec. 601 luma, one value per entry."""
    if len(table) % 3:
        raise ImageError("truncated GIF colour table")
    return [
        (299 * table[i] + 587 * table[i + 1] + 114 * table[i + 2] + 500) // 1000
        for i in range(0, len(table), 3)
    ]


def _sub_blocks(data: bytes, position: int) -> tuple[list[bytes], int]:
    """Read a run of length-prefixed sub-blocks ending with an empty one."""
    blocks = []
    while True:
        if position >= len(data):
            raise ImageError("truncated GIF: a data block is cut short")
        size = data[position]
        position += 1
        if size == 0:
            return blocks, position
        if position + size > len(data):
            raise ImageError("truncated GIF: a data block is cut short")
        blocks.append(data[position : position + size])
        position += size


def _read_image(data: bytes, position: int, global_palette: list[int] | None) -> tuple[_Image, int]:
    """Read an image descriptor and its compressed pixels."""
    if position + 10 > len(data):
        raise ImageError("truncated GIF: an image descriptor is cut short")
    left, top, width, height = (
        int.from_bytes(data[position + offset : position + offset + 2], "little")
        for offset in (0, 2, 4, 6)
    )
    packed = data[position + 8]
    position += 9

    palette = global_palette
    if packed & 0x80:
        size = 3 * (2 << (packed & 0x07))
        palette = _luminances(data[position : position + size])
        position += size
    if palette is None:
        raise ImageError("GIF image has no colour table")

    minimum_code_size = data[position]
    if not 2 <= minimum_code_size <= 8:
        raise ImageError(f"invalid GIF LZW code size {minimum_code_size}")
    blocks, position = _sub_blocks(data, position + 1)
    indices = _lzw_decode(b"".join(blocks), minimum_code_size, width * height)
    if packed & 0x40:
        indices = _deinterlace(indices, width, height)
    return _Image(left, top, width, height, indices, palette), position


def _lzw_decode(stream: bytes, minimum_code_size: int, count: int) -> bytes:
    """Decompress GIF's variable-width LZW into exactly ``count`` indices."""
    clear = 1 << minimum_code_size
    end = clear + 1
    out = bytearray()

    def reset() -> tuple[list[bytes], int]:
        return [bytes([i]) for i in range(clear)] + [b"", b""], minimum_code_size + 1

    table, width = reset()
    previous: bytes | None = None
    bits = bit_count = 0
    for byte in stream:
        bits |= byte << bit_count
        bit_count += 8
        while bit_count >= width:
            code = bits & ((1 << width) - 1)
            bits >>= width
            bit_count -= width

            if code == clear:
                table, width = reset()
                previous = None
                continue
            if code == end:
                return _fit_length(out, count)
            if previous is None:
                if code >= len(table):
                    raise ImageError("corrupt GIF image data")
                entry = table[code]
            elif code < len(table):
                entry = table[code]
                table.append(previous + entry[:1])
            elif code == len(table):
                entry = previous + previous[:1]
                table.append(entry)
            else:
                raise ImageError("corrupt GIF image data")
            out.extend(entry)
            previous = entry
            if len(table) == 1 << width and width < 12:
                width += 1
            if len(out) >= count:
                return _fit_length(out, count)
    return _fit_length(out, count)


def _fit_length(indices: bytearray, count: int) -> bytes:
    """Trim decoded indices to the image size; pad a short stream with index 0."""
    if len(indices) < count:
        indices.extend(bytes(count - len(indices)))
    return bytes(indices[:count])


def _deinterlace(indices: bytes, width: int, height: int) -> bytes:
    """Put the rows of an interlaced image back in top-to-bottom order."""
    out = bytearray(len(indices))
    source = 0
    for first, step in _INTERLACE_PASSES:
        for row in range(first, height, step):
            out[row * width : (row + 1) * width] = indices[source : source + width]
            source += width
    return bytes(out)


def _draw(
    screen: list[int | None], width: int, height: int, image: _Image, transparent: int | None
) -> None:
    """Paint an image onto the logical screen, clipped to it."""
    palette, indices = image.palette, image.indices
    for row in range(image.height):
        y = image.top + row
        if y >= height:
            break
        base = row * image.width
        for column in range(image.width):
            x = image.left + column
            if x >= width:
                break
            index = indices[base + column]
            if index != transparent and index < len(palette):
                screen[y * width + x] = palette[index]


def _dispose(
    screen: list[int | None],
    width: int,
    height: int,
    image: _Image,
    disposal: int,
    saved: list[int | None] | None,
) -> None:
    """Apply the image's disposal method before the next frame is drawn."""
    if disposal == 2:  # restore to background: clear the rectangle
        for y in range(image.top, min(height, image.top + image.height)):
            for x in range(image.left, min(width, image.left + image.width)):
                screen[y * width + x] = None
    elif disposal == 3 and saved is not None:  # restore to previous
        screen[:] = saved

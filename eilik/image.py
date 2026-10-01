"""Turning pictures into framebuffers, with no imaging dependency.

PNG decoding is implemented here with :mod:`zlib` and :mod:`struct`, like the
PNG export in :mod:`eilik.screen`, so showing a picture on the robot needs
nothing beyond the standard library::

    framebuffer = png_to_framebuffer("cat.png", dither=True)
    robot.write_screen(framebuffer)

Every PNG colour type and bit depth is read; interlaced files are not. The
picture is reduced to 8-bit luminance (a :class:`GrayImage`), fitted into
128x64, then reduced to one bit per pixel by a threshold or by Floyd-Steinberg
dithering, which renders photographs far better on a monochrome panel.

Pixels coming from elsewhere -- Pillow, a camera, a numpy array -- go through
the same path by building a :class:`GrayImage` directly, e.g.
``GrayImage(im.width, im.height, im.convert("L").tobytes())`` for Pillow.
"""

from __future__ import annotations

import math
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path

from .errors import ImageError
from .screen import HEIGHT, WIDTH, blank

__all__ = [
    "FIT_MODES",
    "MAX_PIXELS",
    "GrayImage",
    "decode_png",
    "load_png",
    "png_to_framebuffer",
    "to_framebuffer",
]

#: How a picture is fitted into the 128x64 screen:
#:
#: * ``contain`` scales it to fit entirely, leaving margins (letterboxing);
#: * ``cover`` scales it to fill the screen, cropping what overflows;
#: * ``stretch`` scales each axis independently, distorting the picture.
FIT_MODES = ("contain", "cover", "stretch")

#: Largest picture accepted, in pixels. The cap keeps a hostile or mistaken
#: file (a "decompression bomb") from exhausting memory, since a few kilobytes
#: of compressed PNG can claim gigabytes once inflated.
MAX_PIXELS = 4096 * 4096

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"

#: Samples per pixel for each PNG colour type.
_CHANNELS = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}

#: Bit depths the PNG specification allows for each colour type.
_ALLOWED_DEPTHS = {
    0: (1, 2, 4, 8, 16),
    2: (8, 16),
    3: (1, 2, 4, 8),
    4: (8, 16),
    6: (8, 16),
}


@dataclass(frozen=True)
class GrayImage:
    """An 8-bit greyscale picture of any size.

    Attributes:
        width: Width in pixels.
        height: Height in pixels.
        pixels: ``width * height`` luminance bytes, row by row from the top;
            0 is black (an unlit pixel), 255 white.
    """

    width: int
    height: int
    pixels: bytes

    def __post_init__(self) -> None:
        """Validate the dimensions against the pixel data.

        Raises:
            ImageError: If a dimension is not positive or the pixel count does
                not match.
        """
        if self.width < 1 or self.height < 1:
            raise ImageError(f"image dimensions must be positive, got {self.width}x{self.height}")
        if len(self.pixels) != self.width * self.height:
            raise ImageError(
                f"a {self.width}x{self.height} image needs {self.width * self.height} pixel "
                f"bytes, got {len(self.pixels)}"
            )

    def pixel(self, x: int, y: int) -> int:
        """Return the luminance at ``(x, y)``."""
        if not (0 <= x < self.width and 0 <= y < self.height):
            raise IndexError(f"({x}, {y}) is outside the {self.width}x{self.height} image")
        return self.pixels[y * self.width + x]

    def inverted(self) -> GrayImage:
        """Return the negative of this image."""
        return GrayImage(self.width, self.height, bytes(255 - value for value in self.pixels))


# -- PNG decoding ------------------------------------------------------------


def load_png(path: str | Path, background: int = 0) -> GrayImage:
    """Read a PNG file into a :class:`GrayImage`.

    Args:
        path: The file to read.
        background: Luminance that transparent areas are composited onto.
            Black (0) by default, because a dark pixel is an unlit one.

    Raises:
        ImageError: If the file is not a PNG this module can decode.
        OSError: If the file cannot be read.
    """
    return decode_png(Path(path).read_bytes(), background=background)


def decode_png(data: bytes, background: int = 0) -> GrayImage:
    """Decode PNG bytes into a :class:`GrayImage`.

    Args:
        data: The complete contents of a PNG file.
        background: Luminance that transparent areas are composited onto.

    Raises:
        ImageError: If the data is not a valid, non-interlaced PNG within
            :data:`MAX_PIXELS`.
    """
    if not 0 <= background <= 255:
        raise ValueError(f"background must be in 0..255, got {background}")
    if not data.startswith(PNG_SIGNATURE):
        raise ImageError("not a PNG file (bad signature)")

    header, palette, transparency, compressed = _read_chunks(data)
    width, height, depth, color_type = _check_header(header)
    if color_type == 3 and palette is None:
        raise ImageError("palette image without a PLTE chunk")

    channels = _CHANNELS[color_type]
    bits_per_pixel = channels * depth
    stride = (width * bits_per_pixel + 7) // 8
    raw = _inflate(compressed, height * (stride + 1))

    rows = _unfilter(raw, height, stride, max(1, bits_per_pixel // 8))
    pixels = bytearray()
    for row in rows:
        samples = _samples(row, width * channels, depth)
        pixels.extend(_luminance(samples, color_type, depth, palette, transparency, background))
    return GrayImage(width, height, bytes(pixels))


def _read_chunks(data: bytes) -> tuple[bytes, bytes | None, bytes | None, bytes]:
    """Walk the chunk list, checking CRCs, and return the chunks needed."""
    header = palette = transparency = None
    compressed = bytearray()
    position = len(PNG_SIGNATURE)
    while True:
        if position + 8 > len(data):
            raise ImageError("truncated PNG: the file ends before its IEND chunk")
        length, tag = struct.unpack(">I4s", data[position : position + 8])
        body = data[position + 8 : position + 8 + length]
        crc = data[position + 8 + length : position + 12 + length]
        if len(body) != length or len(crc) != 4:
            raise ImageError(f"truncated PNG: the {tag!r} chunk is cut short")
        if zlib.crc32(tag + body) & 0xFFFFFFFF != struct.unpack(">I", crc)[0]:
            raise ImageError(f"corrupt PNG: bad CRC in the {tag.decode('latin-1')} chunk")
        position += 12 + length

        if tag == b"IHDR":
            header = body
        elif tag == b"PLTE":
            palette = body
        elif tag == b"tRNS":
            transparency = body
        elif tag == b"IDAT":
            compressed.extend(body)
        elif tag == b"IEND":
            break

    if header is None:
        raise ImageError("PNG has no IHDR chunk")
    return header, palette, transparency, bytes(compressed)


def _check_header(header: bytes) -> tuple[int, int, int, int]:
    """Validate IHDR and return ``(width, height, bit depth, colour type)``."""
    if len(header) != 13:
        raise ImageError(f"IHDR chunk is {len(header)} bytes, expected 13")
    width, height, depth, color_type, compression, filtering, interlace = struct.unpack(
        ">IIBBBBB", header
    )
    if width < 1 or height < 1:
        raise ImageError(f"PNG dimensions must be positive, got {width}x{height}")
    if width * height > MAX_PIXELS:
        raise ImageError(
            f"{width}x{height} is larger than the {MAX_PIXELS}-pixel limit; scale it down first"
        )
    if color_type not in _ALLOWED_DEPTHS:
        raise ImageError(f"unknown PNG colour type {color_type}")
    if depth not in _ALLOWED_DEPTHS[color_type]:
        raise ImageError(f"bit depth {depth} is not valid for PNG colour type {color_type}")
    if compression != 0 or filtering != 0:
        raise ImageError("unknown PNG compression or filter method")
    if interlace != 0:
        raise ImageError("interlaced PNGs are not supported; save the image without interlacing")
    return width, height, depth, color_type


def _inflate(compressed: bytes, expected: int) -> bytes:
    """Decompress the image data, refusing to produce more than ``expected`` bytes."""
    decompressor = zlib.decompressobj()
    try:
        raw = decompressor.decompress(compressed, expected)
    except zlib.error as exc:
        raise ImageError(f"corrupt PNG image data: {exc}") from None
    if len(raw) < expected:
        raise ImageError(f"truncated PNG image data: {len(raw)} of {expected} bytes")
    return raw


def _unfilter(raw: bytes, height: int, stride: int, bpp: int) -> list[bytearray]:
    """Undo the per-scanline PNG filters.

    Args:
        raw: Inflated image data: per row, one filter-type byte then the row.
        height: Number of rows.
        stride: Bytes per row, excluding the filter-type byte.
        bpp: Bytes per complete pixel, rounded up to 1.
    """
    rows = []
    previous = bytearray(stride)
    position = 0
    for _ in range(height):
        kind = raw[position]
        line = bytearray(raw[position + 1 : position + 1 + stride])
        position += stride + 1

        if kind == 1:  # Sub
            for i in range(bpp, stride):
                line[i] = (line[i] + line[i - bpp]) & 0xFF
        elif kind == 2:  # Up
            line = bytearray((a + b) & 0xFF for a, b in zip(line, previous, strict=True))
        elif kind == 3:  # Average
            for i in range(stride):
                left = line[i - bpp] if i >= bpp else 0
                line[i] = (line[i] + ((left + previous[i]) >> 1)) & 0xFF
        elif kind == 4:  # Paeth
            for i in range(stride):
                left = line[i - bpp] if i >= bpp else 0
                up = previous[i]
                up_left = previous[i - bpp] if i >= bpp else 0
                estimate = left + up - up_left
                distance_left = abs(estimate - left)
                distance_up = abs(estimate - up)
                distance_up_left = abs(estimate - up_left)
                if distance_left <= distance_up and distance_left <= distance_up_left:
                    predictor = left
                elif distance_up <= distance_up_left:
                    predictor = up
                else:
                    predictor = up_left
                line[i] = (line[i] + predictor) & 0xFF
        elif kind != 0:
            raise ImageError(f"unknown PNG filter type {kind}")

        rows.append(line)
        previous = line
    return rows


def _samples(row: bytes, count: int, depth: int) -> list[int]:
    """Split one unfiltered row into ``count`` samples at their native depth."""
    if depth == 8:
        return list(row[:count])
    if depth == 16:
        return [(row[i] << 8) | row[i + 1] for i in range(0, 2 * count, 2)]
    mask = (1 << depth) - 1
    shifts = range(8 - depth, -1, -depth)
    return [(byte >> shift) & mask for byte in row for shift in shifts][:count]


def _luminance(
    samples: list[int],
    color_type: int,
    depth: int,
    palette: bytes | None,
    transparency: bytes | None,
    background: int,
) -> bytes:
    """Convert one row of samples to luminance, compositing transparency."""
    maximum = (1 << depth) - 1

    def scale(value: int) -> int:
        return value * 255 // maximum

    def blend(level: int, alpha: int) -> int:
        return (level * alpha + background * (255 - alpha) + 127) // 255

    if color_type == 0:  # greyscale
        key = int.from_bytes(transparency[:2], "big") if transparency else None
        return bytes(background if value == key else scale(value) for value in samples)

    if color_type == 4:  # greyscale + alpha
        return bytes(
            blend(scale(samples[i]), scale(samples[i + 1])) for i in range(0, len(samples), 2)
        )

    if color_type == 3:  # palette
        assert palette is not None
        alphas = transparency or b""
        out = bytearray()
        for index in samples:
            if 3 * index + 2 >= len(palette):
                raise ImageError(f"palette index {index} is outside the PLTE chunk")
            level = _rgb_luminance(*palette[3 * index : 3 * index + 3])
            out.append(blend(level, alphas[index]) if index < len(alphas) else level)
        return bytes(out)

    # Truecolour, with or without alpha.
    step = _CHANNELS[color_type]
    key = (
        tuple(int.from_bytes(transparency[i : i + 2], "big") for i in (0, 2, 4))
        if color_type == 2 and transparency and len(transparency) >= 6
        else None
    )
    out = bytearray()
    for i in range(0, len(samples), step):
        red, green, blue = samples[i : i + 3]
        level = _rgb_luminance(scale(red), scale(green), scale(blue))
        if step == 4:
            level = blend(level, scale(samples[i + 3]))
        elif (red, green, blue) == key:
            level = background
        out.append(level)
    return bytes(out)


def _rgb_luminance(red: int, green: int, blue: int) -> int:
    """Return the Rec. 601 luma of an 8-bit RGB triple."""
    return (299 * red + 587 * green + 114 * blue + 500) // 1000


# -- fitting and binarising --------------------------------------------------


def _fit(image: GrayImage, fit: str, background: int) -> list[int]:
    """Resample ``image`` onto the 128x64 screen by area averaging.

    Each screen pixel takes the mean of the source pixels its footprint
    touches, which is a box filter when shrinking and close to nearest
    neighbour when enlarging. Screen pixels whose centre falls outside the
    picture get ``background``.
    """
    if fit == "stretch":
        scale_x, scale_y = WIDTH / image.width, HEIGHT / image.height
    else:
        choose = min if fit == "contain" else max
        scale_x = scale_y = choose(WIDTH / image.width, HEIGHT / image.height)
    offset_x = (WIDTH - image.width * scale_x) / 2
    offset_y = (HEIGHT - image.height * scale_y) / 2

    def span(cell: int, offset: float, scale: float, limit: int) -> range | None:
        centre = (cell + 0.5 - offset) / scale
        if not 0 <= centre < limit:
            return None
        start = max(0, math.floor((cell - offset) / scale))
        stop = min(limit, math.ceil((cell + 1 - offset) / scale))
        return range(start, max(stop, start + 1))

    columns = [span(x, offset_x, scale_x, image.width) for x in range(WIDTH)]
    out = [background] * (WIDTH * HEIGHT)
    pixels, width = image.pixels, image.width
    for y in range(HEIGHT):
        rows = span(y, offset_y, scale_y, image.height)
        if rows is None:
            continue
        for x, cols in enumerate(columns):
            if cols is None:
                continue
            total = 0
            for source_y in rows:
                base = source_y * width
                total += sum(pixels[base + cols.start : base + cols.stop])
            out[y * WIDTH + x] = total // (len(rows) * len(cols))
    return out


def to_framebuffer(
    image: GrayImage,
    *,
    fit: str = "contain",
    threshold: int = 128,
    dither: bool = False,
    invert: bool = False,
    background: int = 0,
) -> bytearray:
    """Fit a picture to the screen and reduce it to one bit per pixel.

    Args:
        image: The picture.
        fit: One of :data:`FIT_MODES`; ``contain`` by default, so nothing is
            cropped.
        threshold: Luminance from which a pixel is lit, in 0..256.
        dither: Use Floyd-Steinberg error diffusion instead of a plain
            threshold. Much better for photographs and gradients; a plain
            threshold suits line art and text.
        invert: Light the dark parts of the picture instead of the bright
            ones. Margins left by ``contain`` stay ``background``.
        background: Luminance of the margins, 0 (unlit) by default.

    Returns:
        A logical 1024-byte framebuffer, ready for
        :meth:`~eilik.robot.Eilik.write_screen` or :class:`~eilik.canvas.Canvas`.

    Raises:
        ValueError: If ``fit``, ``threshold`` or ``background`` is out of range.
    """
    if fit not in FIT_MODES:
        raise ValueError(f"fit must be one of {', '.join(FIT_MODES)}; got {fit!r}")
    if not 0 <= threshold <= 256:
        raise ValueError(f"threshold must be in 0..256, got {threshold}")
    if not 0 <= background <= 255:
        raise ValueError(f"background must be in 0..255, got {background}")

    levels = _fit(image.inverted() if invert else image, fit, background)
    framebuffer = blank()
    if dither:
        _diffuse(levels, threshold, framebuffer)
        return framebuffer
    for index, level in enumerate(levels):
        if level >= threshold:
            y, x = divmod(index, WIDTH)
            framebuffer[(y >> 3) * WIDTH + x] |= 1 << (y & 7)
    return framebuffer


def _diffuse(levels: list[int], threshold: int, framebuffer: bytearray) -> None:
    """Floyd-Steinberg dithering of ``levels`` into ``framebuffer``, in place."""
    errors = [float(level) for level in levels]
    for y in range(HEIGHT):
        for x in range(WIDTH):
            index = y * WIDTH + x
            old = errors[index]
            new = 255.0 if old >= threshold else 0.0
            if new:
                framebuffer[(y >> 3) * WIDTH + x] |= 1 << (y & 7)
            error = old - new
            if x + 1 < WIDTH:
                errors[index + 1] += error * 7 / 16
            if y + 1 < HEIGHT:
                below = index + WIDTH
                if x > 0:
                    errors[below - 1] += error * 3 / 16
                errors[below] += error * 5 / 16
                if x + 1 < WIDTH:
                    errors[below + 1] += error * 1 / 16


def png_to_framebuffer(
    path: str | Path,
    *,
    fit: str = "contain",
    threshold: int = 128,
    dither: bool = False,
    invert: bool = False,
    background: int = 0,
) -> bytearray:
    """Load a PNG and turn it into a framebuffer in one step.

    ``background`` serves both for transparent areas and for margins; see
    :func:`load_png` and :func:`to_framebuffer` for the other arguments.

    Raises:
        ImageError: If the file is not a PNG this module can decode.
        OSError: If the file cannot be read.
    """
    return to_framebuffer(
        load_png(path, background=background),
        fit=fit,
        threshold=threshold,
        dither=dither,
        invert=invert,
        background=background,
    )

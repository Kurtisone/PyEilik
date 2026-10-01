"""PNG decoding, fitting to the screen, thresholding and dithering."""

from __future__ import annotations

import io
import random
import struct
import zlib

import pytest

from eilik.canvas import Canvas
from eilik.errors import ImageError
from eilik.image import (
    MAX_PIXELS,
    GrayImage,
    decode_png,
    load_png,
    png_to_framebuffer,
    to_framebuffer,
)
from eilik.screen import HEIGHT, WIDTH, get_pixel, save_png

# -- a deliberately independent PNG writer, for building test inputs ---------


def _chunk(tag: bytes, body: bytes) -> bytes:
    return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", zlib.crc32(tag + body))


def _pack(samples: list[int], depth: int) -> bytes:
    if depth == 8:
        return bytes(samples)
    if depth == 16:
        return b"".join(value.to_bytes(2, "big") for value in samples)
    per_byte = 8 // depth
    out = bytearray()
    for start in range(0, len(samples), per_byte):
        byte = 0
        group = samples[start : start + per_byte]
        for position, value in enumerate(group):
            byte |= value << (8 - depth * (position + 1))
        out.append(byte)
    return bytes(out)


def _paeth(left: int, up: int, up_left: int) -> int:
    estimate = left + up - up_left
    candidates = [(abs(estimate - left), 0, left), (abs(estimate - up), 1, up)]
    candidates.append((abs(estimate - up_left), 2, up_left))
    return min(candidates)[2]


def _filter(kind: int, line: bytes, prior: bytes, bpp: int) -> bytes:
    out = bytearray()
    for i, value in enumerate(line):
        left = line[i - bpp] if i >= bpp else 0
        up = prior[i]
        up_left = prior[i - bpp] if i >= bpp else 0
        predictor = [0, left, up, (left + up) >> 1, _paeth(left, up, up_left)][kind]
        out.append((value - predictor) & 0xFF)
    return bytes([kind]) + bytes(out)


CHANNELS = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}


def make_png(
    width: int,
    height: int,
    color_type: int,
    depth: int,
    rows: list[list[int]],
    *,
    filters: tuple[int, ...] = (0,),
    palette: bytes | None = None,
    trns: bytes | None = None,
    interlace: int = 0,
) -> bytes:
    """Encode ``rows`` of native-depth samples, cycling through ``filters``."""
    bpp = max(1, CHANNELS[color_type] * depth // 8)
    raw = bytearray()
    prior = None
    for index, samples in enumerate(rows):
        line = _pack(samples, depth)
        prior = prior or bytes(len(line))
        raw.extend(_filter(filters[index % len(filters)], line, prior, bpp))
        prior = line
    header = struct.pack(">IIBBBBB", width, height, depth, color_type, 0, 0, interlace)
    png = b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", header)
    if palette is not None:
        png += _chunk(b"PLTE", palette)
    if trns is not None:
        png += _chunk(b"tRNS", trns)
    return png + _chunk(b"IDAT", zlib.compress(bytes(raw))) + _chunk(b"IEND", b"")


def luma(red: int, green: int, blue: int) -> int:
    return (299 * red + 587 * green + 114 * blue + 500) // 1000


def flat(width: int, height: int, level: int) -> GrayImage:
    return GrayImage(width, height, bytes([level]) * (width * height))


def lit_columns(framebuffer: bytes, y: int = 32) -> list[int]:
    return [x for x in range(WIDTH) if get_pixel(framebuffer, x, y)]


# -- decoding ------------------------------------------------------------------


class TestDecode:
    @pytest.mark.parametrize("kind", [0, 1, 2, 3, 4], ids=["none", "sub", "up", "avg", "paeth"])
    def test_every_filter_type_on_rgb(self, kind):
        rng = random.Random(kind)
        rows = [[rng.randrange(256) for _ in range(13 * 3)] for _ in range(7)]
        image = decode_png(make_png(13, 7, 2, 8, rows, filters=(kind,)))
        expected = [luma(*row[i : i + 3]) for row in rows for i in range(0, len(row), 3)]
        assert list(image.pixels) == expected

    def test_mixed_filters_on_one_bit_rows(self):
        """Sub-byte pixels filter with a one-byte stride, a classic pitfall."""
        rng = random.Random(9)
        rows = [[rng.randrange(2) for _ in range(19)] for _ in range(10)]
        image = decode_png(make_png(19, 10, 0, 1, rows, filters=(0, 1, 2, 3, 4)))
        assert list(image.pixels) == [255 * value for row in rows for value in row]

    @pytest.mark.parametrize("depth", [1, 2, 4, 8, 16])
    def test_greyscale_depths_scale_to_full_range(self, depth):
        maximum = (1 << depth) - 1
        rows = [[0, maximum, maximum // 2, 1]]
        image = decode_png(make_png(4, 1, 0, depth, rows, filters=(4,)))
        assert list(image.pixels) == [value * 255 // maximum for value in rows[0]]

    def test_sixteen_bit_rgba(self):
        rows = [[65535, 0, 0, 65535, 0, 65535, 0, 0]]
        image = decode_png(make_png(2, 1, 6, 16, rows, filters=(1,)))
        assert list(image.pixels) == [luma(255, 0, 0), 0]

    def test_grey_alpha_is_composited_onto_the_background(self):
        rows = [[255, 255, 255, 0, 200, 128]]

        def over(background: int) -> int:
            return (200 * 128 + background * (255 - 128) + 127) // 255

        assert list(decode_png(make_png(3, 1, 4, 8, rows)).pixels) == [255, 0, over(0)]
        image = decode_png(make_png(3, 1, 4, 8, rows), background=255)
        assert list(image.pixels) == [255, 255, over(255)]

    @pytest.mark.parametrize("depth", [1, 2, 4, 8])
    def test_palette_with_per_entry_alpha(self, depth):
        palette = bytes([255, 255, 255, 255, 0, 0])
        rows = [[0, 1, 0]]
        trns = bytes([128])  # entry 0 half transparent, entry 1 opaque
        image = decode_png(make_png(3, 1, 3, depth, rows, palette=palette, trns=trns))
        half_white = (255 * 128 + 127) // 255
        assert list(image.pixels) == [half_white, luma(255, 0, 0), half_white]

    def test_greyscale_colour_key(self):
        rows = [[7, 8, 7]]
        image = decode_png(make_png(3, 1, 0, 8, rows, trns=(7).to_bytes(2, "big")), background=99)
        assert list(image.pixels) == [99, 8, 99]

    def test_truecolour_colour_key(self):
        rows = [[10, 20, 30, 10, 20, 31]]
        trns = b"".join(value.to_bytes(2, "big") for value in (10, 20, 30))
        image = decode_png(make_png(2, 1, 2, 8, rows, trns=trns))
        assert list(image.pixels) == [0, luma(10, 20, 31)]

    def test_load_png_reads_a_file(self, tmp_path):
        path = tmp_path / "pixel.png"
        path.write_bytes(make_png(1, 1, 0, 8, [[42]]))
        assert load_png(path).pixels == b"\x2a"


class TestDecodeErrors:
    def good(self) -> bytes:
        return make_png(2, 2, 0, 8, [[1, 2], [3, 4]])

    def test_not_a_png(self):
        with pytest.raises(ImageError, match="not a PNG"):
            decode_png(b"GIF89a....")

    def test_bad_crc(self):
        data = bytearray(self.good())
        data[20] ^= 0xFF  # inside the IHDR body
        with pytest.raises(ImageError, match="bad CRC in the IHDR"):
            decode_png(bytes(data))

    def test_truncated_file(self):
        with pytest.raises(ImageError, match="truncated"):
            decode_png(self.good()[:-15])

    def test_no_header(self):
        data = b"\x89PNG\r\n\x1a\n" + _chunk(b"IEND", b"")
        with pytest.raises(ImageError, match="no IHDR"):
            decode_png(data)

    def test_interlaced(self):
        with pytest.raises(ImageError, match="interlaced"):
            decode_png(make_png(2, 2, 0, 8, [[1, 2], [3, 4]], interlace=1))

    def test_invalid_depth_for_colour_type(self):
        with pytest.raises(ImageError, match="bit depth 4 is not valid"):
            decode_png(make_png(2, 1, 2, 4, [[0] * 6]))

    def test_oversized_image_is_refused_before_inflating(self):
        side = int(MAX_PIXELS**0.5) + 1
        header = struct.pack(">IIBBBBB", side, side, 8, 0, 0, 0, 0)
        data = b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", header) + _chunk(b"IEND", b"")
        with pytest.raises(ImageError, match="larger than"):
            decode_png(data)

    def test_unknown_filter_type(self):
        header = struct.pack(">IIBBBBB", 1, 1, 8, 0, 0, 0, 0)
        data = (
            b"\x89PNG\r\n\x1a\n"
            + _chunk(b"IHDR", header)
            + _chunk(b"IDAT", zlib.compress(b"\x07\x00"))
            + _chunk(b"IEND", b"")
        )
        with pytest.raises(ImageError, match="filter type 7"):
            decode_png(data)

    def test_truncated_image_data(self):
        header = struct.pack(">IIBBBBB", 4, 4, 8, 0, 0, 0, 0)
        data = (
            b"\x89PNG\r\n\x1a\n"
            + _chunk(b"IHDR", header)
            + _chunk(b"IDAT", zlib.compress(b"\x00" * 6))
            + _chunk(b"IEND", b"")
        )
        with pytest.raises(ImageError, match="truncated PNG image data"):
            decode_png(data)

    def test_corrupt_compressed_data(self):
        header = struct.pack(">IIBBBBB", 1, 1, 8, 0, 0, 0, 0)
        data = (
            b"\x89PNG\r\n\x1a\n"
            + _chunk(b"IHDR", header)
            + _chunk(b"IDAT", b"not zlib at all")
            + _chunk(b"IEND", b"")
        )
        with pytest.raises(ImageError, match="corrupt PNG image data"):
            decode_png(data)

    def test_palette_image_without_palette(self):
        with pytest.raises(ImageError, match="without a PLTE"):
            decode_png(make_png(1, 1, 3, 8, [[0]]))

    def test_palette_index_out_of_range(self):
        with pytest.raises(ImageError, match="palette index 5"):
            decode_png(make_png(1, 1, 3, 8, [[5]], palette=bytes(6)))

    def test_bad_background(self):
        with pytest.raises(ValueError, match="background"):
            decode_png(self.good(), background=300)


class TestAgainstPillow:
    """Cross-check against an independent decoder, when one is installed."""

    @pytest.mark.parametrize("mode", ["1", "L", "LA", "RGB", "RGBA", "P", "PA"])
    @pytest.mark.parametrize("size", [(1, 1), (7, 3), (33, 17)])
    def test_matches_pillow(self, mode, size):
        image_module = pytest.importorskip("PIL.Image")
        rng = random.Random(f"{mode}{size}")
        width, height = size
        source = image_module.new("RGBA", size)
        source.putdata([tuple(rng.randrange(256) for _ in range(4)) for _ in range(width * height)])
        converted = source.quantize(colors=16) if mode == "PA" else source.convert(mode)

        buffer = io.BytesIO()
        converted.save(buffer, "PNG")
        ours = decode_png(buffer.getvalue())

        reference = image_module.open(io.BytesIO(buffer.getvalue())).convert("RGBA")
        expected = [
            (luma(*reference.getpixel((x, y))[:3]) * reference.getpixel((x, y))[3] + 127) // 255
            for y in range(height)
            for x in range(width)
        ]
        assert list(ours.pixels) == expected


# -- fitting -------------------------------------------------------------------


class TestFit:
    def test_screen_sized_image_maps_pixel_for_pixel(self):
        rng = random.Random(3)
        pixels = bytes(rng.choice((0, 255)) for _ in range(WIDTH * HEIGHT))
        framebuffer = to_framebuffer(GrayImage(WIDTH, HEIGHT, pixels))
        assert all(
            get_pixel(framebuffer, x, y) == (pixels[y * WIDTH + x] == 255)
            for y in range(HEIGHT)
            for x in range(WIDTH)
        )

    def test_the_top_left_corner_stays_top_left(self):
        pixels = bytearray(WIDTH * HEIGHT)
        pixels[0] = 255
        framebuffer = to_framebuffer(GrayImage(WIDTH, HEIGHT, bytes(pixels)))
        assert get_pixel(framebuffer, 0, 0) == 1
        assert sum(bin(byte).count("1") for byte in framebuffer) == 1

    def test_contain_letterboxes_a_square(self):
        framebuffer = to_framebuffer(flat(64, 64, 255))
        assert lit_columns(framebuffer, 0) == list(range(32, 96))
        assert lit_columns(framebuffer, 63) == list(range(32, 96))

    def test_cover_fills_the_screen(self):
        framebuffer = to_framebuffer(flat(64, 64, 255), fit="cover")
        assert set(framebuffer) == {0xFF}

    def test_cover_crops_the_overflow(self):
        """A tall image keeps its middle band when covering a wide screen."""
        pixels = bytearray(32 * 64)
        for y in range(28, 36):
            pixels[y * 32 : (y + 1) * 32] = b"\xff" * 32
        framebuffer = to_framebuffer(GrayImage(32, 64, bytes(pixels)), fit="cover")
        assert lit_columns(framebuffer, 32) == list(range(WIDTH))
        assert lit_columns(framebuffer, 0) == []

    def test_stretch_fills_the_screen(self):
        assert set(to_framebuffer(flat(10, 10, 255), fit="stretch")) == {0xFF}

    def test_margins_take_the_background(self):
        framebuffer = to_framebuffer(flat(64, 64, 0), background=255)
        assert lit_columns(framebuffer) == [*range(32), *range(96, 128)]

    def test_invert_leaves_the_margins_alone(self):
        framebuffer = to_framebuffer(flat(64, 64, 0), invert=True)
        assert lit_columns(framebuffer) == list(range(32, 96))

    def test_shrinking_averages(self):
        """A fine checkerboard averages to mid grey instead of aliasing."""
        pixels = bytes(255 * ((x + y) % 2) for y in range(128) for x in range(256))
        framebuffer = to_framebuffer(GrayImage(256, 128, pixels), threshold=128)
        assert set(framebuffer) == {0}  # 127 < 128: every pixel is the average
        framebuffer = to_framebuffer(GrayImage(256, 128, pixels), threshold=127)
        assert set(framebuffer) == {0xFF}

    def test_enlarging_keeps_hard_edges(self):
        pixels = bytes(255 if x < 8 else 0 for _ in range(8) for x in range(16))
        framebuffer = to_framebuffer(GrayImage(16, 8, pixels))
        assert lit_columns(framebuffer) == list(range(64))


class TestBinarise:
    @pytest.mark.parametrize(("level", "fraction"), [(128, 0.5), (64, 0.25), (192, 0.75)])
    def test_dithering_preserves_average_brightness(self, level, fraction):
        framebuffer = to_framebuffer(flat(WIDTH, HEIGHT, level), dither=True)
        lit = sum(bin(byte).count("1") for byte in framebuffer)
        assert abs(lit / (WIDTH * HEIGHT) - fraction) < 0.02

    @pytest.mark.parametrize(("level", "expected"), [(0, {0}), (255, {0xFF})])
    def test_dithering_leaves_black_and_white_alone(self, level, expected):
        assert set(to_framebuffer(flat(WIDTH, HEIGHT, level), dither=True)) == expected

    def test_threshold_is_inclusive(self):
        assert set(to_framebuffer(flat(WIDTH, HEIGHT, 100), threshold=100)) == {0xFF}
        assert set(to_framebuffer(flat(WIDTH, HEIGHT, 100), threshold=101)) == {0}

    @pytest.mark.parametrize(
        "options",
        [{"fit": "zoom"}, {"threshold": -1}, {"threshold": 300}, {"background": 256}],
    )
    def test_bad_options_are_refused(self, options):
        with pytest.raises(ValueError):
            to_framebuffer(flat(2, 2, 0), **options)


class TestGrayImage:
    def test_shape_is_validated(self):
        with pytest.raises(ImageError, match="needs 4 pixel bytes"):
            GrayImage(2, 2, b"\x00")
        with pytest.raises(ImageError, match="positive"):
            GrayImage(0, 2, b"")

    def test_pixel_and_inverted(self):
        image = GrayImage(2, 1, b"\x00\x10")
        assert image.pixel(1, 0) == 0x10
        assert image.inverted().pixels == b"\xff\xef"
        with pytest.raises(IndexError):
            image.pixel(2, 0)


class TestRoundTrip:
    @pytest.mark.parametrize("scale", [1, 4])
    def test_a_saved_screen_loads_back_identically(self, tmp_path, scale):
        canvas = Canvas()
        canvas.text(3, 3, "Eilik à l'écran", scale=1)
        canvas.circle(100, 40, 15, fill=True)
        path = tmp_path / "screen.png"
        canvas.save_png(str(path), scale=scale)
        assert png_to_framebuffer(path) == canvas.buffer

    def test_the_result_can_be_drawn_on(self, tmp_path):
        path = tmp_path / "white.png"
        path.write_bytes(make_png(1, 1, 0, 8, [[255]]))
        canvas = Canvas(png_to_framebuffer(path, fit="stretch"))
        canvas.text(0, 0, "x", value=0)
        assert canvas.get_pixel(0, 2) == 0
        assert canvas.get_pixel(127, 63) == 1


def test_save_png_itself_round_trips_through_the_decoder(tmp_path):
    """The exporter and the importer agree on orientation and scaling."""
    framebuffer = bytes(random.Random(5).randrange(256) for _ in range(1024))
    path = tmp_path / "noise.png"
    save_png(framebuffer, str(path), scale=2)
    image = load_png(path)
    assert (image.width, image.height) == (256, 128)
    assert to_framebuffer(image) == bytearray(framebuffer)

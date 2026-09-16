"""Framebuffer geometry, the 180-degree panel rotation, and PNG export."""

from __future__ import annotations

import struct
import zlib

import pytest

from eilik.errors import ProtocolError
from eilik.screen import (
    FRAMEBUFFER_SIZE,
    HEIGHT,
    PAGES,
    WIDTH,
    blank,
    from_rows,
    get_pixel,
    rotate180,
    save_png,
    set_pixel,
    to_ascii,
    to_rows,
)


class TestGeometry:
    """The buffer is eight 128-byte pages."""

    def test_dimensions(self):
        assert (WIDTH, HEIGHT, PAGES, FRAMEBUFFER_SIZE) == (128, 64, 8, 1024)

    def test_blank_is_the_right_size(self):
        assert len(blank()) == FRAMEBUFFER_SIZE
        assert set(blank()) == {0}
        assert set(blank(0xFF)) == {0xFF}

    @pytest.mark.parametrize(
        ("x", "y", "index", "bit"),
        [
            (0, 0, 0, 0),
            (0, 7, 0, 7),
            (0, 8, 128, 0),
            (127, 63, 7 * 128 + 127, 7),
            (5, 9, 128 + 5, 1),
        ],
    )
    def test_pixel_addressing_matches_the_documented_formula(self, x, y, index, bit):
        framebuffer = blank()
        framebuffer[index] = 1 << bit
        assert get_pixel(framebuffer, x, y) == 1

    def test_set_and_clear(self):
        framebuffer = blank()
        set_pixel(framebuffer, 42, 17, 1)
        assert get_pixel(framebuffer, 42, 17) == 1
        set_pixel(framebuffer, 42, 17, 0)
        assert get_pixel(framebuffer, 42, 17) == 0

    def test_setting_a_pixel_leaves_neighbours_alone(self):
        framebuffer = blank()
        set_pixel(framebuffer, 42, 17, 1)
        assert get_pixel(framebuffer, 42, 16) == 0
        assert get_pixel(framebuffer, 42, 18) == 0
        assert get_pixel(framebuffer, 41, 17) == 0

    @pytest.mark.parametrize(("x", "y"), [(-1, 0), (128, 0), (0, -1), (0, 64)])
    def test_off_screen_coordinates_raise(self, x, y):
        with pytest.raises(IndexError):
            get_pixel(blank(), x, y)
        with pytest.raises(IndexError):
            set_pixel(blank(), x, y)


class TestRotation:
    """The panel is mounted upside down, so every transfer is rotated."""

    @pytest.mark.parametrize(
        ("x", "y"), [(0, 0), (127, 63), (5, 9), (64, 32), (0, 63), (127, 0), (63, 7)]
    )
    def test_maps_to_the_opposite_corner(self, x, y):
        framebuffer = blank()
        set_pixel(framebuffer, x, y, 1)
        rotated = rotate180(bytes(framebuffer))
        assert get_pixel(rotated, WIDTH - 1 - x, HEIGHT - 1 - y) == 1

    def test_only_one_pixel_moves(self):
        framebuffer = blank()
        set_pixel(framebuffer, 5, 9, 1)
        rotated = rotate180(bytes(framebuffer))
        lit = [(x, y) for y in range(HEIGHT) for x in range(WIDTH) if get_pixel(rotated, x, y)]
        assert lit == [(122, 54)]

    def test_is_its_own_inverse(self):
        framebuffer = bytes(range(256)) * 4
        assert rotate180(rotate180(framebuffer)) == framebuffer

    def test_preserves_a_uniform_buffer(self):
        assert rotate180(bytes(blank(0xFF))) == bytes(blank(0xFF))

    def test_lit_pixel_count_is_preserved(self):
        framebuffer = bytes(range(256)) * 4
        rotated = rotate180(framebuffer)
        assert sum(bin(b).count("1") for b in rotated) == sum(
            bin(b).count("1") for b in framebuffer
        )

    @pytest.mark.parametrize("size", [0, 1023, 1025])
    def test_wrong_size_is_refused(self, size):
        with pytest.raises(ProtocolError, match="exactly 1024 bytes"):
            rotate180(bytes(size))


class TestRendering:
    """Human-readable renderings of a buffer."""

    def test_to_rows_shape(self):
        rows = to_rows(blank())
        assert len(rows) == HEIGHT
        assert all(len(row) == WIDTH for row in rows)

    def test_from_rows_roundtrip(self):
        framebuffer = blank()
        for x in range(0, WIDTH, 7):
            set_pixel(framebuffer, x, x % HEIGHT, 1)
        assert from_rows(to_rows(framebuffer)) == framebuffer

    def test_from_rows_rejects_a_bad_shape(self):
        with pytest.raises(ValueError, match="expected 64 rows"):
            from_rows([[0] * WIDTH] * 10)

    def test_ascii_art_shape(self):
        framebuffer = blank()
        set_pixel(framebuffer, 0, 0, 1)
        art = to_ascii(framebuffer).splitlines()
        assert len(art) == HEIGHT
        assert art[0][0] == "#"
        assert art[0][1] == "."


class TestPng:
    """PNG export uses only zlib and struct, so it needs no imaging library."""

    def _parse(self, data: bytes):
        assert data[:8] == b"\x89PNG\r\n\x1a\n"
        chunks, offset = {}, 8
        while offset < len(data):
            (length,) = struct.unpack(">I", data[offset : offset + 4])
            tag = data[offset + 4 : offset + 8]
            payload = data[offset + 8 : offset + 8 + length]
            (crc,) = struct.unpack(">I", data[offset + 8 + length : offset + 12 + length])
            assert crc == zlib.crc32(tag + payload) & 0xFFFFFFFF, tag
            chunks[tag] = payload
            offset += 12 + length
        return chunks

    def test_header_and_scaling(self, tmp_path):
        path = tmp_path / "screen.png"
        save_png(blank(), str(path), scale=4)
        chunks = self._parse(path.read_bytes())
        width, height, depth, colour = struct.unpack(">IIBB", chunks[b"IHDR"][:10])
        assert (width, height, depth, colour) == (WIDTH * 4, HEIGHT * 4, 8, 0)
        assert b"IEND" in chunks

    def test_pixels_survive_the_round_trip(self, tmp_path):
        framebuffer = blank()
        set_pixel(framebuffer, 3, 5, 1)
        path = tmp_path / "screen.png"
        save_png(framebuffer, str(path), scale=1)

        raw = zlib.decompress(self._parse(path.read_bytes())[b"IDAT"])
        stride = WIDTH + 1  # one filter byte per row
        assert len(raw) == stride * HEIGHT
        assert all(raw[row * stride] == 0 for row in range(HEIGHT))  # filter type None
        assert raw[5 * stride + 1 + 3] == 0xFF
        assert raw[5 * stride + 1 + 4] == 0x00

    def test_scale_must_be_positive(self, tmp_path):
        with pytest.raises(ValueError, match="scale must be"):
            save_png(blank(), str(tmp_path / "x.png"), scale=0)

    def test_wrong_buffer_size_is_refused(self, tmp_path):
        with pytest.raises(ProtocolError):
            save_png(bytes(10), str(tmp_path / "x.png"))

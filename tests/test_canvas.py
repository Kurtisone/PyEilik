"""Drawing primitives, the built-in font and text layout."""

from __future__ import annotations

import pytest

from eilik import font
from eilik.canvas import Canvas, text_size
from eilik.errors import ProtocolError
from eilik.screen import FRAMEBUFFER_SIZE, blank, rotate180, set_pixel


def lit(canvas: Canvas) -> set[tuple[int, int]]:
    """Return the coordinates of every lit pixel."""
    return {
        (x, y) for y in range(canvas.height) for x in range(canvas.width) if canvas.get_pixel(x, y)
    }


class TestConstruction:
    def test_starts_blank(self):
        assert bytes(Canvas()) == bytes(FRAMEBUFFER_SIZE)

    def test_wraps_a_copy_of_an_existing_framebuffer(self):
        source = blank()
        set_pixel(source, 3, 4)
        canvas = Canvas(source)
        canvas.clear()
        assert canvas.get_pixel(3, 4) == 0
        assert source != blank()

    def test_wrong_size_is_refused(self):
        with pytest.raises(ProtocolError, match="exactly 1024 bytes"):
            Canvas(bytes(10))

    def test_copy_is_independent(self):
        canvas = Canvas()
        clone = canvas.copy()
        clone.pixel(0, 0)
        assert canvas != clone
        assert canvas == Canvas()

    def test_repr_counts_lit_pixels(self):
        canvas = Canvas()
        canvas.rect(0, 0, 2, 2, fill=True)
        assert "4 lit" in repr(canvas)


class TestWholeScreen:
    def test_clear_to_lit(self):
        canvas = Canvas()
        canvas.clear(1)
        assert set(bytes(canvas)) == {0xFF}

    def test_invert_twice_is_identity(self):
        canvas = Canvas()
        canvas.text(0, 0, "Eilik")
        before = bytes(canvas)
        canvas.invert()
        assert bytes(canvas) != before
        canvas.invert()
        assert bytes(canvas) == before


class TestPixelsAndLines:
    def test_off_screen_pixels_are_ignored(self):
        canvas = Canvas()
        for x, y in [(-1, 0), (0, -1), (128, 0), (0, 64), (500, 500)]:
            canvas.pixel(x, y)
        assert lit(canvas) == set()

    def test_value_zero_clears(self):
        canvas = Canvas()
        canvas.pixel(5, 9)
        canvas.pixel(5, 9, 0)
        assert lit(canvas) == set()

    def test_horizontal_line_includes_both_ends(self):
        canvas = Canvas()
        canvas.line(2, 5, 6, 5)
        assert lit(canvas) == {(x, 5) for x in range(2, 7)}

    def test_vertical_line(self):
        canvas = Canvas()
        canvas.line(7, 10, 7, 3)
        assert lit(canvas) == {(7, y) for y in range(3, 11)}

    def test_diagonal_line(self):
        canvas = Canvas()
        canvas.line(0, 0, 9, 9)
        assert lit(canvas) == {(i, i) for i in range(10)}

    @pytest.mark.parametrize(("x0", "y0", "x1", "y1"), [(0, 0, 40, 13), (3, 50, 9, 2)])
    def test_direction_does_not_matter_for_the_endpoints(self, x0, y0, x1, y1):
        forward, backward = Canvas(), Canvas()
        forward.line(x0, y0, x1, y1)
        backward.line(x1, y1, x0, y0)
        for canvas in (forward, backward):
            assert {(x0, y0), (x1, y1)} <= lit(canvas)
        assert len(lit(forward)) == len(lit(backward)) == max(abs(x1 - x0), abs(y1 - y0)) + 1

    def test_line_is_clipped(self):
        canvas = Canvas()
        canvas.line(-10, 5, 200, 5)
        assert lit(canvas) == {(x, 5) for x in range(128)}


class TestRectangles:
    def test_outline(self):
        canvas = Canvas()
        canvas.rect(10, 10, 5, 4)
        pixels = lit(canvas)
        assert len(pixels) == 2 * 5 + 2 * 4 - 4
        assert (12, 11) not in pixels
        assert {(10, 10), (14, 10), (10, 13), (14, 13)} <= pixels

    def test_filled(self):
        canvas = Canvas()
        canvas.rect(10, 10, 5, 4, fill=True)
        assert lit(canvas) == {(x, y) for x in range(10, 15) for y in range(10, 14)}

    def test_full_screen_border(self):
        canvas = Canvas()
        canvas.rect(0, 0, 128, 64)
        assert len(lit(canvas)) == 2 * 128 + 2 * 64 - 4

    def test_clipped(self):
        canvas = Canvas()
        canvas.rect(-3, -3, 6, 6, fill=True)
        assert lit(canvas) == {(x, y) for x in range(3) for y in range(3)}

    @pytest.mark.parametrize(("width", "height"), [(0, 5), (5, 0), (-2, 4)])
    def test_empty_rectangles_draw_nothing(self, width, height):
        canvas = Canvas()
        canvas.rect(5, 5, width, height, fill=True)
        assert lit(canvas) == set()

    def test_one_pixel_rectangle(self):
        canvas = Canvas()
        canvas.rect(4, 4, 1, 1)
        assert lit(canvas) == {(4, 4)}


class TestCircles:
    def test_radius_zero_is_one_pixel(self):
        canvas = Canvas()
        canvas.circle(20, 20, 0)
        assert lit(canvas) == {(20, 20)}

    def test_negative_radius_draws_nothing(self):
        canvas = Canvas()
        canvas.circle(20, 20, -1)
        assert lit(canvas) == set()

    def test_outline_reaches_the_four_extremes(self):
        canvas = Canvas()
        canvas.circle(40, 30, 10)
        pixels = lit(canvas)
        assert {(50, 30), (30, 30), (40, 40), (40, 20)} <= pixels
        assert (40, 30) not in pixels

    def test_outline_is_symmetric(self):
        canvas = Canvas()
        canvas.circle(40, 30, 9)
        pixels = lit(canvas)
        assert pixels == {(80 - x, y) for x, y in pixels}
        assert pixels == {(x, 60 - y) for x, y in pixels}

    def test_outline_stays_near_the_radius(self):
        canvas = Canvas()
        canvas.circle(64, 32, 20)
        for x, y in lit(canvas):
            assert abs(((x - 64) ** 2 + (y - 32) ** 2) ** 0.5 - 20) < 1

    def test_filled_area_is_close_to_pi_r_squared(self):
        canvas = Canvas()
        canvas.circle(64, 32, 20, fill=True)
        area = len(lit(canvas))
        assert abs(area - 3.14159 * 20**2) / area < 0.05
        assert (64, 32) in lit(canvas)


class TestFont:
    def test_covers_printable_ascii(self):
        assert all(chr(code) in font.GLYPHS for code in range(0x20, 0x7F))

    def test_covers_french_accents(self):
        assert all(char in font.GLYPHS for char in "àâçéèêëîïôùûÀÇÉÈ")

    def test_every_glyph_fits_its_box(self):
        for char, rows in font.GLYPHS.items():
            assert len(rows) == font.GLYPH_HEIGHT, char
            assert all(0 <= row < 1 << font.GLYPH_WIDTH for row in rows), char

    def test_space_is_blank_and_everything_else_is_not(self):
        for char, rows in font.GLYPHS.items():
            assert (char == " ") == (not any(rows)), char

    def test_unknown_characters_get_the_replacement_box(self):
        assert font.glyph("☃") == font.REPLACEMENT

    def test_malformed_source_is_rejected(self):
        with pytest.raises(ValueError, match="malformed glyph"):
            font._parse("##### #...#")


class TestText:
    def test_a_glyph_lands_where_the_font_says(self):
        canvas = Canvas()
        canvas.text(10, 20, "A")
        expected = {
            (10 + column, 20 + row)
            for row, bits in enumerate(font.glyph("A"))
            for column in range(5)
            if bits & (1 << (4 - column))
        }
        assert lit(canvas) == expected

    def test_characters_advance_by_one_cell(self):
        one, two = Canvas(), Canvas()
        one.text(0, 0, "H")
        two.text(0, 0, " H")
        assert {(x + font.CELL_WIDTH, y) for x, y in lit(one)} == lit(two)

    def test_newline_returns_to_the_left_margin(self):
        one, two = Canvas(), Canvas()
        one.text(3, 0, "X")
        two.text(3, 0, "\nX")
        assert {(x, y + font.CELL_HEIGHT) for x, y in lit(one)} == lit(two)

    def test_scale_two_doubles_every_pixel(self):
        small, large = Canvas(), Canvas()
        small.text(0, 0, "e")
        large.text(0, 0, "e", scale=2)
        assert len(lit(large)) == 4 * len(lit(small))
        assert all(large.get_pixel(2 * x, 2 * y) for x, y in lit(small))

    def test_value_zero_erases_text(self):
        canvas = Canvas()
        canvas.clear(1)
        canvas.text(0, 0, "I", value=0)
        assert len(lit(canvas)) == 128 * 64 - sum(bin(row).count("1") for row in font.glyph("I"))

    def test_text_is_clipped_at_the_edge(self):
        canvas = Canvas()
        canvas.text(120, 60, "WWWW")
        assert all(x < 128 and y < 64 for x, y in lit(canvas))

    def test_a_full_line_fits_twenty_one_characters(self):
        assert text_size("x" * 21)[0] <= 128 < text_size("x" * 22)[0]

    def test_text_size(self):
        assert text_size("Hi") == (11, 7)
        assert text_size("Hi", scale=2) == (22, 14)
        assert text_size("a\nbcd") == (17, 15)
        assert text_size("") == (0, 7)

    @pytest.mark.parametrize("scale", [0, -1, 1.5, True])
    def test_bad_scale_is_refused(self, scale):
        with pytest.raises(ValueError, match="scale"):
            Canvas().text(0, 0, "x", scale=scale)


class TestOutput:
    def test_robot_accepts_a_canvas(self, robot, fake_robot):
        canvas = Canvas()
        canvas.text(0, 0, "Eilik")
        robot.write_screen(canvas)
        assert bytes(fake_robot.framebuffer) == rotate180(bytes(canvas))

    def test_ascii_preview(self):
        canvas = Canvas()
        canvas.pixel(0, 0)
        art = canvas.to_ascii()
        assert art.splitlines()[0].startswith("#.")
        assert len(art.splitlines()) == 64

    def test_save_png(self, tmp_path):
        path = tmp_path / "canvas.png"
        Canvas().save_png(str(path), scale=1)
        assert path.read_bytes().startswith(b"\x89PNG")

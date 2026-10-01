"""GIF decoding, animations, the reference .fb/.mv formats, and playback."""

from __future__ import annotations

import io
import random
import struct
import time

import pytest

from eilik.animation import Animation, PlaybackReport, load_animation, load_motion, play
from eilik.canvas import Canvas
from eilik.cli import EXIT_INTERRUPTED, main
from eilik.errors import ImageError, ProtocolError
from eilik.gif import MINIMUM_DELAY, decode_gif, load_gif
from eilik.protocol import Command
from eilik.robot import Eilik
from eilik.screen import rotate180
from eilik.servo import Motor
from eilik.simulator import SimulatedEilik

#: Two 8x4 frames with their own delays (0.12 s, 0.25 s); the second frame has
#: a local colour table. Written by Pillow, embedded so the decoder is covered
#: even where Pillow is not installed.
TINY_GIF = bytes.fromhex(
    "47494638396108000400810000000000ffffff00000000000021ff0b4e45545343415045322e3003"
    "0100000021f904000c0000002c000000000800040000080e0003001848b0a0c18308070608080021"
    "f90401190003002c000000000800040081000000808080ffffff000000081000010c1848b0a08082"
    "05032044082020003b"
)


def make_gif(
    width: int,
    height: int,
    palette: list[tuple[int, int, int]],
    frames: list[dict],
) -> bytes:
    """Write a GIF from explicit frames, for testing against the specification.

    Each frame is a dict with ``rect`` (left, top, width, height), ``indices``
    (palette indices, row by row) and optionally ``disposal``, ``transparent``
    and ``delay`` (hundredths). The LZW stream is "uncompressed": a clear code
    every two literals keeps the code width at three bits.
    """
    assert len(palette) == 4  # minimum code size 2: codes 0-3, clear 4, end 5

    def lzw(indices: list[int]) -> bytes:
        codes = []
        for start in range(0, len(indices), 2):
            codes += [4, *indices[start : start + 2]]
        codes.append(5)
        bits = count = 0
        out = bytearray()
        for code in codes:
            bits |= code << count
            count += 3
            while count >= 8:
                out.append(bits & 0xFF)
                bits >>= 8
                count -= 8
        if count:
            out.append(bits & 0xFF)
        chunks = [out[i : i + 255] for i in range(0, len(out), 255)]
        return bytes([2]) + b"".join(bytes([len(c)]) + c for c in chunks) + b"\x00"

    data = bytearray(b"GIF89a")
    data += struct.pack("<HHBBB", width, height, 0x81, 0, 0)  # global table of 4 colours
    data += bytes(channel for colour in palette for channel in colour)
    for frame in frames:
        transparent = frame.get("transparent")
        flags = (frame.get("disposal", 0) << 2) | (transparent is not None)
        control = struct.pack("<BHB", flags, frame.get("delay", 10), transparent or 0)
        data += b"\x21\xf9\x04" + control + b"\x00"
        left, top, w, h = frame["rect"]
        data += b"\x2c" + struct.pack("<HHHHB", left, top, w, h, 0) + lzw(frame["indices"])
    return bytes(data + b"\x3b")


#: Black, white, mid grey, and a spare.
PALETTE = [(0, 0, 0), (255, 255, 255), (128, 128, 128), (0, 0, 0)]
BLACK, WHITE, GREY = 0, 1, 2

#: A first frame covering the whole 4x2 screen in white.
FULL_WHITE = {"rect": (0, 0, 4, 2), "indices": [WHITE] * 8}


def frames_of(count: int) -> list[Canvas]:
    """Distinct canvases: frame ``i`` has a single lit pixel at ``(i, i)``."""
    canvases = []
    for index in range(count):
        canvas = Canvas()
        canvas.pixel(index, index)
        canvases.append(canvas)
    return canvases


class Recorder:
    """A stand-in robot that records what it is sent, optionally slowly."""

    def __init__(self, delay: float = 0.0) -> None:
        """Record writes, taking ``delay`` seconds for each screen write."""
        self.delay = delay
        self.screens: list[bytes] = []
        self.servos: list[dict[Motor, int]] = []
        self.moves: list[dict[Motor, int]] = []

    def write_screen(self, framebuffer: bytes) -> None:
        time.sleep(self.delay)
        self.screens.append(bytes(framebuffer))

    def write_servos(self, positions: dict[Motor, int]) -> None:
        self.servos.append(positions)

    def move(self, positions: dict[Motor, int], duration: float = 0.5) -> None:
        del duration
        self.moves.append(positions)


# -- GIF -------------------------------------------------------------------------


class TestGif:
    def test_embedded_fixture(self):
        first, second = decode_gif(TINY_GIF)
        assert (first.delay, second.delay) == (0.12, 0.25)
        assert first.image.pixel(0, 0) == 255
        assert first.image.pixel(7, 3) == 255
        assert second.image.pixel(3, 1) == 255
        assert second.image.pixel(4, 2) == 128
        assert second.image.pixel(0, 0) == 0

    def test_load_gif_reads_a_file(self, tmp_path):
        path = tmp_path / "tiny.gif"
        path.write_bytes(TINY_GIF)
        assert len(load_gif(path)) == 2

    @pytest.mark.parametrize(
        ("data", "message"),
        [
            (b"\x89PNG\r\n\x1a\n", "not a GIF"),
            (b"GIF89a\x01", "truncated"),
            (b"GIF89a\x00\x00\x01\x00\x00\x00\x00;", "positive"),
            (b"GIF89a\x01\x00\x01\x00\x00\x00\x00;", "no image"),
            (b"GIF89a\xff\xff\xff\xff\x00\x00\x00;", "larger than"),
            (b"GIF89a\x01\x00\x01\x00\x00\x00\x00!", "cut short"),
            (b"GIF89a\x01\x00\x01\x00\x00\x00\x00\x99", "unexpected block"),
            (
                b"GIF89a\x01\x00\x01\x00\x00\x00\x00,\x00\x00\x00\x00\x01\x00\x01\x00\x00\x02",
                "colour",
            ),
        ],
    )
    def test_malformed_files(self, data, message):
        with pytest.raises(ImageError, match=message):
            decode_gif(data)

    def test_bad_lzw_code_size(self):
        data = bytearray(TINY_GIF)
        data[data.index(b",") + 10] = 12  # the first image's minimum code size
        with pytest.raises(ImageError, match="LZW code size"):
            decode_gif(bytes(data))

    def test_bad_background(self):
        with pytest.raises(ValueError, match="background"):
            decode_gif(TINY_GIF, background=-1)


class TestGifAgainstPillow:
    """Pillow encodes known frames; decoding must give back exactly those frames.

    Pillow optimises animations with partial frames, transparency and the
    disposal methods, and interlaces taller images, so this exercises the
    compositing without relying on any decoder but ours.
    """

    @staticmethod
    def encode(frames, **options) -> bytes:
        buffer = io.BytesIO()
        frames[0].save(buffer, "GIF", save_all=True, append_images=frames[1:], **options)
        return buffer.getvalue()

    @staticmethod
    def moving_square(image_module, count: int = 6):
        frames = []
        for index in range(count):
            frame = image_module.new("L", (64, 32), 0)
            for y in range(8, 20):
                for x in range(4 + 6 * index, 16 + 6 * index):
                    frame.putpixel((x, y), 255)
            frame.putpixel((63, 31), 128)
            frames.append(frame)
        return frames

    # Disposal 3 is missing on purpose: Pillow's encoder computes its partial
    # frames as if "restore to previous" meant the previous frame, rather than
    # the state before the current frame was drawn, which is what the
    # specification (and browsers) say. Its disposal-3 output is no oracle;
    # TestGifSpecification covers it instead.
    @pytest.mark.parametrize(
        "options",
        [{}, {"optimize": False, "interlace": False}, {"disposal": 1}, {"disposal": 2}],
        ids=["optimised", "plain", "disposal-1", "disposal-2"],
    )
    def test_compositing(self, options):
        image_module = pytest.importorskip("PIL.Image")
        frames = self.moving_square(image_module)
        decoded = decode_gif(self.encode(frames, duration=80, **options))
        assert [frame.image.pixels for frame in decoded] == [frame.tobytes() for frame in frames]

    def test_large_noisy_frames_exercise_the_lzw_table_reset(self):
        image_module = pytest.importorskip("PIL.Image")
        rng = random.Random(7)
        frames = [
            image_module.frombytes("L", (200, 150), bytes(rng.randrange(256) for _ in range(30000)))
            for _ in range(2)
        ]
        decoded = decode_gif(self.encode(frames, duration=100))
        assert [frame.image.pixels for frame in decoded] == [frame.tobytes() for frame in frames]

    def test_tiny_delays_are_stretched_like_browsers_do(self):
        image_module = pytest.importorskip("PIL.Image")
        frames = self.moving_square(image_module, count=3)
        decoded = decode_gif(self.encode(frames, duration=[0, 10, 500]))
        assert [frame.delay for frame in decoded] == [0.1, 0.1, 0.5]
        assert MINIMUM_DELAY <= 0.1


class TestGifSpecification:
    """Compositing rules, on GIFs whose expected result is known by construction."""

    @staticmethod
    def decode(frames: list[dict], background: int = 0) -> list[list[int]]:
        gif = make_gif(4, 2, PALETTE, frames)
        return [list(frame.image.pixels) for frame in decode_gif(gif, background)]

    def test_partial_frames_draw_over_what_is_there(self):
        frames = [FULL_WHITE, {"rect": (1, 1, 2, 1), "indices": [GREY, BLACK]}]
        assert self.decode(frames)[1] == [255, 255, 255, 255, 255, 128, 0, 255]

    def test_transparent_pixels_leave_the_screen_alone(self):
        frames = [FULL_WHITE, {"rect": (0, 0, 2, 1), "indices": [GREY, 3], "transparent": 3}]
        assert self.decode(frames)[1][:2] == [128, 255]

    def test_disposal_2_clears_the_rectangle_to_the_background(self):
        frames = [{**FULL_WHITE, "disposal": 2}, {"rect": (0, 0, 1, 1), "indices": [GREY]}]
        assert self.decode(frames, background=40)[1] == [128] + [40] * 7

    def test_disposal_3_restores_what_was_there_before_the_frame(self):
        frames = [
            FULL_WHITE,
            {"rect": (0, 0, 2, 1), "indices": [GREY, GREY], "disposal": 3},
            {"rect": (3, 1, 1, 1), "indices": [BLACK]},
        ]
        decoded = self.decode(frames)
        assert decoded[1][:2] == [128, 128]  # shown while it is the current frame
        assert decoded[2] == [255] * 7 + [0]  # then undone: the grey is gone

    def test_disposal_1_keeps_the_frame(self):
        frames = [
            FULL_WHITE,
            {"rect": (0, 0, 2, 1), "indices": [GREY, GREY], "disposal": 1},
            {"rect": (3, 1, 1, 1), "indices": [BLACK]},
        ]
        assert self.decode(frames)[2] == [128, 128] + [255] * 5 + [0]

    def test_uncovered_pixels_show_the_background(self):
        frames = [{"rect": (1, 0, 1, 1), "indices": [WHITE]}]
        assert self.decode(frames, background=9)[0] == [9, 255] + [9] * 6

    def test_frames_off_the_screen_are_clipped(self):
        frames = [{"rect": (3, 1, 2, 2), "indices": [WHITE] * 4}]
        assert self.decode(frames)[0] == [0] * 7 + [255]


# -- animations --------------------------------------------------------------------


class TestAnimation:
    def test_from_frames(self):
        animation = Animation.from_frames(frames_of(4), fps=8)
        assert len(animation) == 4
        assert animation.duration == pytest.approx(0.5)
        assert animation.frames[1] == bytes(frames_of(2)[1])

    def test_at_fps(self):
        animation = Animation.from_frames(frames_of(3), fps=10).at_fps(30)
        assert animation.durations == [pytest.approx(1 / 30)] * 3

    @pytest.mark.parametrize(
        ("arguments", "message"),
        [
            (([], []), "at least one frame"),
            (([bytes(1024)], [0.1, 0.1]), "durations"),
            (([bytes(1024)], [0]), "positive"),
            (([bytes(1024)], [0.1], [(1500, 1500, 1500, 1500)] * 2), "motion track"),
        ],
    )
    def test_validation(self, arguments, message):
        with pytest.raises(ValueError, match=message):
            Animation(*arguments)

    def test_wrong_frame_size(self):
        with pytest.raises(ProtocolError, match="1024"):
            Animation([bytes(10)], [0.1])

    def test_bad_fps(self):
        with pytest.raises(ValueError, match="fps"):
            Animation.from_frames(frames_of(1), fps=0)


class TestReferenceFormats:
    def test_fb_and_mv_round_trip(self, tmp_path):
        motion = [(1500 + i, 1400, 1600, 1500 - i) for i in range(5)]
        original = Animation([bytes(c) for c in frames_of(5)], [0.1] * 5, motion)
        original.save_fb(tmp_path / "clip.fb", tmp_path / "clip.mv")
        loaded = load_animation(tmp_path / "clip.fb", motion=tmp_path / "clip.mv")
        assert loaded.frames == original.frames
        assert loaded.motion == motion
        assert loaded.durations == [pytest.approx(1 / 30)] * 5  # the reference player's rate

    def test_fb_frames_are_stored_in_panel_order(self, tmp_path):
        frame = bytes(frames_of(3)[2])
        Animation([frame], [0.1]).save_fb(tmp_path / "one.fb")
        assert (tmp_path / "one.fb").read_bytes() == rotate180(frame)

    def test_motion_entries_are_four_little_endian_shorts(self, tmp_path):
        path = tmp_path / "track.mv"
        path.write_bytes(struct.pack("<8H", 1, 2, 3, 4, 1500, 1600, 1700, 1800))
        assert load_motion(path) == [(1, 2, 3, 4), (1500, 1600, 1700, 1800)]

    def test_truncated_files(self, tmp_path):
        (tmp_path / "bad.mv").write_bytes(b"\x00" * 7)
        with pytest.raises(ImageError, match="8-byte entries"):
            load_motion(tmp_path / "bad.mv")
        (tmp_path / "bad.fb").write_bytes(b"\x00" * 1000)
        with pytest.raises(ImageError, match="1024-byte frames"):
            load_animation(tmp_path / "bad.fb")

    def test_motion_must_match_the_frames(self, tmp_path):
        Animation([bytes(1024)] * 3, [0.1] * 3).save_fb(tmp_path / "clip.fb")
        (tmp_path / "clip.mv").write_bytes(struct.pack("<4H", 1500, 1500, 1500, 1500))
        with pytest.raises(ImageError, match="1 entries but the animation has 3 frames"):
            load_animation(tmp_path / "clip.fb", motion=tmp_path / "clip.mv")

    def test_saving_motion_that_does_not_exist(self, tmp_path):
        with pytest.raises(ValueError, match="no motion track"):
            Animation([bytes(1024)], [0.1]).save_fb(tmp_path / "a.fb", tmp_path / "a.mv")


class TestLoadAnimation:
    def test_gif_keeps_its_timing(self, tmp_path):
        path = tmp_path / "tiny.gif"
        path.write_bytes(TINY_GIF)
        animation = load_animation(path, fit="stretch")
        assert animation.durations == [0.12, 0.25]
        assert Canvas(animation.frames[0]).get_pixel(0, 0) == 1

    def test_fps_overrides_gif_timing(self, tmp_path):
        path = tmp_path / "tiny.gif"
        path.write_bytes(TINY_GIF)
        assert load_animation(path, fps=5).durations == [0.2, 0.2]

    def test_png_folder_in_natural_order(self, tmp_path):
        for index, canvas in enumerate(frames_of(11)):
            canvas.save_png(str(tmp_path / f"frame{index}.png"), scale=1)
        (tmp_path / "notes.txt").write_text("ignored")
        animation = load_animation(tmp_path)
        assert animation.frames == [bytes(canvas) for canvas in frames_of(11)]
        assert animation.durations[0] == pytest.approx(1 / 12)

    def test_single_png(self, tmp_path):
        frames_of(1)[0].save_png(str(tmp_path / "still.png"), scale=1)
        assert len(load_animation(tmp_path / "still.png", fps=2)) == 1

    def test_empty_folder(self, tmp_path):
        with pytest.raises(ImageError, match=r"no \.png files"):
            load_animation(tmp_path)

    def test_unsupported_kind(self, tmp_path):
        (tmp_path / "clip.mp4").write_bytes(b"")
        with pytest.raises(ImageError, match="cannot play"):
            load_animation(tmp_path / "clip.mp4")


# -- playback ----------------------------------------------------------------------


class TestPlay:
    def test_frames_go_out_in_order_on_time(self):
        robot = Recorder()
        animation = Animation.from_frames(frames_of(5), fps=25)
        report = play(robot, animation)
        assert robot.screens == animation.frames
        assert report.shown == 5
        assert report.dropped == 0
        assert report.elapsed == pytest.approx(0.2, abs=0.05)

    def test_loops_and_speed(self):
        robot = Recorder()
        animation = Animation.from_frames(frames_of(4), fps=20)
        report = play(robot, animation, loops=3, speed=2)
        assert len(robot.screens) == 12
        assert report.elapsed == pytest.approx(3 * 0.2 / 2, abs=0.05)

    def test_late_frames_are_dropped_not_delayed(self):
        """A slow robot falls behind; the animation keeps its total duration."""
        robot = Recorder(delay=0.05)
        animation = Animation.from_frames(frames_of(10), fps=50)  # 20 ms a frame
        report = play(robot, animation)
        assert report.dropped > 0
        assert report.shown + report.dropped == 10
        assert robot.screens[-1] == animation.frames[-1]  # the last frame always shows
        assert report.elapsed < 10 * 0.05  # not stretched to the robot's pace

    def test_motion_is_decimated_and_ends_at_rest(self):
        robot = Recorder()
        motion = [(1500 + 10 * i, 1500, 1500, 1500) for i in range(7)]
        animation = Animation([bytes(c) for c in frames_of(7)], [0.01] * 7, motion)
        play(robot, animation, motion_every=3)
        assert [entry[Motor.ARM_RIGHT] for entry in robot.servos] == [1500, 1530, 1560]
        assert robot.moves == [dict.fromkeys(Motor, 1500)]

    def test_rest_can_be_skipped(self):
        robot = Recorder()
        animation = Animation([bytes(1024)], [0.01], [(1600, 1500, 1500, 1500)])
        play(robot, animation, rest=False)
        assert robot.moves == []

    def test_on_frame(self):
        seen = []
        play(Recorder(), Animation.from_frames(frames_of(3), fps=100), on_frame=seen.append)
        assert seen == [0, 1, 2]

    @pytest.mark.parametrize("options", [{"loops": -1}, {"speed": 0}, {"motion_every": 0}])
    def test_bad_options(self, options):
        with pytest.raises(ValueError):
            play(Recorder(), Animation.from_frames(frames_of(1)), **options)

    def test_report(self):
        report = PlaybackReport(shown=30, dropped=2, elapsed=1.0)
        assert report.fps == 30
        assert str(report) == "30 frames in 1.00s (30.0 fps), 2 dropped"
        assert PlaybackReport(0, 0, 0.0).fps == 0

    def test_screen_and_motion_never_crash_the_robot(self):
        """Every write waits for its acknowledgement.

        So the strict simulator never sees a servo frame land while a screen
        frame is still being processed.
        """
        motion = [(1400 + 20 * i, 1600 - 20 * i, 1500, 1500 + 10 * i) for i in range(12)]
        animation = Animation([bytes(c) for c in frames_of(12)], [0.02] * 12, motion)
        with SimulatedEilik() as sim, Eilik(port=sim.port) as robot:
            report = play(robot, animation, loops=2, motion_every=1)
            assert not sim.crashed
            assert sim.violations == []
        assert report.shown + report.dropped == 24


class TestPlayCommand:
    def test_plays_on_the_robot(self, fake_robot, tmp_path, capsys):
        path = tmp_path / "tiny.gif"
        path.write_bytes(TINY_GIF)
        assert main(["--port", fake_robot.port, "play", str(path), "--fps", "50"]) == 0
        commands = [command for command, _ in fake_robot.received]
        assert commands.count(Command.WRITE_SCREEN) == 2
        assert "2 frames in" in capsys.readouterr().out

    def test_motion_track_option(self, fake_robot, tmp_path):
        Animation([bytes(1024)] * 3, [0.1] * 3).save_fb(tmp_path / "clip.fb")
        (tmp_path / "clip.mv").write_bytes(struct.pack("<12H", *([1600, 1500, 1500, 1500] * 3)))
        port = ["--port", fake_robot.port]
        assert (
            main([*port, "play", str(tmp_path / "clip.fb"), "--motion", str(tmp_path / "clip.mv")])
            == 0
        )
        assert fake_robot.servos[Motor.ARM_RIGHT] == 1500  # back at rest afterwards
        writes = [data for command, data in fake_robot.received if command == Command.WRITE_SERVOS]
        assert writes[0][:4] == bytes([4, 1]) + (1600).to_bytes(2, "little")

    def test_mismatched_motion_is_an_error(self, tmp_path, capsys):
        Animation([bytes(1024)] * 2, [0.1] * 2).save_fb(tmp_path / "clip.fb")
        (tmp_path / "clip.mv").write_bytes(b"\x00" * 8)
        assert main(["play", str(tmp_path / "clip.fb"), "--motion", str(tmp_path / "clip.mv")]) == 1
        assert "1 entries" in capsys.readouterr().err

    def test_preview_off_a_terminal_summarises(self, tmp_path, capsys):
        path = tmp_path / "tiny.gif"
        path.write_bytes(TINY_GIF)
        assert main(["play", str(path), "--preview"]) == 0
        out = capsys.readouterr().out.splitlines()
        assert out[0] == "2 frames, 0.37s per pass"
        assert len(out) == 1 + 32

    def test_preview_on_a_terminal_plays_it(self, tmp_path, capsys, monkeypatch):
        monkeypatch.setattr("sys.stdout.isatty", lambda: True)
        path = tmp_path / "tiny.gif"
        path.write_bytes(TINY_GIF)
        assert main(["play", str(path), "--preview", "--speed", "4"]) == 0
        out = capsys.readouterr().out
        assert out.startswith("\x1b[?1049h")
        assert out.count("\x1b[H") == 2  # one redraw per frame
        assert "2 frames in" in out

    def test_ctrl_c_stops_cleanly(self, fake_robot, tmp_path, capsys, monkeypatch):
        def interrupted(*_args, **_kwargs):
            raise KeyboardInterrupt

        monkeypatch.setattr("eilik.cli.play", interrupted)
        path = tmp_path / "tiny.gif"
        path.write_bytes(TINY_GIF)
        assert main(["--port", fake_robot.port, "play", str(path)]) == EXIT_INTERRUPTED
        assert "stopped" in capsys.readouterr().err

    @pytest.mark.parametrize("option", [["--fps", "0"], ["--speed", "-1"], ["--loop", "-2"]])
    def test_bad_options_are_usage_errors(self, tmp_path, option):
        with pytest.raises(SystemExit) as excinfo:
            main(["play", str(tmp_path / "x.gif"), *option])
        assert excinfo.value.code == 2

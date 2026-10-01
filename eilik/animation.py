"""Animations: loading them and playing them on the robot, on time.

An :class:`Animation` is a list of framebuffers, how long each stays on screen,
and optionally a motion track giving the four servo targets for each frame. It
loads from an animated GIF, a folder of PNGs, or the ``.fb``/``.mv`` streams the
original macOS tools write (``tools/video2fb.py`` and ``tools/audio2motion.py``
there turn a video and its soundtrack into those)::

    animation = load_animation("cat.gif", dither=True)
    report = play(robot, animation, loops=3)
    print(report)

:func:`play` keeps an absolute schedule: a frame whose slot has already passed
is dropped rather than shown late, so a slow moment does not push the rest of
the animation behind. Every write waits for its acknowledgement, which is what
keeps screen frames and servo frames from interleaving and crashing the robot.
"""

from __future__ import annotations

import itertools
import re
import struct
import time
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

from .canvas import Canvas
from .errors import ImageError
from .gif import load_gif
from .image import load_png, to_framebuffer
from .screen import FRAMEBUFFER_SIZE, check_size, rotate180
from .servo import Motor, neutral_positions

__all__ = [
    "Animation",
    "PlaybackReport",
    "load_animation",
    "load_motion",
    "play",
]

#: Frame rate assumed for a folder of PNGs when none is given.
DEFAULT_SEQUENCE_FPS = 12.0

#: Frame rate assumed for a ``.fb`` stream when none is given, as the
#: reference player does.
DEFAULT_STREAM_FPS = 30.0

#: Servo targets go out on every this-many frames. The servos cannot follow 30
#: updates a second anyway, and the reference player uses the same decimation.
DEFAULT_MOTION_EVERY = 3

#: Bytes per frame in a ``.mv`` motion track: four ``uint16`` LE targets.
MOTION_ENTRY_SIZE = 8

Targets = tuple[int, int, int, int]


@dataclass
class Animation:
    """Frames to show, how long each one lasts, and optional servo motion.

    Attributes:
        frames: Logical framebuffers, 1024 bytes each, the right way up.
        durations: Seconds each frame stays on screen.
        motion: For each frame, the targets of motors 1 to 4, or None.
    """

    frames: list[bytes]
    durations: list[float]
    motion: list[Targets] | None = field(default=None)

    def __post_init__(self) -> None:
        """Validate that frames, durations and motion line up.

        Raises:
            ValueError: If the lists differ in length, a duration is not
                positive, or there are no frames.
            ProtocolError: If a frame is not 1024 bytes.
        """
        if not self.frames:
            raise ValueError("an animation needs at least one frame")
        self.frames = [bytes(frame) for frame in self.frames]
        for frame in self.frames:
            check_size(frame)
        if len(self.durations) != len(self.frames):
            raise ValueError(f"{len(self.frames)} frames but {len(self.durations)} durations")
        if any(not duration > 0 for duration in self.durations):
            raise ValueError("every frame duration must be positive")
        if self.motion is not None and len(self.motion) != len(self.frames):
            raise ValueError(
                f"the motion track has {len(self.motion)} entries for {len(self.frames)} frames"
            )

    @classmethod
    def from_frames(
        cls, frames: Iterable[Canvas | Sequence[int]], fps: float = DEFAULT_SEQUENCE_FPS
    ) -> Animation:
        """Build an animation from canvases or framebuffers at a constant rate."""
        _check_fps(fps)
        buffers = [bytes(frame) for frame in frames]
        return cls(buffers, [1 / fps] * len(buffers))

    def __len__(self) -> int:
        """Return the number of frames."""
        return len(self.frames)

    @property
    def duration(self) -> float:
        """Seconds one pass of the animation lasts."""
        return sum(self.durations)

    def at_fps(self, fps: float) -> Animation:
        """Return the same frames at a constant ``fps``, ignoring their own timing."""
        _check_fps(fps)
        return Animation(self.frames, [1 / fps] * len(self.frames), self.motion)

    def save_fb(self, path: str | Path, motion_path: str | Path | None = None) -> None:
        """Write the frames as a ``.fb`` stream, and the motion as ``.mv``.

        The formats are those of the reference macOS tools: frames in the
        panel's own order (rotated), motion as four ``uint16`` LE per frame.
        Timing is not stored; the player is told the frame rate.
        """
        if motion_path is not None and self.motion is None:
            raise ValueError("this animation has no motion track to save")
        Path(path).write_bytes(b"".join(rotate180(frame) for frame in self.frames))
        if motion_path is not None:
            Path(motion_path).write_bytes(
                b"".join(struct.pack("<4H", *entry) for entry in self.motion or [])
            )


def _check_fps(fps: float) -> None:
    if not fps > 0:
        raise ValueError(f"fps must be positive, got {fps}")


def _natural_key(path: Path) -> list[object]:
    """Sort ``frame2.png`` before ``frame10.png``."""
    return [int(part) if part.isdigit() else part for part in re.split(r"(\d+)", path.name)]


def load_motion(path: str | Path) -> list[Targets]:
    """Read a ``.mv`` motion track: four ``uint16`` LE targets per frame.

    Raises:
        ImageError: If the file size is not a whole number of entries.
    """
    data = Path(path).read_bytes()
    if len(data) % MOTION_ENTRY_SIZE:
        raise ImageError(
            f"{path} is {len(data)} bytes, not a whole number of {MOTION_ENTRY_SIZE}-byte entries"
        )
    return [
        struct.unpack_from("<4H", data, offset) for offset in range(0, len(data), MOTION_ENTRY_SIZE)
    ]


def load_animation(
    source: str | Path,
    *,
    fps: float | None = None,
    motion: str | Path | None = None,
    fit: str = "contain",
    threshold: int = 128,
    dither: bool = False,
    invert: bool = False,
    background: int = 0,
) -> Animation:
    """Load an animation from a GIF, a folder of PNGs, a ``.fb`` stream or a PNG.

    Args:
        source: A ``.gif`` file, a directory of ``.png`` files (played in
            natural name order, so ``2.png`` before ``10.png``), a ``.fb``
            stream from the reference tools, or a single ``.png``.
        fps: Constant frame rate. By default a GIF keeps its own timing, a
            folder plays at 12 frames a second and a ``.fb`` stream at 30.
        motion: A ``.mv`` track with one entry per frame.
        fit: How pictures are fitted; see :func:`eilik.image.to_framebuffer`.
        threshold: As for :func:`eilik.image.to_framebuffer`.
        dither: As for :func:`eilik.image.to_framebuffer`.
        invert: As for :func:`eilik.image.to_framebuffer`.
        background: As for :func:`eilik.image.to_framebuffer`.

    Raises:
        ImageError: If the source is of an unsupported kind or cannot be
            decoded, or the motion track does not match the frames.
        OSError: If a file cannot be read.
    """
    if fps is not None:
        _check_fps(fps)
    path = Path(source)
    options = {
        "fit": fit,
        "threshold": threshold,
        "dither": dither,
        "invert": invert,
        "background": background,
    }

    if path.is_dir():
        files = sorted(
            (child for child in path.iterdir() if child.suffix.lower() == ".png"),
            key=_natural_key,
        )
        if not files:
            raise ImageError(f"{path} contains no .png files")
        frames = [
            to_framebuffer(load_png(file, background=background), **options) for file in files
        ]
        rate = fps or DEFAULT_SEQUENCE_FPS
        durations = [1 / rate] * len(frames)
    elif path.suffix.lower() == ".gif":
        decoded = load_gif(path, background=background)
        frames = [to_framebuffer(frame.image, **options) for frame in decoded]
        durations = [1 / fps] * len(frames) if fps else [frame.delay for frame in decoded]
    elif path.suffix.lower() == ".fb":
        data = path.read_bytes()
        if not data or len(data) % FRAMEBUFFER_SIZE:
            raise ImageError(
                f"{path} is {len(data)} bytes, not a whole number of {FRAMEBUFFER_SIZE}-byte frames"
            )
        # The stream is in the panel's order; frames here are the right way up.
        frames = [
            rotate180(data[i : i + FRAMEBUFFER_SIZE]) for i in range(0, len(data), FRAMEBUFFER_SIZE)
        ]
        rate = fps or DEFAULT_STREAM_FPS
        durations = [1 / rate] * len(frames)
    elif path.suffix.lower() == ".png":
        frames = [to_framebuffer(load_png(path, background=background), **options)]
        durations = [1 / (fps or DEFAULT_SEQUENCE_FPS)]
    else:
        raise ImageError(f"cannot play {path}: expected a .gif, a .fb, a .png or a folder of PNGs")

    track = None
    if motion is not None:
        track = load_motion(motion)
        if len(track) != len(frames):
            raise ImageError(
                f"the motion track has {len(track)} entries but the animation has "
                f"{len(frames)} frames"
            )
    return Animation([bytes(frame) for frame in frames], durations, track)


# -- playback --------------------------------------------------------------------


class Display(Protocol):
    """What :func:`play` needs from a robot; :class:`~eilik.robot.Eilik` has it."""

    def write_screen(self, framebuffer: bytes) -> object:
        """Show a framebuffer, waiting for the acknowledgement."""

    def write_servos(self, positions: dict[Motor, int]) -> object:
        """Send servo targets, waiting for the acknowledgement."""

    def move(self, positions: dict[Motor, int], duration: float = 0.5) -> object:
        """Glide servos to targets."""


@dataclass(frozen=True)
class PlaybackReport:
    """How a playback went.

    Attributes:
        shown: Frames actually sent to the screen.
        dropped: Frames skipped because their slot had already passed.
        elapsed: Seconds from the first frame to the end of the last.
    """

    shown: int
    dropped: int
    elapsed: float

    @property
    def fps(self) -> float:
        """Frames actually shown per second."""
        return self.shown / self.elapsed if self.elapsed > 0 else 0.0

    def __str__(self) -> str:
        """Return a one-line summary."""
        return (
            f"{self.shown} frames in {self.elapsed:.2f}s ({self.fps:.1f} fps), "
            f"{self.dropped} dropped"
        )


def play(
    robot: Display,
    animation: Animation,
    *,
    loops: int = 1,
    speed: float = 1.0,
    motion_every: int = DEFAULT_MOTION_EVERY,
    rest: bool = True,
    on_frame: Callable[[int], object] | None = None,
) -> PlaybackReport:
    """Play an animation on the robot's screen, with its motion if it has one.

    Args:
        robot: The robot, usually an :class:`~eilik.robot.Eilik`.
        animation: What to play.
        loops: How many times to play it; 0 repeats until interrupted.
        speed: Playback speed; 2 plays twice as fast.
        motion_every: Send servo targets on every this-many frames.
        rest: Glide the servos back to neutral at the end, if there was motion.
        on_frame: Called with each frame's index after it is shown.

    Returns:
        How many frames were shown and dropped, and how long it took.

    Raises:
        ValueError: If ``loops`` is negative or ``speed`` or ``motion_every``
            is not positive.
    """
    if loops < 0:
        raise ValueError(f"loops must not be negative, got {loops}")
    if not speed > 0:
        raise ValueError(f"speed must be positive, got {speed}")
    if motion_every < 1:
        raise ValueError(f"motion_every must be at least 1, got {motion_every}")

    durations = [duration / speed for duration in animation.durations]
    offsets = [0.0, *itertools.accumulate(durations)]
    period = offsets[-1]
    last = len(animation.frames) - 1
    shown = dropped = 0

    start = time.monotonic()
    passes = itertools.count() if loops == 0 else range(loops)
    for iteration in passes:
        base = start + iteration * period
        for index, frame in enumerate(animation.frames):
            due = base + offsets[index]
            now = time.monotonic()
            if now >= base + offsets[index + 1] and index != last:
                dropped += 1
                continue
            if due > now:
                time.sleep(due - now)
            robot.write_screen(frame)
            if animation.motion is not None and index % motion_every == 0:
                robot.write_servos(dict(zip(Motor, animation.motion[index], strict=True)))
            shown += 1
            if on_frame is not None:
                on_frame(index)

    end = start + (loops or 1) * period
    if (remaining := end - time.monotonic()) > 0:
        time.sleep(remaining)  # let the last frame have its time on screen
    elapsed = time.monotonic() - start
    if animation.motion is not None and rest:
        robot.move(neutral_positions(), duration=0.4)
    return PlaybackReport(shown, dropped, elapsed)

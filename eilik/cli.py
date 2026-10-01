"""Command-line interface: drive the robot from a terminal.

Run as ``eilik`` once installed, or ``python -m eilik``::

    eilik text "Bonjour !"                 # centred on the screen
    eilik show photo.png --dither          # any PNG, fitted to 128x64
    eilik show logo.png --preview          # ASCII preview, no robot needed
    eilik move HEAD=1650 BODY=1400 --duration 1
    eilik center
    eilik servos
    eilik capture screen.png
    eilik play cat.gif --loop 3            # animations: GIF, PNG folder, .fb
    eilik simulate                         # a virtual robot, for trying things

``eilik simulate`` runs a simulated robot and draws its screen and servos live;
in another terminal, ``export EILIK_PORT=/tmp/eilik-sim-$UID`` and every command
above (and any script using the SDK) talks to it instead of a real robot.

Exit status: 0 on success, 1 on an error, 2 on a usage error, 3 when the link
works but the servo controller reports the all-zero fault and needs a power
cycle, 130 when interrupted with Ctrl-C.
"""

from __future__ import annotations

import argparse
import contextlib
import errno
import logging
import os
import sys
import tempfile
import time
from collections.abc import Callable
from pathlib import Path

from . import __version__
from .animation import Animation, load_animation, play
from .canvas import Canvas, text_size
from .errors import EilikError, ServoControllerFaultError
from .image import FIT_MODES, png_to_framebuffer
from .robot import Eilik
from .screen import HEIGHT, WIDTH, save_png, to_ascii, to_blocks
from .servo import Motor, describe, linear, neutral_positions, smoothstep
from .simulator import SimulatedEilik, describe_frame, render
from .transport import PORT_ENVIRONMENT_VARIABLE

__all__ = ["main"]

#: Exit status when the servo controller reports the all-zero fault.
EXIT_SERVO_FAULT = 3

#: Exit status when interrupted with Ctrl-C, as shells report SIGINT.
EXIT_INTERRUPTED = 130

_EASINGS: dict[str, Callable[[float], float]] = {"smooth": smoothstep, "linear": linear}


def _int_in(low: int, high: int | None = None) -> Callable[[str], int]:
    """Return an argparse type accepting integers in ``low..high``."""

    def parse(text: str) -> int:
        try:
            value = int(text)
        except ValueError:
            raise argparse.ArgumentTypeError(f"expected an integer, got {text!r}") from None
        if value < low or (high is not None and value > high):
            bounds = f"{low}..{high}" if high is not None else f">= {low}"
            raise argparse.ArgumentTypeError(f"must be {bounds}, got {value}")
        return value

    return parse


def _duration(text: str) -> float:
    """Parse a non-negative number of seconds."""
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected seconds, got {text!r}") from None
    if not value >= 0:  # also rejects NaN
        raise argparse.ArgumentTypeError(f"must not be negative, got {text}")
    return value


def _positive_float(text: str) -> float:
    """Parse a strictly positive number."""
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected a number, got {text!r}") from None
    if not value > 0:  # also rejects NaN
        raise argparse.ArgumentTypeError(f"must be positive, got {text}")
    return value


def _add_picture_options(command: argparse.ArgumentParser) -> None:
    """Add the options that control how pictures become 1-bit frames."""
    command.add_argument("--fit", choices=FIT_MODES, default="contain", help="default: contain")
    command.add_argument("--dither", action="store_true", help="dither; best for photographs")
    command.add_argument(
        "--threshold", type=_int_in(0, 256), default=128, help="lit from this luminance (0-256)"
    )
    command.add_argument("--invert", action="store_true", help="light the dark parts instead")
    command.add_argument("--preview", action="store_true", help="draw it here instead, no robot")


def _motor_position(spec: str) -> tuple[Motor, int]:
    """Parse ``NAME=POSITION`` or ``ID=POSITION`` for ``eilik move``."""
    name, separator, value = spec.partition("=")
    if not separator:
        raise argparse.ArgumentTypeError(f"expected MOTOR=POSITION, got {spec!r}")
    key = name.strip().upper().replace("-", "_")
    try:
        motor = Motor(int(key)) if key.isdigit() else Motor[key]
    except (KeyError, ValueError):
        choices = ", ".join(motor.name.lower() for motor in Motor)
        raise argparse.ArgumentTypeError(
            f"unknown motor {name!r}; use one of {choices} or 1-4"
        ) from None
    try:
        return motor, int(value)
    except ValueError:
        raise argparse.ArgumentTypeError(f"position must be an integer, got {value!r}") from None


def build_parser() -> argparse.ArgumentParser:
    """Return the command-line parser."""
    parser = argparse.ArgumentParser(
        prog="eilik",
        description="Drive an Eilik desktop robot over USB.",
        epilog="Exit status: 0 ok, 1 error, 2 usage error, 3 servo controller fault.",
    )
    parser.add_argument("--version", action="version", version=f"pyeilik {__version__}")
    parser.add_argument(
        "--port", help=f"serial device; default: ${PORT_ENVIRONMENT_VARIABLE}, else auto-detected"
    )
    parser.add_argument("--timeout", type=float, default=2.0, help="reply timeout in seconds")
    parser.add_argument("-v", "--verbose", action="store_true", help="log protocol details")
    commands = parser.add_subparsers(dest="command", required=True, metavar="COMMAND")

    text = commands.add_parser("text", help="write text on the screen")
    text.add_argument("message", help=r"the text; \n starts a new line")
    text.add_argument(
        "--scale", type=_int_in(1), help="glyph magnification (default: largest that fits, max 3)"
    )
    text.add_argument("--x", type=int, help="left edge (default: centred)")
    text.add_argument("--y", type=int, help="top edge (default: centred)")
    text.add_argument("--invert", action="store_true", help="dark text on a lit screen")
    text.add_argument("--preview", action="store_true", help="draw it here instead, no robot")

    show = commands.add_parser("show", help="show a PNG image on the screen")
    show.add_argument("image", help="path to a PNG file")
    _add_picture_options(show)

    play_command = commands.add_parser(
        "play", help="play an animation: a GIF, a folder of PNGs or a .fb stream"
    )
    play_command.add_argument("source", help="a .gif, a folder of .png files, a .fb or a .png")
    play_command.add_argument(
        "--fps",
        type=_positive_float,
        help="constant frame rate (default: a GIF's own timing, 12 for PNGs, 30 for .fb)",
    )
    play_command.add_argument(
        "--speed", type=_positive_float, default=1.0, help="2 plays twice as fast"
    )
    play_command.add_argument(
        "--loop", type=_int_in(0), default=1, help="times to play it; 0 repeats until Ctrl-C"
    )
    play_command.add_argument(
        "--motion", metavar="TRACK.mv", help="servo track with one entry per frame"
    )
    _add_picture_options(play_command)

    commands.add_parser("clear", help="blank the screen")

    capture = commands.add_parser("capture", help="save the screen contents as a PNG")
    capture.add_argument("output", help="PNG file to write")
    capture.add_argument("--scale", type=_int_in(1), default=4, help="magnification (default: 4)")
    capture.add_argument("--ascii", action="store_true", help="also print it as ASCII art")

    commands.add_parser("servos", help="print the servo positions")

    move = commands.add_parser("move", help="move servos, e.g. HEAD=1650 BODY=1400")
    move.add_argument("positions", nargs="+", type=_motor_position, metavar="MOTOR=POSITION")
    move.add_argument("--duration", type=_duration, default=0.5, help="seconds (default: 0.5)")
    move.add_argument("--easing", choices=sorted(_EASINGS), default="smooth")

    center = commands.add_parser("center", help="return every servo to neutral")
    center.add_argument("--duration", type=_duration, default=0.5, help="seconds (default: 0.5)")

    simulate = commands.add_parser("simulate", help="run a virtual robot to try things on")
    simulate.add_argument(
        "--link",
        default=str(Path(tempfile.gettempdir()) / f"eilik-sim-{os.getuid()}"),
        help="stable path to the simulator's port, kept across reboots (default: %(default)s)",
    )
    simulate.add_argument("--instant", action="store_true", help="servos jump, no travel time")
    simulate.add_argument(
        "--wedged", action="store_true", help="start with a wedged servo controller"
    )
    simulate.add_argument(
        "--lenient", action="store_true", help="do not crash on interleaved servo and screen frames"
    )
    simulate.add_argument("--log", action="store_true", help="print a line per frame, no drawing")
    simulate.add_argument(
        "--for", dest="seconds", type=_duration, help="stop after this many seconds"
    )
    return parser


# -- the simulator ---------------------------------------------------------------


def _point_link(link: Path, port: str) -> None:
    """Make ``link`` a symlink to ``port``, atomically, refusing to clobber a file."""
    if link.exists() and not link.is_symlink():
        raise FileExistsError(f"{link} exists and is not a symlink; pass --link elsewhere")
    staging = link.with_name(link.name + ".new")
    with contextlib.suppress(FileNotFoundError):
        staging.unlink()
    staging.symlink_to(port)
    staging.replace(link)


def _reboot(crashed: SimulatedEilik, options: dict[str, bool]) -> SimulatedEilik:
    """Bring a crashed simulator back the way the robot comes back.

    It re-enumerates on a new port, its joints are wherever they physically
    were, and its servo controller stays wedged until a power cycle.
    """
    revived = SimulatedEilik(servo_fault=True, **options)
    revived.servos.update(crashed.positions())
    revived.violations = list(crashed.violations)
    crashed.close()
    return revived


def _simulate(args: argparse.Namespace) -> int:
    """Run ``eilik simulate`` until interrupted or ``--for`` elapses."""
    link = Path(args.link)
    options = {"slew": not args.instant, "strict": not args.lenient}
    sim = SimulatedEilik(servo_fault=args.wedged, **options)
    interactive = sys.stdout.isatty() and not args.log
    deadline = None if args.seconds is None else time.monotonic() + args.seconds
    hint = f"export {PORT_ENVIRONMENT_VARIABLE}={link}"
    shown_frames = shown_violations = 0
    drawn = None
    try:
        _point_link(link, sim.port)
        if interactive:
            sys.stdout.write("\x1b[?1049h\x1b[?25l")  # alternate screen, hidden cursor
        else:
            print(f"simulated Eilik on {sim.port}, linked at {link}", flush=True)
            print(f"in another terminal: {hint}", flush=True)
        while deadline is None or time.monotonic() < deadline:
            if sim.crashed:
                sim = _reboot(sim, options)
                _point_link(link, sim.port)
                shown_frames = 0
                if not interactive:
                    print(
                        "the robot crashed and dropped off the bus; it came back with its "
                        "servo controller wedged (restart the simulator to power-cycle)",
                        flush=True,
                    )
            if interactive:
                picture = render(sim, f"Eilik simulator \u00b7 {hint} \u00b7 Ctrl-C to stop")
                if picture != drawn:
                    sys.stdout.write("\x1b[H\x1b[2J" + picture)
                    sys.stdout.flush()
                    drawn = picture
            else:
                for command, data in sim.received[shown_frames:]:
                    print(describe_frame(command, data), flush=True)
                shown_frames = len(sim.received)
                for violation in sim.violations[shown_violations:]:
                    print(f"! {violation}", flush=True)
                shown_violations = len(sim.violations)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        if interactive:
            sys.stdout.write("\x1b[?25h\x1b[?1049l")
            sys.stdout.flush()
        sim.close()
        if link.is_symlink() and str(link.readlink()) == sim.port:
            link.unlink()
    return 0


# -- rendering, which needs no robot -------------------------------------------


def _largest_scale(message: str) -> int:
    """Return the largest scale, up to 3, at which ``message`` fits on screen."""
    for scale in (3, 2):
        width, height = text_size(message, scale)
        if width <= WIDTH and height <= HEIGHT:
            return scale
    return 1


def _render_text(args: argparse.Namespace) -> Canvas:
    """Lay the message out on a canvas, centred unless a position is given."""
    message = args.message.replace("\\n", "\n")
    scale = args.scale or _largest_scale(message)
    width, height = text_size(message, scale)
    x = args.x if args.x is not None else (WIDTH - width) // 2
    y = args.y if args.y is not None else (HEIGHT - height) // 2
    canvas = Canvas()
    canvas.text(x, y, message, scale=scale)
    if args.invert:
        canvas.invert()
    return canvas


class _TerminalScreen:
    """Stands in for the robot under ``play --preview``: draws frames here."""

    def write_screen(self, framebuffer: bytes) -> None:
        sys.stdout.write("\x1b[H" + to_blocks(framebuffer))
        sys.stdout.flush()

    def write_servos(self, positions: dict[Motor, int]) -> None:
        """Servo targets have nothing to move in a terminal."""

    def move(self, positions: dict[Motor, int], duration: float = 0.5) -> None:
        """Servo targets have nothing to move in a terminal."""


def _load_animation(args: argparse.Namespace) -> Animation:
    """Load the animation named on the command line."""
    return load_animation(
        args.source,
        fps=args.fps,
        motion=args.motion,
        fit=args.fit,
        threshold=args.threshold,
        dither=args.dither,
        invert=args.invert,
    )


def _preview_animation(animation: Animation, args: argparse.Namespace) -> None:
    """Play the animation in the terminal, or summarise it when not on one."""
    summary = f"{len(animation)} frames, {animation.duration / args.speed:.2f}s per pass"
    if not sys.stdout.isatty():
        print(summary)
        print(to_blocks(animation.frames[0]))
        return
    sys.stdout.write("\x1b[?1049h\x1b[?25l\x1b[2J")
    try:
        report = play(_TerminalScreen(), animation, loops=args.loop, speed=args.speed)
    finally:
        sys.stdout.write("\x1b[?25h\x1b[?1049l")
        sys.stdout.flush()
    print(f"{summary}; {report}")


def _render_image(args: argparse.Namespace) -> Canvas:
    """Load and convert the image."""
    return Canvas(
        png_to_framebuffer(
            args.image,
            fit=args.fit,
            threshold=args.threshold,
            dither=args.dither,
            invert=args.invert,
        )
    )


# -- commands that talk to the robot ---------------------------------------------


def _run(
    robot: Eilik,
    args: argparse.Namespace,
    picture: Canvas | None,
    animation: Animation | None,
) -> None:
    """Carry out ``args.command`` on a connected robot."""
    if picture is not None:
        robot.write_screen(picture)
    elif animation is not None:
        print(play(robot, animation, loops=args.loop, speed=args.speed))
    elif args.command == "clear":
        robot.clear_screen()
    elif args.command == "capture":
        framebuffer = robot.read_screen(timeout=max(args.timeout, 5.0))
        save_png(framebuffer, args.output, scale=args.scale)
        if args.ascii:
            print(to_ascii(framebuffer))
        print(f"saved {args.output}")
    elif args.command == "servos":
        print(describe(robot.read_servos()))
    elif args.command == "move":
        targets = dict(args.positions)
        sent = robot.move(targets, duration=args.duration, easing=_EASINGS[args.easing])
        print(f"moved {describe(sent)}")
    elif args.command == "center":
        sent = robot.move(neutral_positions(), duration=args.duration)
        print(f"moved {describe(sent)}")


def main(argv: list[str] | None = None) -> int:
    """Run the command line. Returns a process exit status."""
    parser = build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )
    if args.command == "move" and len({motor for motor, _ in args.positions}) != len(
        args.positions
    ):
        parser.error("each motor may appear only once")
    if args.command == "simulate":
        return _simulate(args)

    try:
        picture = animation = None
        if args.command == "text":
            picture = _render_text(args)
        elif args.command == "show":
            picture = _render_image(args)
        elif args.command == "play":
            animation = _load_animation(args)
        if picture is not None and args.preview:
            print(to_blocks(bytes(picture)))
            return 0
        if animation is not None and args.preview:
            _preview_animation(animation, args)
            return 0

        with Eilik(port=args.port, timeout=args.timeout) as robot:
            _run(robot, args, picture, animation)
    except KeyboardInterrupt:
        print("stopped", file=sys.stderr)
        return EXIT_INTERRUPTED
    except ServoControllerFaultError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return EXIT_SERVO_FAULT
    except EilikError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"error: {exc}", file=sys.stderr)
        if getattr(exc, "errno", None) == errno.EACCES:
            print(
                "add yourself to the `uucp` group (Arch/SteamOS) or `dialout` "
                "(Debian/Ubuntu), then log back in",
                file=sys.stderr,
            )
        return 1
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())

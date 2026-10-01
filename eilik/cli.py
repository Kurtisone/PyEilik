"""Command-line interface: drive the robot from a terminal.

Run as ``eilik`` once installed, or ``python -m eilik``::

    eilik text "Bonjour !"                 # centred on the screen
    eilik show photo.png --dither          # any PNG, fitted to 128x64
    eilik show logo.png --preview          # ASCII preview, no robot needed
    eilik move HEAD=1650 BODY=1400 --duration 1
    eilik center
    eilik servos
    eilik capture screen.png

Exit status: 0 on success, 1 on an error, 2 on a usage error, 3 when the link
works but the servo controller reports the all-zero fault and needs a power
cycle.
"""

from __future__ import annotations

import argparse
import errno
import logging
import sys
from collections.abc import Callable

from . import __version__
from .canvas import Canvas, text_size
from .errors import EilikError, ServoControllerFaultError
from .image import FIT_MODES, png_to_framebuffer
from .robot import Eilik
from .screen import HEIGHT, WIDTH, save_png, to_ascii
from .servo import Motor, describe, linear, neutral_positions, smoothstep

__all__ = ["main"]

#: Exit status when the servo controller reports the all-zero fault.
EXIT_SERVO_FAULT = 3

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
    parser.add_argument("--port", help="serial device; auto-detected when omitted")
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
    text.add_argument("--preview", action="store_true", help="print it here instead")

    show = commands.add_parser("show", help="show a PNG image on the screen")
    show.add_argument("image", help="path to a PNG file")
    show.add_argument("--fit", choices=FIT_MODES, default="contain", help="default: contain")
    show.add_argument("--dither", action="store_true", help="dither; best for photographs")
    show.add_argument(
        "--threshold", type=_int_in(0, 256), default=128, help="lit from this luminance (0-256)"
    )
    show.add_argument("--invert", action="store_true", help="light the dark parts instead")
    show.add_argument("--preview", action="store_true", help="print it here instead")

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
    return parser


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


def _run(robot: Eilik, args: argparse.Namespace, picture: Canvas | None) -> None:
    """Carry out ``args.command`` on a connected robot."""
    if picture is not None:
        robot.write_screen(picture)
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

    try:
        picture = None
        if args.command == "text":
            picture = _render_text(args)
        elif args.command == "show":
            picture = _render_image(args)
        if picture is not None and args.preview:
            print(picture.to_ascii())
            return 0

        with Eilik(port=args.port, timeout=args.timeout) as robot:
            _run(robot, args, picture)
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

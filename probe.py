#!/usr/bin/env python3
"""Bring-up probe for a real Eilik robot.

Runs the safe commands end to end against attached hardware and reports what
came back: firmware identification, servo positions, an optional small test
movement, and a screen capture saved as a PNG for visual comparison with the
panel.

Examples:
    List what looks like a candidate serial port::

        python probe.py --list

    Read-only pass, safe to run at any time::

        python probe.py

    Include a small test movement and save the screen::

        python probe.py --move --png screen.png
"""

from __future__ import annotations

import argparse
import logging
import sys
import time

from eilik import (
    Eilik,
    EilikError,
    Motor,
    ServoControllerFaultError,
    ServoLimits,
    __version__,
    screen,
)
from eilik.servo import describe
from eilik.transport import DEFAULT_BAUDRATE, default_port, describe_ports

#: How far from neutral the optional test movement travels, in pulse-width units.
TEST_MOVEMENT_OFFSET = 150


def build_parser() -> argparse.ArgumentParser:
    """Return the command-line parser."""
    parser = argparse.ArgumentParser(
        prog="probe.py",
        description=__doc__.split("\n\n")[1],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", help="serial device; default: $EILIK_PORT, else auto-detected")
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"nominal line rate (default: {DEFAULT_BAUDRATE}; ignored by CDC-ACM)",
    )
    parser.add_argument("--timeout", type=float, default=2.0, help="reply timeout in seconds")
    parser.add_argument("--list", action="store_true", help="list candidate serial ports and exit")
    parser.add_argument(
        "--move",
        action="store_true",
        help="also perform a small test movement (the robot will move)",
    )
    parser.add_argument(
        "--yes", action="store_true", help="skip the confirmation prompt before moving"
    )
    parser.add_argument(
        "--png",
        metavar="PATH",
        default="eilik-screen.png",
        help="where to save the screen capture (default: eilik-screen.png)",
    )
    parser.add_argument("--no-png", action="store_true", help="skip the screen capture")
    parser.add_argument("--ascii", action="store_true", help="also print the screen as ASCII art")
    parser.add_argument("-v", "--verbose", action="store_true", help="log protocol details")
    return parser


def section(title: str) -> None:
    """Print a section header."""
    print(f"\n=== {title} " + "=" * max(0, 60 - len(title)))


def list_ports() -> int:
    """Print every serial port the machine knows about."""
    section("serial ports")
    lines = describe_ports()
    if not lines:
        print("no serial ports found; plug the robot in and check `dmesg | tail`")
        return 1
    for line in lines:
        print(f"  {line}")
    print("\nThe robot enumerates as USB CDC-ACM, so it is normally a /dev/ttyACM* node.")
    return 0


def confirm_movement() -> bool:
    """Ask before moving the robot, unless stdin is not a terminal."""
    if not sys.stdin.isatty():
        print("stdin is not a terminal; refusing to move without --yes")
        return False
    answer = input(
        f"About to move BODY and HEAD by +/-{TEST_MOVEMENT_OFFSET} around neutral. "
        "Make sure the robot has room to move. Continue? [y/N] "
    )
    return answer.strip().lower() in {"y", "yes"}


def probe_movement(robot: Eilik) -> None:
    """Run a small, symmetric test movement and return to neutral."""
    section("servo write (0xA2)")
    original = robot.read_servos()
    print(f"  starting position: {describe(original)}")

    for offset in (TEST_MOVEMENT_OFFSET, -TEST_MOVEMENT_OFFSET, 0):
        target = {
            Motor.BODY: 1500 + offset,
            Motor.HEAD: 1500 + offset,
        }
        sent = robot.write_servos(target)
        print(f"  wrote {describe(sent)}")
        time.sleep(0.4)

    print(f"  reported position: {describe(robot.read_servos())}")
    print("  restoring the starting position")
    robot.write_servos(original)


def probe_screen(robot: Eilik, path: str | None, as_ascii: bool) -> None:
    """Read the display and optionally save it as a PNG."""
    section("screen read (0xA3)")
    framebuffer = robot.read_screen(timeout=5.0)
    lit = sum(bin(byte).count("1") for byte in framebuffer)
    print(f"  {len(framebuffer)} bytes, {lit} lit pixels of {screen.WIDTH * screen.HEIGHT}")

    if lit == 0:
        print("  the screen reads as entirely blank; the panel may simply be off")

    if as_ascii:
        print()
        print(screen.to_ascii(framebuffer))

    if path:
        screen.save_png(framebuffer, path)
        print(f"  saved {path} (rotation applied, so it should match the panel)")


def main(argv: list[str] | None = None) -> int:
    """Run the probe. Returns a process exit code."""
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )

    print(f"pyeilik {__version__} probe")

    if args.list:
        return list_ports()

    try:
        port = args.port or default_port()
    except EilikError as exc:
        print(f"\nerror: {exc}")
        print("run `python probe.py --list` to see what is attached")
        return 1

    section("connection")
    print(f"  port:     {port}")
    try:
        robot = Eilik(
            port=port,
            baudrate=args.baudrate,
            timeout=args.timeout,
            limits=ServoLimits.verified(),
        )
    except EilikError as exc:
        print(f"  error: {exc}")
        return 1
    except OSError as exc:
        print(f"  error: could not open {port}: {exc}")
        print("  if this is a permissions problem, add yourself to the `uucp` group")
        print("  (Arch/SteamOS) or `dialout` (Debian/Ubuntu), then log back in")
        return 1

    with robot:
        reported = robot.transport.actual_baudrate
        print(f"  requested rate: {args.baudrate}")
        print(f"  kernel reports: {reported if reported is not None else 'unavailable'}")
        if reported is not None and reported != args.baudrate:
            print("  note: the rate is nominal on CDC-ACM, so a mismatch is not fatal")

        try:
            section("ping (0x01)")
            info = robot.ping()
            print(f"  status:   0x{info.status:02X}")
            print(f"  payload:  {info.payload.hex(' ')}")
            # Tentative layout, from one documented device; report what it says.
            if info.text and info.firmware_number is None:
                print(f"  text:     {info.text!r}")
            print(f"  firmware: {info.firmware_number or 'not at the documented offset'}")
            print(f"  boot:     {info.boot_firmware or 'not at the documented offset'}")
            if info.identifier is not None:
                print(f"  id:       0x{info.identifier:08X}")

            section("heartbeat (0x61/0xFF)")
            robot.heartbeat()
            print("  echoed")

            section("servo read (0xA1)")
            servos_ok = True
            try:
                print(f"  {describe(robot.read_servos())}")
            except ServoControllerFaultError as exc:
                # The display keeps working in this state, so carry on to it.
                servos_ok = False
                print(f"  FAULT: {exc}")

            if args.move and not servos_ok:
                print("\nskipping the test movement: the servo controller is wedged")
            elif args.move:
                if args.yes or confirm_movement():
                    probe_movement(robot)
                else:
                    print("\nskipping the test movement")

            probe_screen(robot, None if args.no_png else args.png, args.ascii)

        except EilikError as exc:
            print(f"\nerror: {type(exc).__name__}: {exc}")
            return 1

    section("done")
    if not servos_ok:
        print("  the link works, but the servo controller needs a power cycle")
        return 3
    print("  all requested commands completed")
    return 0


if __name__ == "__main__":
    sys.exit(main())

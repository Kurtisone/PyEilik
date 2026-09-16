# pyeilik

Pure-Python driver for the [Energize Lab Eilik](https://energizelab.com/) desktop
robot, targeting Linux (developed for Steam Deck / SteamOS, works on any
Arch-based or mainstream distribution).

The robot enumerates as a USB CDC-ACM serial device. This package reimplements
the community-reverse-engineered frame protocol directly in Python, so there is
no vendor library, no C extension, and no platform-specific baud-rate hack.

Only non-destructive commands are implemented. See [Safety](#safety).

## The baud-rate question, settled

The existing macOS SDK reaches 125000 baud through a proprietary `IOSSIOSPEED`
ioctl, which is why it does not port. On Linux none of that is needed:

* 125000 is not a standard POSIX rate, but the kernel exposes arbitrary rates
  through the standard `TCSETS2` ioctl with the `BOTHER` flag.
* **pyserial >= 3.0 already uses that path on Linux.** Opening a port at 125000
  works out of the box; the rate reads back correctly via `TCGETS2`.
* The rate is nominal anyway. CDC-ACM forwards a line-coding request to the
  device, but the bytes travel at USB speed, so the termios rate does not gate
  throughput.

`eilik.transport` therefore lets pyserial set the rate, **verifies it by reading
it back**, and re-applies it directly through `TCSETS2`/`BOTHER` only if it did
not stick. A rate that cannot be set is logged as a warning rather than being
fatal, because on CDC-ACM it does not stop the robot from answering.

## Install

```sh
git clone https://github.com/kurtisone/PyEilik.git
cd PyEilik
python -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
```

Requires Python 3.10+ and `pyserial`. Nothing else — PNG export is implemented
with `zlib` and `struct`, so no imaging library is needed on the target machine.

### Serial port permissions

Reading `/dev/ttyACM*` needs group membership: `uucp` on Arch/SteamOS, `dialout`
on Debian/Ubuntu.

```sh
sudo usermod -aG uucp "$USER"   # log out and back in afterwards
```

On SteamOS remember that `/usr` is read-only by default; a `udev` rule belongs in
`/etc/udev/rules.d/`, which is writable and survives updates.

## Quick start

```python
from eilik import Eilik, Motor

with Eilik() as robot:            # auto-detects a single /dev/ttyACM*
    print(robot.ping())           # firmware identification
    print(robot.read_servos())    # {Motor.ARM_RIGHT: 1500, ...}

    robot.write_servos({Motor.HEAD: 1600, Motor.BODY: 1400})
    robot.center()                # everything back to 1500

    framebuffer = robot.read_screen()       # 1024 bytes, already the right way up
    from eilik import save_png
    save_png(framebuffer, "screen.png")
```

If several `/dev/ttyACM*` nodes are present, auto-detection refuses to guess and
raises `AmbiguousPortError`; pass `Eilik(port="/dev/ttyACM1")`. Use
`python probe.py --list` to see the candidates with their USB IDs.

## Bring-up probe

`probe.py` exercises the safe commands against real hardware:

```sh
python probe.py --list              # what is attached?
python probe.py                     # read-only: ping, servos, screen -> PNG
python probe.py --move --ascii      # also a small test movement, screen as ASCII art
```

The test movement prompts for confirmation first (`--yes` to skip) and returns
the robot to the position it started from.

## Safety

The firmware accepts commands that rewrite flash or reformat the SD card. A
malformed or mistimed frame on any of them can leave the robot unbootable with
no recovery path over the serial link. This SDK refuses to build or transmit
them at all:

| opcode | name | why it is refused |
|---|---|---|
| `0x02` | `confirm_upgrade` | commits a staged firmware upgrade |
| `0x03` | `content_update` | overwrites on-device content |
| `0x04` | `firmware_flash` | writes firmware to flash |
| `0x05` | `firmware_flash_direct` | writes firmware to flash without staging |
| `0x31` | `write_specified` | writes arbitrary data to the SD card |
| `0x41` | `reinit_sd` | reinitialises the SD card |
| `0x42` | `format_sd` | formats the SD card |

The guard is `eilik.protocol.ensure_command_allowed`, and it is applied at
**two** independent points:

1. `encode_frame()`, so a destructive frame cannot be built; and
2. `SerialTransport.send_frame()`, which re-extracts the opcode from the raw
   bytes, so hand-assembling a frame and calling the low-level write path does
   not get around it either.

It is both a blacklist (`BLACKLISTED_COMMANDS`, with the reason for each entry)
and an allowlist (only members of the `Command` enum are transmittable), so a
mistyped opcode is refused as well. The `0x61` envelope nests a second opcode
inside its payload, so the check is applied one level deeper there too.

### Servo limits

Positions are clamped before transmission. The defaults are the ranges that have
actually been exercised on hardware:

| motor | id | verified range |
|---|---|---|
| `ARM_RIGHT` | 1 | 1000–2000 |
| `ARM_LEFT` | 2 | 1000–2000 |
| `BODY` | 3 | 1200–1800 |
| `HEAD` | 4 | 1200–1800 |

Neutral is 1500 for every motor. Left and right are from the robot's own point
of view, so `ARM_RIGHT` is on your left when it faces you.

Clamping *into* the verified range is silent, since that is the safe direction.
Limits can be widened, but any position that then lands outside the verified
range raises a `ServoRangeWarning`:

```python
from eilik import Eilik, ServoLimits, Motor

robot = Eilik(limits=ServoLimits.widened(HEAD=(1000, 2000)))
robot.write_servos({Motor.HEAD: 1950})   # moves, and warns
```

Body and head have only been driven through small excursions, so widening those
means measuring the mechanical stops first.

### Command sequencing

Sending a screen command (`0xA4`) while a servo command (`0xA2`) is still being
processed crashes the servo controller and the robot drops off the USB bus with
`ENXIO`. Every high-level call waits for the firmware's acknowledgement before
returning, and the transport holds a lock across each whole request/response
exchange, so frames cannot interleave even across threads.

## Protocol reference

Frame layout, identical in both directions:

```
offset      size        field
0           3           AA AA AA            magic
3           2           length (u16 LE)     total frame size minus 3
5           1           command id
6           length - 4  data
length + 2  1           checksum
```

The checksum covers the `length` field through the last data byte:

```python
def checksum(data: bytes) -> int:
    return 255 - (sum(data) % 256)
```

A frame with a bad checksum is **silently dropped** by the firmware. There is no
error reply, so corruption surfaces as a timeout — which is what
`EilikTimeoutError` says in its message.

### Implemented commands

| cmd | role | direction |
|---|---|---|
| `0x01` | ping / firmware identification | read-only |
| `0x61` | envelope; carries the `0xFF` heartbeat | read-only |
| `0xA1` | read the four servo angles | read-only |
| `0xA2` | write servo angles, up to 4 per frame | write |
| `0xA3` | read the 1024-byte framebuffer | read-only |
| `0xA4` | write the 1024-byte framebuffer | write |

### Screen format

128×64, 1 bit per pixel, SSD1306 page mode — eight 128-byte pages, each byte
covering eight vertically stacked pixels, LSB on top:

```python
bit = (framebuffer[(y // 8) * 128 + x] >> (y % 8)) & 1
```

**The panel is mounted upside down** relative to that ordering, so a 180° rotation
is applied on both read and write. `eilik.screen.rotate180` does it by reversing
the byte order of the buffer and the bit order within each byte — which, in page
mode, is exactly a half-turn. The operation is its own inverse. Callers of
`read_screen()` / `write_screen()` always work with buffers the right way up.

## Development

```sh
python -m pytest        # 171 tests, no hardware required
ruff check .
```

The suite covers the golden frame vectors, checksums, the safety guard, servo
clamping and the screen rotation, and additionally runs the transport and the
high-level API end to end against a **fake robot on a real pty** (see
`tests/fake_robot.py`) — real file descriptors, real framing, real
resynchronisation, no hardware attached.

## Status

Everything here is validated against the documented golden vectors and the
pty-backed fake, and the custom baud-rate path is confirmed working on Linux.
**None of it has yet been run against a physical robot**, so the hardware-facing
details — the exact ping payload layout, the USB VID/PID, real-world timing —
are still to be confirmed. `probe.py` exists to do exactly that; please report
what it prints.

## Credits

Protocol documentation reverse-engineered by the community and published at
<https://eiliksdk.com/protocol/>. The original macOS
[PyEilik](https://github.com/aklto/PyEilik) established the approach.

## License

MIT

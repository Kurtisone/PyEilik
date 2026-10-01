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

The guard is `eilik.protocol.ensure_frame_allowed`, and it is applied at
**two** independent points:

1. `encode_frame()`, so a destructive frame cannot be built; and
2. `SerialTransport.send_frame()`, which decodes the raw bytes in full and runs
   the same check, so hand-assembling a frame and calling the low-level write
   path does not get around it either. The bytes must be exactly one frame with
   a valid checksum, so a destructive frame cannot ride behind a harmless one.

It is both a blacklist (`BLACKLISTED_COMMANDS`, with the reason for each entry)
and an allowlist (only members of the `Command` enum are transmittable), so a
mistyped opcode is refused as well. The `0x61` envelope nests a second opcode
inside its payload, so the check is applied one level deeper there too: the
only `0x61` frame that can go out is the heartbeat. That also refuses the legacy
single-motor servo command (`0x03` inside `0x61`), which would move a motor
without going through the clamping below.

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

A lock inside one program cannot stop a second program from talking to the
robot at the same time, so the port is also opened with an exclusive lock. A
second connection to the same robot raises `PortBusyError` instead of quietly
interleaving its frames with yours (`SerialTransport(exclusive=False)` turns
this off).

## When the robot misbehaves

| symptom | what it raises | what to do |
|---|---|---|
| every servo position reads `0` | `ServoControllerFaultError` | power-cycle with the switch on the body |
| the robot drops off the USB bus | `EilikConnectionError` | reopen the port once it re-enumerates |
| the port is already open elsewhere | `PortBusyError` | close the other program (`fuser -v /dev/ttyACM0`) |
| no reply at all | `EilikTimeoutError` | check the cable and `dmesg`; a corrupted frame looks the same |

**All-zero servo positions are a fault, not a reading.** After a crash the servo
controller can stay wedged: `0xA2` is still acknowledged and `0xA1` still answers
with the right motor ids, but every position reads zero and nothing moves, while
the display keeps working normally. `read_servos()` raises rather than returning
those zeros. Reconnecting does not help, and neither does unplugging the cable:
**Eilik has an internal battery**, so the controller only resets when you turn
the robot off with the switch on its body.

**An acknowledgement only means the frame arrived intact.** The firmware
acknowledges a write to a non-existent motor just the same. To confirm that a
move happened, read the positions back with `read_servos()`.

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
| `0x61` | nonce + sub-command; only the `0xFF` heartbeat is sent | read-only |
| `0xA1` | read the four servo angles | read-only |
| `0xA2` | write servo angles, up to 4 per frame | write |
| `0xA3` | read the 1024-byte framebuffer | read-only |
| `0xA4` | write the 1024-byte framebuffer | write |

The protocol is stateless: no handshake, no session, and the link does not go
stale, so nothing needs sending while idle. The five bytes that open every
`0x61` payload are a nonce, not a session token. The firmware ignores them on
input and puts a fresh value in every frame it sends. The heartbeat is
therefore a link check, not a keep-alive.

The ping reply carries 33 bytes after a status byte. On the device the
reference documents, `"4424"` (probably the firmware number) sits at offset 1
and `"H090"` (probably the boot firmware) at offset 7. `FirmwareInfo` exposes
these as `firmware_number`, `boot_firmware` and `identifier`. They are best
effort and return `None` when the bytes do not look like that. Some commands
differ between firmware versions, so check these first when your robot does
not behave the way someone else's does.

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

Don't expect a write to read back bit for bit. The original SDK's author
reports that the firmware slightly smooths what it stores, by 58 to 429 pixels
out of 8192. Nobody has measured this independently yet.

## Development

```sh
python -m pytest        # 205 tests, no hardware required
ruff check .
```

The suite covers the golden frame vectors (including frames captured from a
real device), checksums, the safety guard, servo clamping and the screen
rotation, and additionally runs the transport and the
high-level API end to end against a **fake robot on a real pty** (see
`tests/fake_robot.py`) — real file descriptors, real framing, real
resynchronisation, no hardware attached.

## Status

Everything here is validated against the golden vectors and the pty-backed
fake, and the custom baud-rate path is confirmed working on Linux. The
implementation has also been checked against the community protocol reference,
whose findings were verified on a real robot: every frame it quotes decodes with
this implementation's checksum, and the fake robot answers the way it documents.

**None of this SDK has been run against a physical robot yet**, and the
reference itself was only exercised on macOS. Still to be confirmed on Linux
hardware: the ping payload layout, the USB VID/PID (documented nowhere, so not
hardcoded), and real-world timing. `probe.py` exists to do exactly that;
please report what it prints. It exits with status 2 if the link works but
the servo controller reports the all-zero fault.

## Credits

Protocol documentation reverse-engineered by the community and published at
<https://eiliksdk.com/protocol/>; its source is
[`PROTOCOL.md`](https://github.com/aklto/PyEilik/blob/main/PROTOCOL.md) in the
original macOS [PyEilik](https://github.com/aklto/PyEilik), which established
the approach.

## License

MIT

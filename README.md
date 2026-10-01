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

Requires Python 3.10+ and `pyserial`. Nothing else: PNG import and export are
implemented with `zlib` and `struct`, so no imaging library is needed on the
target machine. (The `dev` extra pulls in Pillow, but only so the test suite
can cross-check the built-in PNG decoder against it.)

Installing also provides the `eilik` command (or `python -m eilik`).

### Serial port permissions

The simplest way is the udev rule the package ships. It gives the logged-in
user access to the robot (no group to join, no logging out), adds the stable
name `/dev/eilik`, and keeps ModemManager from probing the robot:

```sh
eilik udev-rule | sudo tee /etc/udev/rules.d/70-eilik.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
# then unplug the robot and plug it back in
```

On SteamOS, `/etc` is writable and survives system updates (unlike `/usr`);
`sudo` needs a password, which `passwd` sets in Desktop mode if you have never
set one. The rule matches the robot's USB ID, `28e9:018a`, which is
GigaDevice's stock GD32 virtual COM port ID, so other GD32 gadgets would match
too.

Without the rule, reading `/dev/ttyACM*` needs group membership: `uucp` on
Arch/SteamOS, `dialout` on Debian/Ubuntu (`sudo usermod -aG uucp "$USER"`, then
log out and back in).

## Quick start

```python
from eilik import Canvas, Eilik, Motor, png_to_framebuffer, save_png

with Eilik() as robot:            # auto-detects a single /dev/ttyACM*
    print(robot.ping())           # firmware identification
    print(robot.read_servos())    # {Motor.ARM_RIGHT: 1500, ...}

    robot.move({Motor.HEAD: 1650, Motor.BODY: 1400}, duration=0.8)   # glide there
    robot.center()                # everything back to 1500, in one frame

    canvas = Canvas()
    canvas.text(10, 25, "Bonjour !", scale=2)
    robot.write_screen(canvas)

    robot.write_screen(png_to_framebuffer("photo.png", dither=True))
    save_png(robot.read_screen(), "screen.png")   # already the right way up
```

Auto-detection takes `/dev/eilik` if the udev rule created it, else the only
`/dev/ttyACM*` node, else the only one carrying the robot's USB ID. If that
still leaves several, it refuses to guess and raises `AmbiguousPortError`; pass `Eilik(port="/dev/ttyACM1")`, or set the
`EILIK_PORT` environment variable, which every entry point honours when no port
is given. Use `python probe.py --list` to see the candidates with their USB IDs.

## No robot yet? The simulator

`eilik simulate` runs a virtual Eilik that speaks the real protocol over a
pseudo-terminal and draws its screen and servos live:

```sh
eilik simulate                              # terminal 1: the virtual robot
export EILIK_PORT=/tmp/eilik-sim-$UID       # terminal 2: talk to it
eilik text "Bonjour !"
eilik move HEAD=1700 ARM_LEFT=1200 --duration 1
python probe.py
python my_script.py                         # any script using the SDK
```

Everything runs unchanged against a real robot later. The simulator behaves
the way `PROTOCOL.md` documents the device, including the parts that hurt:

* servos travel at the measured speeds (arms 3000 units/s, body and head
  1450), so a read during a move sees it in progress;
* a servo frame and a screen frame sent without waiting for the
  acknowledgement crash it. It drops off the bus and comes back on a new
  port with its servo controller wedged (all positions read 0, nothing moves)
  until you restart it, which is the power cycle. The `/tmp/eilik-sim-$UID`
  link follows it;
* bad checksums are ignored without a reply, `0xA5` answers tagged `0xA4`,
  `0xA6` is acknowledged but inert.

Destructive commands are the one difference: the simulator reports and
ignores them instead of executing them. The simulator shows a stand-in for the
robot's own face whenever the host does not hold the screen, and by default
behaves like the firmware that needs the screen hold; `--implicit-hold`
models the robot `PROTOCOL.md` describes, where writing a frame is enough.
Options: `--instant` (no servo
travel), `--wedged` (start with the fault), `--lenient` (do not crash on
interleaving), `--log` (one line per frame instead of the display), `--for
SECONDS`.

In Python the same thing is `eilik.SimulatedEilik`, handy in your own tests:

```python
from eilik import Eilik, Motor, SimulatedEilik

with SimulatedEilik() as sim, Eilik(port=sim.port) as robot:
    robot.move({Motor.HEAD: 1650}, duration=0.5)
    assert sim.servos[Motor.HEAD] == 1650  # the target; sim.positions() is where
                                           # the joints are now, still travelling
    assert sim.violations == []            # nothing that would hurt a real robot
```

## Command line

```sh
eilik text "Bonjour !"                     # centred, as large as fits
eilik text 'Ligne 1\nLigne 2' --scale 1    # \n starts a new line
eilik show photo.png --dither              # any PNG, fitted to 128x64
eilik show logo.png --fit cover --invert
eilik move HEAD=1650 BODY=1400 --duration 1
eilik center
eilik servos
eilik capture screen.png --ascii
eilik clear
eilik play cat.gif --loop 3                # animations, see below
eilik simulate                             # a virtual robot, see above
```

`text`, `show` and `play` take `--preview` to draw the result in the terminal
instead of sending it, which needs no robot at all. Motors are named
`arm_right`, `arm_left`, `body`, `head` (any case) or numbered 1 to 4. The exit
status is 0 on success, 1 on an error, 2 on a usage error, 3 when the servo
controller reports the all-zero fault described below, and 130 after Ctrl-C.

## Drawing and text

`Canvas` wraps a framebuffer with drawing operations. Coordinates start at the
top-left, the right way up, and everything is clipped at the edges:

```python
from eilik import Canvas, text_size

canvas = Canvas()                          # or Canvas(robot.read_screen())
canvas.rect(0, 0, 128, 64)                 # outline; fill=True to fill
canvas.line(0, 63, 127, 0)
canvas.circle(100, 32, 12, fill=True)
canvas.text(4, 4, "Température : 21°")     # built-in 5x7 font
width, height = text_size("Salut", scale=3)
canvas.text((128 - width) // 2, 40, "Salut", scale=3)
canvas.circle(100, 32, 6, value=0, fill=True)   # value=0 erases
robot.write_screen(canvas)
print(canvas.to_ascii())                   # preview in the terminal
```

The font covers printable ASCII and the French accented letters, 21 characters
per line at scale 1. A character without a glyph is drawn as a hollow box, so
it shows up instead of silently disappearing. The glyphs live in
`eilik/font.py` as rows of `#` and `.`, so they can be read and edited in place.

### Who owns the screen

The robot plays its own animations (its eyes) on the screen. Before each
frame, `write_screen()` asks it to hold the host's picture ("user-display
mode", running number 100): on some firmware the robot repaints its face
within about 50 ms otherwise. To give the screen back:

```python
robot.release_screen()                     # the robot's own face again
```

```sh
eilik release
eilik text "Back in 5 minutes" --for 300   # show it, then give the screen back
```

Running numbers 100 and 0 are the only values this SDK sends with `0xA6`; see
[the reverse-engineering notes](docs/REVERSE_ENGINEERING.md) for why.

## Images

PNG files are decoded in pure Python: every colour type and bit depth,
transparency included (interlaced files are not supported). The picture is
fitted into 128x64 and reduced to one bit per pixel:

```python
from eilik import png_to_framebuffer

png_to_framebuffer("logo.png")                      # threshold at 128: line art, text
png_to_framebuffer("photo.png", dither=True)        # Floyd-Steinberg: photos, gradients
png_to_framebuffer("photo.png", fit="cover")        # fill the screen, crop the overflow
png_to_framebuffer("drawing.png", invert=True)      # light the dark strokes
```

`fit` is `contain` (default, letterboxed), `cover` (cropped) or `stretch`.
Transparent areas and margins are dark unless `background=` says otherwise.
Dithering preserves brightness, so a dark-blue backdrop becomes a sparse field
of dots; for logos on a coloured background, a plain threshold usually looks
cleaner. A 1280x720 photograph decodes in well under a second.

Pixels from another source enter the same path as a `GrayImage`, for example
from Pillow: `to_framebuffer(GrayImage(im.width, im.height, im.convert("L").tobytes()))`.

## Smooth movement

`write_servos()` sends one frame and the joints move as fast as they can.
`move()` glides instead:

```python
from eilik import linear

robot.move({Motor.ARM_LEFT: 1800, Motor.ARM_RIGHT: 1200}, duration=1.0)
robot.move({"head": 1400}, duration=0.3, easing=linear)   # constant speed
```

It reads the current positions, then sends intermediate positions 20 times a
second on a fixed schedule, easing in and out by default. Targets are clamped
once up front, so widened limits warn once rather than at every step. Because
it starts from a read-back, `move()` refuses to run on a wedged servo
controller instead of animating a robot that will not move.

## Animations

```sh
eilik play cat.gif                          # keeps the GIF's own timing
eilik play frames/ --fps 15 --loop 0        # PNGs in name order, until Ctrl-C
eilik play cat.gif --dither --preview       # watch it in the terminal first
eilik play movie.fb --motion movie.mv       # streams from the reference tools
```

```python
from eilik import load_animation, play

animation = load_animation("cat.gif", dither=True)
print(play(robot, animation, loops=3))     # "72 frames in 4.80s (15.0 fps), 0 dropped"
```

Animated GIFs are decoded in pure Python, transparency and frame disposal
included. A folder of PNGs plays in natural order (`2.png` before `10.png`).
`.fb` and `.mv` are the formats of the original macOS tools: there,
`tools/video2fb.py` turns a video into a `.fb` frame stream and
`tools/audio2motion.py` derives a `.mv` dance track from its soundtrack, and
both files play here unchanged. `Animation.save_fb()` writes them too.

Playback keeps an absolute schedule: a frame whose time has passed is dropped
rather than shown late, so one slow moment does not push the rest of the
animation behind. Every write waits for its acknowledgement, so the screen and
the motion track can never interleave into the crash described below. Servo
targets go out every third frame, and the joints glide back to neutral at the
end.

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
| `0xA6` | running number: 100 holds the screen, 0 releases it (no other value) | write |

The protocol is stateless: no handshake, no session, and the link does not go
stale, so nothing needs sending while idle. The five bytes that open every
`0x61` payload are a nonce, not a session token. The firmware ignores them on
input and puts a fresh value in every frame it sends. The heartbeat is
therefore a link check, not a keep-alive. (One project claims a heartbeat
every 2 seconds stops the robot resuming its autonomous behaviour when idle;
nobody has shown it, so the SDK sends none. On another project's robot the
heartbeat got no reply at all until the official app had run, which is why
nothing here depends on it.)

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
python -m pytest        # 531 tests, no hardware required
ruff check . && ruff format --check .
```

CI (`.github/workflows/ci.yml`) runs both on every push, with the tests on
Python 3.10 to 3.14.

The suite covers the golden frame vectors (including frames captured from a
real device), checksums, the safety guard, servo clamping, the screen rotation,
drawing, the PNG and GIF decoders, animation playback and the command line.
The PNG decoder is checked against an independent test-side encoder covering
every filter, colour type and depth, and against Pillow; GIF compositing
against frames encoded by Pillow and against the specification. The suite
runs the transport, the CLI and the high-level API end to end against the
**simulator on a real pty** — real file descriptors, real framing, real
resynchronisation, no hardware attached. Its strict mode stays on, so a
regression that let frames interleave would crash it and fail the tests.

## Status

Everything here is validated against the golden vectors and the simulator, and
the custom baud-rate path is confirmed working on Linux. The
implementation has also been checked against the community protocol reference,
whose findings were verified on a real robot: every frame it quotes decodes with
this implementation's checksum, and the simulator answers the way it documents.

The public sources describe at least two firmware behaviours, around the
heartbeat and the screen hold; the SDK copes with both, and the simulator can
play either. [docs/REVERSE_ENGINEERING.md](docs/REVERSE_ENGINEERING.md) has
the details.

**None of this SDK has been run against a physical robot yet**, and the
reference itself was only exercised on macOS. Still to be confirmed on Linux
hardware: the ping payload layout, the USB VID/PID (documented nowhere, so not
hardcoded), and real-world timing. `probe.py` exists to do exactly that;
please report what it prints. It exits with status 3 if the link works but
the servo controller reports the all-zero fault.

## Credits

Protocol documentation reverse-engineered by the community and published at
<https://eiliksdk.com/protocol/>; its source is
[`PROTOCOL.md`](https://github.com/aklto/PyEilik/blob/main/PROTOCOL.md) in the
original macOS [PyEilik](https://github.com/aklto/PyEilik), which established
the approach. The USB ID and the screen hold come from
[strognoff/eilik-sdk](https://github.com/strognoff/eilik-sdk)'s decompilation
of the official apps and tests on their robot; the original `0x61` work is
[uDamocles/EilikSerialController](https://github.com/uDamocles/EilikSerialController).
[docs/REVERSE_ENGINEERING.md](docs/REVERSE_ENGINEERING.md) collects what each
source found, how sure it is, and where they disagree.

## License

MIT

# What is known about the Eilik, and from where

Everything this SDK does rests on community reverse engineering; EnergizeLab
publishes no interface. This page collects what the public sources say, how
sure each finding is, and what the SDK does about it. Where the sources
disagree, it says so: they were working on different robots, and the firmware
clearly differs between them.

## Sources

| source | what it did | hardware |
|---|---|---|
| [`PROTOCOL.md`](https://github.com/aklto/PyEilik/blob/main/PROTOCOL.md) in aklto/PyEilik | systematic tests of every safe command, captured frames, timing, the crash | one robot, macOS, August 2026 |
| [strognoff/eilik-sdk](https://github.com/strognoff/eilik-sdk) | decompiled the official Windows app (PyInstaller) and Android app (Hermes); USB captures of the official app | one robot, Windows + WSL (usbipd), August 2026 |
| [uDamocles/EilikSerialController](https://github.com/uDamocles/EilikSerialController) | the original work on the `0x61` servo frame | one robot |
| [ailynux/Eilik-Robot](https://github.com/ailynux/Eilik-Robot) | Bluetooth scaffolding; no services or characteristics documented yet | — |

strognoff/eilik-sdk has no licence file, so nothing is copied from it: only
facts (identifiers, values, observed behaviour) are used, with credit.

**Confidence:** *confirmed* means checked on a robot by a source and consistent
with the others; *observed* means one source saw it on its robot; *claimed*
means stated without the evidence shown; *inferred* means read out of code or
data, not tested.

## Findings

### USB identity — observed

The robot enumerates as a CDC-ACM serial port with USB ID **`28e9:018a`**
(strognoff, from `usbipd list`). `28e9` is GigaDevice; `018a` is the stock
product ID of the GD32 virtual COM port, so the microcontroller is very likely
a GD32 and other GD32 gadgets can carry the same ID.

*In the SDK:* auto-detection prefers a port with this ID when several are
plugged in; `eilik udev-rule` prints a rule that grants the desktop user
access (no group membership needed), adds `/dev/eilik`, and keeps ModemManager
from probing the robot. The rule passes `udevadm verify`.

### Who owns the screen — observed, and it differs between robots

The `0xA6` "running number" decides whether the host's frames stay on screen:

- **100 — user-display mode.** strognoff: on their firmware this must be sent
  before every `0xA4` write, or the robot's own idle animation paints over the
  frame within about 50 ms.
- **0 — release.** Both sources: the robot's own animations resume. strognoff
  notes it can come back as a status icon rather than the usual eyes.
- **Everything else.** `PROTOCOL.md` swept 0–255: every value is acknowledged,
  only 0 changed anything visible. On that robot a written frame stayed on
  screen without 100, so writing a frame took the screen by itself.

*In the SDK:* `write_screen()` sends 100 before each frame (harmless where it
is not needed), `release_screen()` and `eilik release` send 0, and `--for`
shows a picture for a while before releasing. `0xA6` is allowlisted **with
those two values only**; the rest may select stock behaviours nobody has
mapped. The simulator models both firmwares (`--implicit-hold` for the
`PROTOCOL.md` one) and draws a stand-in for the robot's face.

### The heartbeat — observed, and it differs

`aa aa aa 0a 00 61 e4 c6 f1 ca 83 ff ad`. `PROTOCOL.md`: answered on a cold
port, with a fresh nonce each time. strognoff: **no reply** on their robot
until the official app had run its own sequence once, after which it answered.

That sequence includes `0x02` (confirm_upgrade), so the SDK does not replay
it. *In the SDK:* nothing depends on the heartbeat; `probe.py` reports silence
and carries on.

### The legacy servo frame — observed, and refused

`0x61` with sub-command `0x03` moves one motor without the `0xA2` clamping.
strognoff: on current firmware it switches the robot to a turquoise "USB
control/status" screen without moving anything. *In the SDK:* refused on both
the encoder and the raw write path.

### Keep-alive and autonomous mode — claimed

uDamocles: sending the heartbeat every 2 seconds keeps the robot from
reverting to its autonomous behaviour after inactivity. `PROTOCOL.md`: the link
does not go stale after 30 s of silence and the robot sends nothing on its
own. Both can be true (the link stays usable while the robot resumes its own
behaviour), but neither showed the robot taking over. *In the SDK:* no
keep-alive is sent. If your robot starts moving on its own after a while,
that is the experiment to report.

### Servo ranges — observed, deliberately not followed

strognoff drives the joints from 500 to 2500 and describes positions as
0–3000. `PROTOCOL.md` verified 1000–2000 on the arms and only 1200–1800 on the
body and head. Going past a mechanical stop damages servos, so *the SDK keeps
the verified ranges*; `ServoLimits.widened()` exists for whoever measures
further, and warns.

### The ping reply — inferred field names, partially observed layout

The official app's table names the fields `chip_id`, `mode_number`,
`firmware_number`, `boot_firmware`, `mcu_status`. `PROTOCOL.md` found the
strings `4424` and `H090` at payload offsets 1 and 7 and a likely identifier
at 11. *In the SDK:* `FirmwareInfo` exposes those three, best effort.

### How the official app frames replies — inferred

Its receive framer (`PackAnalyData.CheckDataBase`) discards a partial frame
20 ms after its first byte. The firmware, from the same maker, may resync the
same way. The SDK writes each frame in a single call, so this does not bite.

### Bluetooth — inferred from the Android app, not implemented

The Android app talks over BLE with a longer frame: `aa aa aa`, a one-byte
length, a `headId`, a five-byte token, the command, packet number and packet
count (big-endian u16 each), up to 112 data bytes, and the checksum. `headId`
namespaces: `00` idle/heartbeat, `01` body info and parameters, `02` app data,
`03` resources, `04` OTA (firmware — as dangerous as `0x04`/`0x05` over USB),
`08` parameters. No GATT service or characteristic UUID is documented
anywhere public, and nothing in the app sends a face or a sound.

### Other facts from the official app — inferred

- It also drives other EnergizeLab products: PANXER, MATICONTROLLER, QEILIK.
- It downloads firmware from EnergizeLab servers. The SDK never contacts them,
  and never flashes anything.
- Sound: no command exists over USB in either official app; the microphone is
  processed on the robot and never crosses the cable (both sources agree).

## Still open

What a run of `python probe.py` on your robot would settle, and what to look
for:

- **Does your firmware need the screen hold?** `robot.write_screen(canvas,
  hold=False)` writes a frame without it: if the robot's face is back within a
  fraction of a second, yours does.
- **Does the heartbeat answer on a cold port?** `probe.py` says.
- **The ping layout:** paste the `payload:` line; with two or three robots the
  fields can be lined up.
- **Does the robot take over after a while?** Leave it idle after a command
  and note whether it moves on its own, and after how long.

When testing anything new: start from the simulator, use read commands only,
and never sweep opcodes — `0x04`, `0x05` and `0x42` sit right next to the
harmless ones.

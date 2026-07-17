# JMCMotorControl — UDP Command Protocol Reference

Complete syntax and response documentation for the JMC_CONTROLLINO_V3 command
system implemented by this library.

Library: https://github.com/ailagari/JMCMotorControl
Author: LAGARI A <lagariscience@gmail.com> · License: MIT

---

## 1. Transport

| Item | Value |
|---|---|
| Protocol | UDP, plain ASCII text |
| Controller listens on | `LOCAL_IP:LOCAL_PORT` (default `192.168.1.101:8001`) |
| Replies + events sent to | `PC_IP:PC_PORT` (default `192.168.1.100:8000`) |
| Packet rule | **One command per UDP packet.** No terminator needed; whitespace is trimmed |
| Unknown command | reply `INVALID COMMAND` |

On boot the controller announces itself:

```
READY:6/6          ← detected motors / configured motors
```

### Field conventions

- Commands are `HEAD:arguments`. Arguments are comma-separated.
- **Positional fields** (P, ZO, H selections, SD/P queries): the 1st field is
  motor 1, the 2nd motor 2, … An **empty field skips that motor**:
  `P:1000,,3000` moves M1 and M3, leaves M2 alone.
- **ID lists** (L, Z, R): motor numbers, e.g. `L:1,3`, or `all`, or empty = all.
- Units: speed **rps** (rev/s, decimals OK, resolution 0.1), accel/decel
  **rps/s** (resolution 0.1), position **encoder steps** (steps/rev = drive
  P17; signed 32-bit, ±2,147,483,647).

---

## 2. Motion commands

### `P` — position move (absolute)

| Send | Effect | Reply |
|---|---|---|
| `P:10000` | move motor 1 to step 10000 | `OK` |
| `P:1000,2000,3000` | synchronised move M1, M2, M3 | `OK` |
| `P:500,,1500` | M1→500, skip M2, M3→1500 | `OK` |
| `P:-40000` | negative positions allowed | `OK` |

- All addressed motors are **prepared first, then launched together**. When
  the command addresses *every* motor, launch is a slave-0 broadcast: all
  drives start in the same instant.
- Each motor moves at its stored velocity (see `V`).
- Errors: `ERR:P needs positions`, `ERR:no positions`.

### `P:?` — position query (positional)

| Send | Reads | Reply example |
|---|---|---|
| `P:?` | motor 1 | `M1=10000` |
| `P:,,?` | motor 3 | `M3=200` |
| `P:?,?` | motors 1+2 | `M1=10000,M2=2000` |

A non-responding motor reports `ERROR` and the whole reply gains a `WARN: `
prefix: `WARN: M1=10000,M2=ERROR`.

### `PA` — all motors

| Send | Effect | Reply |
|---|---|---|
| `PA:5000` | broadcast: every motor to step 5000, simultaneous start | `OK` |
| `PA:?` | positions of ALL motors | `M1=…,M2=…,…,M6=…` |

### `V` — run velocity (rps, decimals OK)

| Send | Effect | Reply |
|---|---|---|
| `V:0.5` | motor 1 velocity = 0.5 rps | `Set Motor 1 V: 0.50` |
| `V:0.5,1,0.3` | per-motor velocities | `Set V: 0.50,1.00,0.30` |
| `V:?` | read stored velocities | `V: 0.30,0.30,0.30,0.30,0.30,0.30` |

Velocity is used by the next `P`/`PA` move and by `VS`. Runtime values are
lost on controller power failure — hardcode with `setMotorVelocity()`.

### `VA` — velocity for all motors

| Send | Effect | Reply |
|---|---|---|
| `VA:1` | all motors 1 rps (also broadcast to drives) | `VA: Set all motors velocity to 1.00 rps (BROADCAST)` |

Range 0–1000 rps; out of range → `ERR:VA range 0-1000 rps`.

### `VS` — velocity mode start (endless rotation)

| Send | Effect | Reply |
|---|---|---|
| `VS` | all motors run continuously at their stored `V` speeds (synchronised broadcast start) | `VS: velocity mode started (stop with B)` |

Use velocity mode for endless rotation — it consumes no position range.
Stop with `B`.

### `AD` — acceleration / deceleration (rps/s)

| Send | Effect | Reply |
|---|---|---|
| `AD:0.2` | accel = decel = 0.2 rps/s, all motors | `AD: all motors accel=0.2 decel=0.2 rps/s` |
| `AD:0.3,0.1` | accel 0.3, decel 0.1, all motors | `AD: all motors accel=0.3 decel=0.1 rps/s` |
| `AD:?` | read per-motor values | `AD: M1=0.5/0.5,M2=0.5/0.5,… rps/s (accel/decel)` |

Ramp time = speed ÷ accel (0.3 rps at 0.1 rps/s → 3 s). Also refreshes the
quick-stop ramp (0x6085). For geared mechanisms combine `AD:0.1` with drive
parameter **P19 = 7** (speed smoothness, set on the drive panel).

---

## 3. Stopping

### `B` — brake / stop

| Send | Effect | Reply |
|---|---|---|
| `B` | controlled stop, normal deceleration ramp | `All motors stopped with deceleration` |
| `B:1` | quick stop on the deceleration ramp | `Deceleration brake applied to all motors` |
| `B:2` | fast brake (quick-stop ramp 0x6085) | `Fast brake applied to all motors` |
| `B:3` | immediate stop | `Immediate brake applied to all motors` |

### `L` — loosen brakes

| Send | Effect | Reply |
|---|---|---|
| `L` or `L:all` | release brakes, all motors | `All motor brakes loosened` |
| `L:2` | motor 2 only | `Motor 2 brake loosened` |
| `L:1,3` | motors 1 and 3 | `Motor 1,3 brake loosened` |

Note: loosening the brake also disables motor torque (drive control word
0x0003). The next move re-enables automatically.

### `BS` — brake status

```
Send:  BS
Reply: BRAKE STATUS: M1:FREE/RUN,M2:BRAKE/STOP,M3:ERROR | SUMMARY: MIXED
```
Per motor `BRAKE|FREE` / `RUN|STOP`; summary `BRAKED`, `FREE`, `MIXED` or `ERROR`.

---

## 4. Zero & homing

### `Z` — current position becomes 0 (no movement)

| Send | Effect | Reply |
|---|---|---|
| `Z` or `Z:all` | zero all motors where they stand | `All motors set to zero position` |
| `Z:1,2` | motors 1+2 | `Motor 1,2 set to zero position` |

### `ZO` — current position becomes a value (no movement)

| Send | Effect | Reply |
|---|---|---|
| `ZO:10000` | wherever M1 is now reads 10000 | `ZO: M1=10000 (current position redefined)` |
| `ZO:10000,1000` | M1 reads 10000, M2 reads 1000 | `ZO: M1=10000,M2=1000 (current position redefined)` |
| `ZO:,,500` | M3 only | `ZO: M3=500 (current position redefined)` |

### `ZS` — zero status

```
Send:  ZS
Reply: ZERO STATUS: M1:ZERO(0),M2:NOT_SET(1500) | ZERO_NOT_SET
```
`ZERO`/`NOT_SET` = whether a zero reference was set since boot; `(n)` = the
current position. Trailer `ALL_ZERO_SET` or `ZERO_NOT_SET`.

### `HV` — homing velocity (rps)

| Send | Effect | Reply |
|---|---|---|
| `HV:0.3` | motor 1 homing speed (sets both search 0x6099 and zero-find 0x609B) | `Set Motor 1 HV: 0.30` |
| `HV:0.3,0.5` | per motor | `Set HV: 0.3,0.5` |

### `HO` — homing offset (steps past the origin sensor)

| Send | Effect | Reply |
|---|---|---|
| `HO:1000` | M1: stop 1000 steps past the sensor | `HO: M1=1000 steps after sensor (used at next homing)` |
| `HO:1000,2000` | M1+M2 | `HO: M1=1000,M2=2000 steps after sensor (used at next homing)` |
| `HO:-800` | negative = other direction | as above |
| `HO:?` | read stored offsets | `HO: M1=1000,M2=0,M3=0,…` |

The offset is stored in the controller and **re-applied automatically at every
homing and drive re-initialisation**.

### `H<method>` — start homing (methods 1–12)

| Send | Effect | Reply |
|---|---|---|
| `H1` | home ALL motors with method 1 | `Homing method 1 started for all motors` |
| `H3:1,,1` | home M1 and M3 with method 3 (positional flags) | `Homing method 3 started for M1,3` |

Method summary (see the JMC manual for diagrams): 1/2 = to CW/CCW limit,
3/4 = to origin switch via CW/CCW, 5–12 = fast variants with back-off.
Watch status bit12 (homing complete) via `S` or `SD`.

---

## 5. Status & diagnostics

### `S` — status word, six formats

Syntax: `S:<motor>[,<FORMAT>]` — motor `0` = all, `1..n` = one motor.
Formats: `RAW` (default) `COMPACT` `STATE` `BITS` `FLAGS` `DETAIL`.

```
S:0          → STATUS:M1=0x1437,M2=ERROR,…
S:1,COMPACT  → M1:RUN|STP|TR|POS=10000|VEL=0.0
S:0,STATE    → STATES:M1:Device enable,M2:ERROR,…
S:1,BITS     → M1:0001 0100 0011 0111
S:1,FLAGS    → M1:READY|INIT|ENABLED|WORKING|QSTOP|REACHED|HOMED
S:0,DETAIL   → one multi-line packet PER motor
```

COMPACT tokens: `RUN|OFF|ERR` (enabled/disabled/fault), `MOV|STP`, `TR|NR`
(target reached / not reached).

### `SD` — detailed human-readable status

| Send | Reports |
|---|---|
| `SD` | all motors (one packet per motor) |
| `SD:?` / `SD:,,?` | positional: motor 1 / motor 3 |
| `SD:2` | motor 2 |

Example packet:

```
=== MOTOR 1 (Slave ID 1) Status 0x1437 ===
State: Device enable
Motor: ENABLED, stopped
Position: 10000 steps (target REACHED)
Velocity: 0.0 rps
Homing: complete | Origin switch: off
Limits: CW clear | CCW clear
Health: OK, no faults
```

On a fault the last line becomes e.g.
`FAULT: over-current (0x1001=0x2) -> send R:1 to reset`, and limit lines show
`*** ACTIVE ***`. A dead motor reports
`=== MOTOR 2 === NOT RESPONDING (TIMEOUT) - check power, slave ID, RS485 wiring`.

### `DG` — low-level register diagnosis

```
Send:  DG:1
Reply: DG M1: fmt6000=0 st6041=0x1437 mode6061=1 err1001=0x0
       tgt607A=0,10000 vel6081=0,3 acc6083=5 dec6084=5 pos6064=0,10000
       WRITE06=OK WRITE10=OK libScale=10
```

Reads back what the drive actually holds (32-bit values shown as `hi,lo`
words) and live-tests both Modbus write types. Use when a motor "replies OK
but doesn't move": `tgt607A` empty → writes not landing; `vel6081=0,0` →
velocity 0; `WRITE10=EXCEPTION(exc…)` → drive rejected the frame.

---

## 6. Administration

### `I` — detect + initialise all motors

```
MOTOR INITIALIZATION:
Motor 1 (Slave ID 1): DETECTED - Initialized in Position mode
  Status: 0x1437 (Device enable)
  Position: 10000
Motor 2 (Slave ID 2): NOT DETECTED (TIMEOUT)
  Check: Power, Address, RS485 wiring
...
Total motors detected: 1 of 6
```

### `R` — alarm reset + re-initialise

| Send | Reply |
|---|---|
| `R` or `R:all` | `All motors reset complete` |
| `R:2` | `Motor 2 reset complete` (failure: `Motor 2(FAIL) reset complete`) |

### `MC` — motor count at runtime

| Send | Reply |
|---|---|
| `MC:?` | `MC: 6 motors (slave IDs 1..6)` |
| `MC:12` | `MC: 12 motors, 7 online (not saved - set in sketch with setMotorCount)` |

Not persistent — the sketch's `setMotorCount()` applies after reboot.

### `X` — reboot the controller

Reply `System restart initiated...`, then the controller reboots and sends a
fresh `READY:n/m`.

---

## 7. Automatic events (unsolicited pushes)

A background monitor polls one motor's status every 8 ms and pushes to
`PC_IP:PC_PORT` **without being asked**:

| Event | Meaning |
|---|---|
| `EVENT:LIMIT:M2:CW POS=48211 ST=0x4677` | CW limit switch hit (position + status attached) |
| `EVENT:LIMIT:M2:CW_CLEARED` | limit released (same pair for `CCW`) |
| `EVENT:FAULT:M3 ST=0x678 ERR=0x2 CURRENT` | drive fault; 0x1001 decoded: `CURRENT VOLTAGE TEMP COMM OVERPOS PHASE GENERIC` |
| `EVENT:FAULT_CLEARED:M3` | fault cleared |
| `EVENT:OFFLINE:M4` | drive stopped answering |
| `EVENT:ONLINE:M4 ST=0x1437 (re-initialised)` | drive came (back) online and was automatically re-initialised — settings restored, but its **position counter reset to 0**: re-home before trusting coordinates |

Detection latency ≈ configured motor count × 13 ms (6 motors → <100 ms).

---

## 8. Status word bit reference (0x6041)

| Bit | Flag name | Meaning when 1 |
|---|---|---|
| 0 | READY | ready to initialise |
| 1 | INIT | drive initialisation complete |
| 2 | ENABLED | motor enabled |
| 3 | FAULT | drive error (see `SD` / 0x1001) |
| 4 | WORKING | drive working normally |
| 5 | QSTOP | (state bit; part of "Device enable" pattern) |
| 6 | INITSTATE | entering initialisation |
| 7 | WARN | warning |
| 8 | HALT | halted (after `B`) |
| 9 | RUN | motor running |
| 10 | REACHED | target reached |
| 11 | SWORIGIN | origin switch input active |
| 12 | HOMED | homing complete / ready for new position |
| 13 | POSERR | over-position error |
| 14 | CWLIM | CW limit switch active |
| 15 | CCWLIM | CCW limit switch active |

Common healthy values: `0x1437` (enabled, idle, target reached),
`0x1637` (enabled + running).

---

## 9. Quick recipes

```
# First contact
I                     detect everything
P:?                   where is motor 1?

# Speed-scale verification (once per new drive type)
V:1                   1 rps commanded
VS                    run … count shaft revs for 60 s (expect exactly 60)
B                     stop

# Gentle geared motion
AD:0.1                3-second ramps        (+ P19=7 on the drive panel)
V:0.3                 18 RPM working speed
P:40000               move

# Commission a homing axis
HV:0.2                slow homing speed
HO:1000               stop 1000 steps past the sensor
H3                    home (method 3: origin switch via CW)
ZS                    confirm

# Emergency
B:3                   immediate stop, all motors
R                     clear faults + re-initialise
```

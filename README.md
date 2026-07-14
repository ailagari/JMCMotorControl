# JMCMotorControl

**Complete UDP command system for JMC integrated servo/stepper drives on Controllino.**

Control up to **32 JMC drives** (IHSS / IHT "-RC" series — the RS-485 bus
maximum) over Modbus-RTU with a battle-tested UDP command protocol, automatic
limit-switch and fault event push, and a hardened comms layer — in a ~15-line
sketch.

Verified against the **JMC Modbus-RTU manual V2.2** and the **IHSS60-R/RC
datasheet**. *Independent project — not affiliated with JMC (Shenzhen Just
Motion Control).*

---

## Install

**Arduino IDE:** Sketch → Include Library → Add .ZIP Library… → select
`JMCMotorControl.zip`. Or copy this folder into `Documents/Arduino/libraries/`.

Requires the **CONTROLLINO** board package + library and the built-in
**Ethernet** library.

## Quick start

```cpp
#include <JMCMotorControl.h>

JMCController jmc;                            // uses Serial3 (Controllino RS-485)

void setup() {
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  jmc.setNetwork(mac, IPAddress(192,168,1,101), 8001,   // this Controllino
                      IPAddress(192,168,1,100), 8000);  // PC (replies + events)
  jmc.setMotorCount(6);                       // slave IDs 1..6
  jmc.setMotionDefaults(1.0, 1.0, 1.0, 1.0);  // accel, decel, velocity, homing vel
  jmc.begin();                                // sends "READY:n/6" to the PC
}

void loop() { jmc.run(); }                    // non-blocking
```

Flash it, then send UDP text commands from any PC/PLC/app.

## Command reference (JMC_CONTROLLINO_V3)

| Command | Action | Reply |
|---|---|---|
| `P:p1,p2,…` | synchronised absolute move (blank = skip motor) | `OK` |
| `P:1000` | move motor 1 | `OK` |
| `P:?` / `P:,,?` / `P:?,?` | positional query — `?` slots are read (`P:?`→M1, `P:,,?`→M3) | `M1=1500` … |
| `PA:<steps>` | broadcast: all motors to same position | `OK` |
| `PA:?` | positions of ALL motors | `M1=…,M2=…` |
| `S:0[,FMT]` / `S:n[,FMT]` | status; FMT = RAW COMPACT DETAIL STATE BITS FLAGS | per format |
| `SD` / `SD:?` / `SD:n` | **detailed human-readable status** (all / `?` slots / one) | plain-language report |
| `B` / `B:1` / `B:2` / `B:3` | stop all: controlled / decel / fast / immediate | confirmation |
| `L` / `L:1,2` / `L:all` | loosen brakes | confirmation |
| `BS` | brake status | `BRAKE STATUS: M1:FREE/RUN,… \| SUMMARY: …` |
| `Z` / `Z:1,2` / `Z:all` | current position becomes **0** (no movement) | confirmation |
| `ZO:v1,v2,…` | current position becomes the given **value** (no movement) | `ZO: M1=10000,…` |
| `ZS` | zero status | `ZERO STATUS: … \| ALL_ZERO_SET` |
| `R` / `R:1,2` / `R:all` | alarm reset + re-initialise | confirmation |
| `I` | detect + initialise all motors | detection report |
| `X` | reboot controller | `System restart initiated…` |
| `V:v1,…` / `V:?` | set / read stored run velocities (rps) | confirmation |
| `VA:<rps>` | broadcast velocity target | confirmation |
| `VS` | synchronised velocity-mode start | confirmation |
| `HV:v1,…` | set homing velocity (rps) — sets both search (0x6099) and zero-find (0x609B) speeds | confirmation |
| `HO:o1,…` / `HO:?` | **homing offset**: steps to move *after* the origin sensor triggers, before declaring zero (sign = direction). Stored and re-applied at every homing | `HO: M1=1000,…` |
| `H<m>` / `H<m>:1,,1` | homing method m (1–12), all / selected | confirmation |

**Typical homing setup:** `HV:0.3` (slow approach) → `HO:1000` (back off 1000
steps from the sensor) → `H3` (home with method 3) → position now reads the
offset point as reference.

## Automatic events (pushed to the PC, no request needed)

A non-blocking monitor polls one motor's status every 8 ms (full 6-motor sweep
< 100 ms) and pushes on change:

```
EVENT:LIMIT:M2:CW POS=48211 ST=0x4677     limit switch hit (also CCW)
EVENT:LIMIT:M2:CW_CLEARED                 limit released
EVENT:FAULT:M3 ST=0x678 ERR=0x2 CURRENT   drive fault, 0x1001 decoded
EVENT:FAULT_CLEARED:M3
EVENT:ONLINE:M4 / EVENT:OFFLINE:M4        drive appeared / stopped answering
```

Dead drives are re-probed every 3 s with a fast 12 ms no-retry read so they
never delay limit detection on live motors. A drive that appears later (e.g.
powered up after the controller) is automatically re-initialised.

## Scaling to 32 motors

- `jmc.setMotorCount(n)` accepts 1–32 (slave IDs 1..n).
- **Synchronised launch is broadcast-based**: when a `P:` command addresses
  every motor (and always for `VS`), the arm+start are sent as two single
  broadcast frames (slave 0) — all drives start in the same instant, with no
  per-motor stagger even at 32 motors. Partial selections launch per motor
  (~5 ms apart each).
- Monitor sweep time scales with the configured count (≈ count × 13 ms), so
  **set `setMotorCount` to the number of drives actually installed**: 6 motors
  → limit events in <100 ms; 32 motors → <450 ms.
- Boot stays fast with missing drives: every slot gets a 12 ms presence probe
  first; only responding drives get the full initialisation.

## Drive setup (once per drive)

> **⚠ Factory default is CANopen, not Modbus!** Per the IHSS60-R/RC manual,
> P43 defaults to **1** (CANopen). If a drive never answers (`I` reports NOT
> DETECTED, green LED flickering), set **P43 = 0** (RS-485/Modbus) on the
> front panel and restart the drive. This is the most common first-run issue.

- **P43 = 0** — RS-485 / Modbus-RTU mode (factory default is 1 = CANopen!). Restart required.
- Unique **slave ID** 1..N (P40 / rotary switches: `ID = S2×16 + S1`)
- **Baud** 115200 (P41 / BD switch = 7). Restart required.
- Sensor polarity **P42** (0 = PNP, 1 = NPN; factory default 1 = NPN). Restart required.
- **P45 (target speed unit)** — must match the sketch! `P45=0` (drive factory
  default, and the library default) → register unit 1 rps. `P45=1` → unit
  0.1 rps: call `jmc.setDriveP45(1)` in the sketch, or speeds will be **10×
  off** in one direction or the other.
- **P17 (pulses/revolution)** — sets how many steps = 360°. **Factory default
  is setting 2 = 1600 steps/rev.** Options: 0=user-defined (via P20), 1=800,
  2=1600, 3=3200, 4=6400, 5=12800, 6=25600, 7=51200, 8=1000, 9=2000,
  **10=4000**, 11=5000, 12=8000, 13=10000, 14=20000, 15=40000. Panel-set
  only (restart required), not reachable over Modbus. Positions in this
  library are raw steps, so `P:4000` = one full turn only when P17 = 4000.
- RS-485 A/B daisy-chain, 120 Ω termination at the last drive

**Verify speed scaling on first bring-up:** send `V:1` then `VS` — the shaft
must turn exactly 60 revolutions in 60 s. 10× off → P45 mismatch (fix P45 or
call `setDriveP45`).

## Units

| Quantity | Unit |
|---|---|
| velocity | rev/s (rps) |
| accel / decel | rps/s |
| position | encoder steps (steps/rev = drive P17) |

Register scaling (×10 per the JMC manual) is handled inside the library.

## Advanced use

`JMCController::motor(i)` returns the underlying `JMCMotor*` for direct API
calls (`moveToPosition`, `performHoming`, `getDetailedStatus`, …), and
`busRef()` exposes the raw Modbus transport (`readHolding`, `writeSingle`,
`writeMultiple`) — every call returns a `JMCResult` so comms failures are
never silent.

## Author / license / status

**Author:** LAGARI A — lagariscience@gmail.com
**License:** MIT (see LICENSE)
**Repository:** https://github.com/ailagari/JMCMotorControl

Bench-test before production use: `I` → `L` → small `P:` move → trip a limit
switch and watch for the `EVENT:LIMIT` packet.

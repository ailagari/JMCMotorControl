# JMCMotorControl

**Complete UDP command system for JMC integrated servo/stepper drives on Controllino.**

Control up to 8 JMC drives (IHSS / IHT "-RC" series) over Modbus-RTU / RS-485
with a battle-tested UDP command protocol, automatic limit-switch and fault
event push, and a hardened comms layer — in a ~15-line sketch.

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
never delay limit detection on live motors.

## Drive setup (once per drive)

- Unique **slave ID** 1..N (P40 / rotary switches: `ID = S2×16 + S1`)
- **Baud** 115200 (P41 / BD switch = 7), **P43 = 0** (RS-485 mode)
- Sensor polarity **P42** (0 = PNP, 1 = NPN)
- RS-485 A/B daisy-chain, 120 Ω termination at the last drive

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

## License / status

Author: Sentient By Elysian. Bench-test before production use: `I` → `L` →
small `P:` move → trip a limit switch and watch for the `EVENT:LIMIT` packet.

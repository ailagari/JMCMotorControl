/*
  JMC_Controllino_6Motor - complete 6-motor JMC controller in ~15 lines.
  Part of JMCMotorControl by LAGARI A <lagariscience@gmail.com> (MIT license).

  Controls JMC integrated servo/stepper drives (IHSS/IHT "-RC") over the
  Controllino's built-in RS-485 (Serial3), commanded via UDP with the
  JMC_CONTROLLINO_V3 command system. This example runs 6 motors; the library
  supports up to 32 (the RS-485 bus maximum) - just change setMotorCount().

    P:1000,2000        move motors 1+2 (reply: OK)
    P:? / P:,,?        read position of motor 1 / motor 3
    PA:?               read all positions
    S:0,COMPACT        compact status, all motors
    SD                 detailed human-readable status, all motors
    B / B:3            stop all (controlled / immediate)
    L / Z / ZO:5000    loosen brakes / zero here / redefine position
    HV:0.3 / HO:1000   homing speed / offset steps past the origin sensor
    AD:0.2 / AD:?      accel+decel rps/s, all motors (slow ramps for gears)
    MC:2 / MC:?        change/read active motor count at runtime (1..32,
                       great for bench tests; reverts to sketch on reboot)
    R / I / X          reset / detect / reboot

  The controller also PUSHES events to the PC without being asked:
    EVENT:LIMIT:M2:CW POS=... ST=...   (limit switch hit - and _CLEARED)
    EVENT:FAULT:M3 ...                 (drive fault, 0x1001 decoded)
    EVENT:ONLINE:M4 / OFFLINE:M4       (drive appeared / disappeared)

  Drive setup (once, on each drive): unique slave ID 1..6 (P40 / rotary
  switches), baud 115200 (P41=7), RS-485 mode (P43=0).

  Wiring: RS-485 A/B daisy-chain to all drives, 120 ohm termination at the
  last one. Ethernet to the same network as the PC.
*/
#include <JMCMotorControl.h>

JMCController jmc;                            // uses Serial3 (Controllino RS-485)

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

void setup() {
  Serial.begin(115200);

  jmc.setNetwork(mac,
                 IPAddress(192, 168, 1, 101), 8001,   // this Controllino
                 IPAddress(192, 168, 1, 100), 8000);  // PC (replies + events)
  // Motors on this bus: set per project (1..32, slave IDs 1..n).
  // This value is what the controller returns to after every reboot;
  // during commissioning you can override it live with the UDP command MC:n.
  jmc.setMotorCount(6);
  // Gentle defaults: accel/decel 0.5 rps/s, run 0.3 rps (18 RPM), homing 0.3 rps.
  // Raise these once your mechanics are proven, or at runtime via V / VA / HV.
  // Homing offset (steps to move after the origin sensor triggers) is set at
  // runtime with HO:  e.g. HO:1000,1000  then home with  H3
  jmc.setMotionDefaults(0.5, 0.5, 0.3, 0.3);

  // Speed units follow the JMC Modbus manual: register = rps x 10, so
  // decimal speeds like 0.3 rps (18 RPM) are exact.
  // VERIFY on first bring-up: V:1 then VS -> exactly 60 shaft revs in 60 s.
  // Only if a motor measurably runs 10x FASTER than commanded, uncomment:
  // jmc.setDriveP45(0);   // that drive uses whole-rps units

  // Per-motor HARDCODED settings (optional, motor IDs 1-based). Runtime
  // commands (V / HV / HO / MC) are lost after a power failure - values set
  // here are re-applied on every boot and whenever a drive comes back online.
  // jmc.setMotorVelocity(1, 0.5);        // M1 run speed 0.5 rps
  // jmc.setMotorHomingVelocity(1, 0.2);  // M1 homing speed 0.2 rps
  // jmc.setMotorHomingOffset(1, 1000);   // M1: 1000 steps past origin sensor

  uint8_t found = jmc.begin();                // bus + Ethernet + motors + "READY:n/6"
  Serial.print(found);
  Serial.println(F(" motor(s) detected"));
}

void loop() {
  jmc.run();                                  // non-blocking: commands + event monitor
}

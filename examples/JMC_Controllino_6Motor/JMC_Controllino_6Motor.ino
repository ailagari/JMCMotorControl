/*
  JMC_Controllino_6Motor - complete 6-motor JMC controller in ~15 lines.

  Controls up to 6 JMC integrated servo/stepper drives (IHSS/IHT "-RC")
  over the Controllino's built-in RS-485 (Serial3), commanded via UDP with
  the JMC_CONTROLLINO_V3 command system:

    P:1000,2000        move motors 1+2 (reply: OK)
    P:? / P:,,?        read position of motor 1 / motor 3
    PA:?               read all positions
    S:0,COMPACT        compact status, all motors
    SD                 detailed human-readable status, all motors
    B / B:3            stop all (controlled / immediate)
    L / Z / ZO:5000    loosen brakes / zero here / redefine position
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
  jmc.setMotorCount(6);                       // slave IDs 1..6
  // Gentle defaults: accel/decel 0.5 rps/s, run 0.3 rps (18 RPM), homing 0.3 rps.
  // Raise these once your mechanics are proven, or at runtime via V / VA / HV.
  // Homing offset (steps to move after the origin sensor triggers) is set at
  // runtime with HO:  e.g. HO:1000,1000  then home with  H3
  jmc.setMotionDefaults(0.5, 0.5, 0.3, 0.3);

  uint8_t found = jmc.begin();                // bus + Ethernet + motors + "READY:n/6"
  Serial.print(found);
  Serial.println(F(" motor(s) detected"));
}

void loop() {
  jmc.run();                                  // non-blocking: commands + event monitor
}

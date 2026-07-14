/*==============================================================================
  JMCMotorControl.h  -  JMC servo/stepper drive control library for Controllino
==============================================================================
  Author : LAGARI A <lagariscience@gmail.com>
  License: MIT
  Repo   : https://github.com/ailagari/JMCMotorControl

  Complete control stack for JMC integrated servo/stepper drives (IHSS/IHT
  "-RC" series) over Modbus-RTU / RS-485, with a ready-to-use UDP command
  system (JMC_CONTROLLINO_V3).

  Verified against the JMC Modbus-RTU manual V2.2 and the IHSS60-R/RC
  user manual.

  QUICK START (Controllino MAXI + built-in RS-485)
  ------------------------------------------------
    #include <JMCMotorControl.h>

    JMCController jmc;                       // uses Serial3 (Controllino RS-485)

    void setup() {
      byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
      jmc.setNetwork(mac, IPAddress(192,168,1,101), 8001,   // controller
                          IPAddress(192,168,1,100), 8000);  // PC
      jmc.setMotorCount(6);                  // slave IDs 1..6
      jmc.begin();                           // bus + Ethernet + motors + READY
    }

    void loop() { jmc.run(); }               // non-blocking: commands + monitor

  That's the whole sketch - the full UDP command system is then live:
    P / PA   position moves + positional '?' queries
    S / SD   status (raw formats / detailed human-readable)
    B / L / BS   brakes
    Z / ZO / ZS  zero & position redefinition
    R / I / X    reset / detect / reboot
    V / VA / VS / HV / H<m>  velocity & homing extensions
  ...plus automatic EVENT push (limit switches, faults, online/offline).

  LAYERS (all in this one header)
  -------------------------------
    JMCBus        shared RS-485 Modbus-RTU transport (CRC-checked, retries)
    JMCMotor      one drive: position/velocity/homing, correct unit scaling
    JMCController N motors + Ethernet UDP command system + event monitor

  UNITS: speed = rps, accel = rps/s, position = encoder steps (drive P17).
==============================================================================*/
#ifndef JMCMotorControl_h
#define JMCMotorControl_h

#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

//==============================================================================
//  JMCBus - hardened Modbus-RTU (RS-485) transport
//==============================================================================

// Result / error codes returned by every bus transaction.
enum JMCResult : uint8_t {
  JMC_OK = 0,          // transaction completed, response verified
  JMC_ERR_TIMEOUT,     // no (or incomplete) response before the timeout
  JMC_ERR_CRC,         // response CRC did not match
  JMC_ERR_EXCEPTION,   // slave returned a Modbus exception frame
  JMC_ERR_BADFRAME,    // wrong slave / function / byte-count in response
  JMC_ERR_PARAM        // invalid argument supplied by the caller
};

class JMCBus {
  public:
    explicit JMCBus(HardwareSerial& serial);

    // Controllino built-in RS-485 (typically Serial3).
    void beginControllino(uint32_t baud);

    // Generic RS-485 transceiver: dePin = driver enable (HIGH = TX).
    // Pass rePin too if DE and RE are wired separately.
    void begin(uint32_t baud, int dePin, int rePin = -1);

    // Tuning (sensible defaults; override any time).
    void setResponseTimeout(uint16_t ms) { _timeoutMs = ms; }
    void setRetries(uint8_t n)           { _retries = n; }
    void setInterFrameGap(uint16_t us)   { _gapUs = us; }

    // Modbus operations. Slave 0 = broadcast (writes only, no response).
    JMCResult writeSingle(uint8_t slave, uint16_t reg, uint16_t value);
    JMCResult writeMultiple(uint8_t slave, uint16_t reg,
                            const uint16_t* words, uint8_t count);
    JMCResult readHolding(uint8_t slave, uint16_t reg,
                          uint16_t* out, uint8_t count);

    uint8_t lastException() const { return _lastException; }
    static const char* resultName(JMCResult r);
    static uint16_t crc16(const uint8_t* data, uint16_t len);

  private:
    HardwareSerial& _serial;
    int      _dePin          = -1;
    int      _rePin          = -1;
    bool     _useControllino = false;
    uint16_t _timeoutMs      = 60;
    uint8_t  _retries        = 2;
    uint16_t _gapUs          = 350;   // >= 3.5 char times @ 115200
    uint32_t _lastActivityUs = 0;
    uint8_t  _lastException  = 0;

    void txEnable();
    void rxEnable();
    void interFrameGuard();
    void drainRx();
    JMCResult transact(uint8_t* req, uint8_t reqLen,
                       uint8_t* resp, uint8_t expectedRespLen,
                       uint8_t slave, uint8_t funcCode);
};

//==============================================================================
//  JMCMotor - one drive on the bus
//==============================================================================

// JMC Modbus register map (manual section 8, "Register list").
namespace JMCReg {
  const uint16_t ERROR       = 0x1001; // RO  error register
  const uint16_t FORMAT      = 0x6000; // RW  32-bit word order (0 = MSB first)
  const uint16_t QUICKSTOP   = 0x605A; // RW  quick-stop code
  const uint16_t HALTCODE    = 0x605D; // RW  halt code
  const uint16_t CONTROL     = 0x6040; // WO  control word
  const uint16_t STATUS      = 0x6041; // RO  status word
  const uint16_t MODE        = 0x6060; // WO  modes of operation
  const uint16_t MODE_DISP   = 0x6061; // RO  mode display
  const uint16_t POS_ACTUAL  = 0x6064; // RO  position actual value (INT32)
  const uint16_t VEL_ACTUAL  = 0x606C; // RO  velocity actual value (INT32)
  const uint16_t TARGET_POS  = 0x607A; // RW  target position (INT32)
  const uint16_t HOME_OFFSET = 0x607C; // RW  home offset (INT32)
  const uint16_t TARGET_VEL  = 0x6081; // RW  target velocity (INT32)
  const uint16_t ACCEL       = 0x6083; // RW  acceleration (UINT16)
  const uint16_t DECEL       = 0x6084; // RW  deceleration (UINT16)
  const uint16_t QS_DECEL    = 0x6085; // RW  quick-stop deceleration (UINT16)
  const uint16_t HOME_METHOD = 0x6098; // RW  homing method (INT16)
  const uint16_t HOME_SPEED  = 0x6099; // RW  speed to find mech. origin (U32)
  const uint16_t HOME_ACCEL  = 0x609A; // RW  homing acceleration (UINT16)
  const uint16_t HOME_ZEROSPD= 0x609B; // RW  speed to find zero origin (U32)
}

class JMCMotor {
  public:
    enum OperationMode : uint8_t {
      MODE_POSITION = 1,
      MODE_VELOCITY = 3,
      MODE_HOMING   = 6
    };

    enum QuickStopMode : uint8_t {
      QUICK_STOP_DECEL     = 1,   // current deceleration ramp
      QUICK_STOP_FAST      = 2,   // quick-stop ramp
      QUICK_STOP_IMMEDIATE = 3    // stop immediately
    };

    // Decoded status word (0x6041).
    struct MotorStatus {
      bool readyToInit, initComplete, motorEnabled, errorStatus;
      bool driveWorking, quickStopActive, initState, warning;
      bool motorHalt, motorRunning, targetReached, swMechanicalOrigin;
      bool homingComplete, overPositionError, cwLimit, ccwLimit;
      bool valid;                 // false if the status read failed
    };

    // Decoded error register (0x1001).
    struct ErrorFlags {
      bool generic, current, voltage, temperature;
      bool communication, overPosition, phaseLoss;
      uint16_t raw;
      bool valid;
    };

    JMCMotor(JMCBus& bus, uint8_t slaveId);

    JMCResult begin(float acceleration = 1.0f, float deceleration = 1.0f);
    bool isPresent();

    // Configuration
    JMCResult setAccelDecel(float accel_rps_s, float decel_rps_s);
    JMCResult setOperationMode(OperationMode mode);
    JMCResult loosenBrake();
    JMCResult enable();
    JMCResult alarmReset();

    // Position mode
    JMCResult moveToPosition(long steps, float velocity_rps, bool absolute = true);
    JMCResult preparePositionMove(long steps, float velocity_rps, bool absolute = true);
    JMCResult enableMovement();
    JMCResult startMovement();
    long getPosition();

    // Velocity mode
    JMCResult setVelocity(float velocity_rps);
    JMCResult setTargetVelocity(float velocity_rps);
    JMCResult prepareVelocityMode(float velocity_rps);
    float getVelocity();

    // Homing
    JMCResult setHomingVelocity(float velocity_rps);      // 0x6099 search speed
    JMCResult setHomingZeroVelocity(float velocity_rps);  // 0x609B zero-find speed
    JMCResult setHomingAccel(float accel_rps_s);
    // Steps to move AFTER the origin sensor triggers (0x607C). Sign = direction.
    JMCResult setHomingOffset(long steps);
    JMCResult performHoming(uint8_t method);       // methods 0..12
    JMCResult setZeroPosition();                   // current position -> 0
    JMCResult setCurrentPosition(long value);      // current position -> value

    // Stopping
    JMCResult stop();                              // controlled halt
    JMCResult quickStop(QuickStopMode mode = QUICK_STOP_FAST);
    JMCResult eStop();

    // Status / diagnostics
    uint16_t    getStatusWord();
    MotorStatus getDetailedStatus();
    ErrorFlags  getErrorFlags();
    bool        isRunning();
    bool        isTargetReached();
    bool        hasFault();

    // Broadcast (slave 0 -> all drives at once)
    JMCResult broadcastPrepareMove(long steps, float velocity_rps, bool absolute = true);
    JMCResult broadcastSetVelocity(float velocity_rps);
    JMCResult broadcastEnableMovement();
    JMCResult broadcastStartMovement();
    JMCResult broadcastStop();

    // Speed-register scale: counts written per 1 rps. Depends on drive P45:
    //   P45 = 0 -> unit 1 rps   -> scale 1  (factory default, library default)
    //   P45 = 1 -> unit 0.1 rps -> scale 10 (the Modbus manual's examples)
    void  setSpeedScale(float countsPerRps) { if (countsPerRps > 0) _speedScale = countsPerRps; }
    float speedScale() const                { return _speedScale; }

    uint8_t   slaveId() const    { return _slave; }
    JMCResult lastResult() const { return _last; }
    JMCBus&   bus() const        { return _bus; }

  private:
    JMCBus&   _bus;
    uint8_t   _slave;
    float     _accel = 1.0f;
    float     _decel = 1.0f;
    bool      _formatMSBFirst = true;   // 0x6000 == 0 (JMC default)
    uint16_t  _moveBase = 0x000F;
    float     _speedScale = 1.0f;       // counts per rps (drive P45 = 0, factory default)
    JMCResult _last = JMC_OK;

    int32_t spdReg(float rps) const;    // rps -> speed register counts
    static uint16_t toAccelReg(float rps_s);

    JMCResult write32(uint8_t slave, uint16_t reg, int32_t value);
    JMCResult read32(uint16_t reg, int32_t& out);
    JMCResult ctrl(uint16_t value);
};

//==============================================================================
//  JMCController - N motors + Ethernet UDP command system + event monitor
//==============================================================================
//  Implements the JMC_CONTROLLINO_V3 command set over UDP and a non-blocking
//  background monitor that pushes EVENT packets (limit switches, faults,
//  online/offline) to the PC. See README for the full command reference.
//==============================================================================

class JMCController {
  public:
    // RS-485 allows up to 32 Modbus slaves per bus (IHSS60-R/RC datasheet).
    static const uint8_t JMC_MAX_MOTORS = 32;

    explicit JMCController(HardwareSerial& rs485Serial = Serial3);

    // ---- Configuration (call before begin) ---------------------------------
    void setNetwork(const byte* mac,
                    IPAddress localIp, uint16_t localPort,
                    IPAddress pcIp,    uint16_t pcPort);
    void setMotorCount(uint8_t count);                 // 1..32 (slave IDs 1..count)
    void setMotionDefaults(float accel_rps_s, float decel_rps_s,
                           float velocity_rps, float homingVel_rps);

    // Match the drives' P45 "target speed unit" parameter so commanded rps is
    // physically correct. P45=0 is the drive factory default and the library
    // default; call setDriveP45(1) only if your drives are configured with
    // P45=1 (register unit 0.1 rps).
    void setDriveP45(uint8_t p45);

    // ---- Lifecycle ----------------------------------------------------------
    // Brings up RS-485 (Controllino mode), Ethernet/UDP and all motors.
    // Sends "READY:n/count" to the PC. Returns the number of motors found.
    uint8_t begin(uint32_t baud = 115200);

    // Call from loop(). Never blocks: processes one UDP command if pending,
    // then runs one background-monitor tick.
    void run();

    // ---- Direct access for advanced sketches --------------------------------
    JMCMotor* motor(uint8_t index);                    // 0-based, NULL if bad
    JMCBus&   busRef() { return _bus; }
    uint8_t   motorCount() const { return _count; }

  private:
    // Monitor timing. Limit-switch latency ~= count x (tick + ~5 ms read):
    // 6 motors @ 8 ms tick -> a CW/CCW hit is pushed in well under 100 ms.
    static const uint32_t MONITOR_TICK_MS  = 8;
    static const uint32_t OFFLINE_RETRY_MS = 3000;
    static const uint16_t PROBE_TIMEOUT_MS = 12;
    static const int      CMD_BUFFER_SIZE  = 512;   // fits P: with 32 positions

    struct MotorCache {
      bool     online;
      uint8_t  failCnt;
      uint16_t status;
      bool     cw, ccw, fault, zeroSet;
    };

    JMCBus      _bus;
    JMCMotor*   _m[JMC_MAX_MOTORS];
    uint8_t     _count = 6;
    float       _storedVel[JMC_MAX_MOTORS];
    MotorCache  _cache[JMC_MAX_MOTORS];
    uint32_t    _offlineTry[JMC_MAX_MOTORS];

    EthernetUDP _udp;
    byte        _mac[6];
    IPAddress   _localIp, _pcIp;
    uint16_t    _localPort = 8001, _pcPort = 8000;

    // Gentle factory defaults: 0.3 rps = 18 RPM run speed, soft 0.5 rps/s
    // ramps, slow 0.3 rps homing. Raise per installation with
    // setMotionDefaults() or at runtime with the V / VA / HV commands.
    float _defAccel = 0.5f, _defDecel = 0.5f;
    float _defVel   = 0.3f, _defHomingVel = 0.3f;
    float _speedScale = 1.0f;              // counts per rps (drive P45 = 0 default)
    long  _homingOffset[JMC_MAX_MOTORS];   // steps after sensor trigger (HO cmd)

    uint32_t _lastTick = 0;
    int      _monIdx   = -1;
    char     _cmdBuf[CMD_BUFFER_SIZE];

    // Comms
    void reply(const String& msg);
    void pushEvent(const String& msg) { reply(msg); }

    // Parsing helpers
    int  parseFields(const String& csv, long* value, bool* active);
    int  parseIdList(const String& csv, bool* sel);

    // Engine
    void handleUdp();
    void serviceMonitor();

    // Command handlers (JMC_CONTROLLINO_V3)
    void handlePosition(const String& args);
    void handlePositionQuery(const String& args);
    void replyPositions(const bool* sel);
    void handlePositionAll(const String& args);
    void handleStatus(const String& args);
    void handleStatusDetail(const String& args);
    void sendHumanStatus(int i);
    void sendDetailStatus(int i);
    String compactStatus(int i);
    void handleBrake(const String& args);
    void handleLoosen(const String& args);
    void handleBrakeStatus();
    void handleZero(const String& args);
    void handleZeroOffset(const String& args);
    void handleZeroStatus();
    void handleReset(const String& args);
    void handleInit();
    // Extensions
    void handleVelocity(const String& args);
    void handleVelocityAll(const String& args);
    void handleVelocityStart();
    void handleHomingVelocity(const String& args);
    void handleHomingOffset(const String& args);
    void handleHoming(const String& head, const String& args);
};

#endif // JMCMotorControl_h

/*==============================================================================
  JMCMotorControl.cpp  -  implementation (JMCBus + JMCMotor + JMCController)
  Author: LAGARI A <lagariscience@gmail.com>  |  License: MIT
==============================================================================*/
#include "JMCMotorControl.h"
#include <Controllino.h>
#include <math.h>

//==============================================================================
//  JMCBus
//==============================================================================

// Largest frame we ever build/receive with these drives (write-multiple of
// 4 x 32-bit regs = 25 bytes). 32 gives comfortable head-room.
static const uint8_t JMC_MAX_FRAME = 32;

JMCBus::JMCBus(HardwareSerial& serial) : _serial(serial) {}

void JMCBus::beginControllino(uint32_t baud) {
  _useControllino = true;
  _serial.begin(baud, SERIAL_8N1);       // JMC default: 8 data, 1 stop, no parity
  Controllino_RS485Init();
  Controllino_RS485RxEnable();
  _lastActivityUs = micros();
}

void JMCBus::begin(uint32_t baud, int dePin, int rePin) {
  _useControllino = false;
  _dePin = dePin;
  _rePin = rePin;
  if (_dePin >= 0) pinMode(_dePin, OUTPUT);
  if (_rePin >= 0) pinMode(_rePin, OUTPUT);
  _serial.begin(baud, SERIAL_8N1);
  rxEnable();
  _lastActivityUs = micros();
}

void JMCBus::txEnable() {
  if (_useControllino) { Controllino_RS485TxEnable(); return; }
  if (_rePin >= 0) digitalWrite(_rePin, HIGH);  // disable receiver (RE active-low)
  if (_dePin >= 0) digitalWrite(_dePin, HIGH);  // enable driver
}

void JMCBus::rxEnable() {
  if (_useControllino) { Controllino_RS485RxEnable(); return; }
  if (_dePin >= 0) digitalWrite(_dePin, LOW);
  if (_rePin >= 0) digitalWrite(_rePin, LOW);
}

void JMCBus::interFrameGuard() {
  // Guarantee >= 3.5 char times of bus idle before a new frame.
  uint32_t elapsed = micros() - _lastActivityUs;
  if (elapsed < _gapUs) delayMicroseconds(_gapUs - elapsed);
}

void JMCBus::drainRx() {
  while (_serial.available()) _serial.read();
}

uint16_t JMCBus::crc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else              crc >>= 1;
    }
  }
  return crc;   // transmit low byte first, then high byte
}

JMCResult JMCBus::transact(uint8_t* req, uint8_t reqLen,
                           uint8_t* resp, uint8_t expectedRespLen,
                           uint8_t slave, uint8_t funcCode) {
  uint16_t crc = crc16(req, reqLen);
  req[reqLen]     = (uint8_t)(crc & 0xFF);
  req[reqLen + 1] = (uint8_t)(crc >> 8);
  uint8_t totalReq = reqLen + 2;

  uint8_t attempts = _retries + 1;
  JMCResult lastErr = JMC_ERR_TIMEOUT;

  for (uint8_t a = 0; a < attempts; a++) {
    interFrameGuard();
    drainRx();

    txEnable();
    _serial.write(req, totalReq);
    _serial.flush();                 // block until the last bit is on the wire
    delayMicroseconds(50);           // transceiver settle before releasing DE
    rxEnable();
    _lastActivityUs = micros();

    if (slave == 0) {                // broadcast: no response expected
      delayMicroseconds(_gapUs);
      _lastActivityUs = micros();
      return JMC_OK;
    }

    uint8_t idx = 0;
    uint32_t start = millis();
    while (idx < expectedRespLen) {
      if (_serial.available()) {
        resp[idx++] = (uint8_t)_serial.read();
      } else if ((uint16_t)(millis() - start) > _timeoutMs) {
        break;
      }
    }
    _lastActivityUs = micros();

    if (idx < 5)          { lastErr = JMC_ERR_TIMEOUT;  continue; }
    if (resp[0] != slave) { lastErr = JMC_ERR_BADFRAME; continue; }

    if (resp[1] == (funcCode | 0x80)) {          // exception frame
      uint16_t rc = crc16(resp, 3);
      if (((rc & 0xFF) == resp[3]) && ((rc >> 8) == resp[4])) {
        _lastException = resp[2];
        return JMC_ERR_EXCEPTION;
      }
      lastErr = JMC_ERR_CRC;
      continue;
    }

    if (resp[1] != funcCode || idx < expectedRespLen) { lastErr = JMC_ERR_BADFRAME; continue; }

    uint16_t rc = crc16(resp, expectedRespLen - 2);
    if (((rc & 0xFF) != resp[expectedRespLen - 2]) ||
        ((rc >> 8)  != resp[expectedRespLen - 1])) { lastErr = JMC_ERR_CRC; continue; }

    _lastException = 0;
    return JMC_OK;
  }
  return lastErr;
}

JMCResult JMCBus::writeSingle(uint8_t slave, uint16_t reg, uint16_t value) {
  uint8_t req[JMC_MAX_FRAME];
  req[0] = slave;
  req[1] = 0x06;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = (uint8_t)(value >> 8);
  req[5] = (uint8_t)(value & 0xFF);
  uint8_t resp[JMC_MAX_FRAME];
  return transact(req, 6, resp, (slave == 0 ? 0 : 8), slave, 0x06);
}

JMCResult JMCBus::writeMultiple(uint8_t slave, uint16_t reg,
                                const uint16_t* words, uint8_t count) {
  if (count == 0 || (7 + count * 2 + 2) > JMC_MAX_FRAME) return JMC_ERR_PARAM;
  uint8_t req[JMC_MAX_FRAME];
  req[0] = slave;
  req[1] = 0x10;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = 0x00;
  req[5] = count;
  req[6] = count * 2;
  uint8_t p = 7;
  for (uint8_t i = 0; i < count; i++) {
    req[p++] = (uint8_t)(words[i] >> 8);
    req[p++] = (uint8_t)(words[i] & 0xFF);
  }
  uint8_t resp[JMC_MAX_FRAME];
  return transact(req, p, resp, (slave == 0 ? 0 : 8), slave, 0x10);
}

JMCResult JMCBus::readHolding(uint8_t slave, uint16_t reg,
                              uint16_t* out, uint8_t count) {
  if (slave == 0) return JMC_ERR_PARAM;            // broadcast reads illegal
  if (count == 0 || (3 + count * 2 + 2) > JMC_MAX_FRAME) return JMC_ERR_PARAM;
  uint8_t req[JMC_MAX_FRAME];
  req[0] = slave;
  req[1] = 0x03;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = 0x00;
  req[5] = count;
  uint8_t resp[JMC_MAX_FRAME];
  uint8_t expected = 3 + count * 2 + 2;
  JMCResult r = transact(req, 6, resp, expected, slave, 0x03);
  if (r != JMC_OK) return r;
  if (resp[2] != count * 2) return JMC_ERR_BADFRAME;
  for (uint8_t i = 0; i < count; i++)
    out[i] = ((uint16_t)resp[3 + i * 2] << 8) | resp[4 + i * 2];
  return JMC_OK;
}

const char* JMCBus::resultName(JMCResult r) {
  switch (r) {
    case JMC_OK:            return "OK";
    case JMC_ERR_TIMEOUT:   return "TIMEOUT";
    case JMC_ERR_CRC:       return "CRC";
    case JMC_ERR_EXCEPTION: return "EXCEPTION";
    case JMC_ERR_BADFRAME:  return "BADFRAME";
    case JMC_ERR_PARAM:     return "PARAM";
    default:                return "?";
  }
}

//==============================================================================
//  JMCMotor
//==============================================================================

// Control-word bit patterns (manual section "0x6040 control word").
static const uint16_t CW_INIT       = 0x0001; // parameter init, brake shut
static const uint16_t CW_BRAKE_OPEN = 0x0003; // drive power, brake open
static const uint16_t CW_ENABLE     = 0x000F; // operation enabled
static const uint16_t CW_SAMPLE_BIT = 0x0010; // bit4: sample position / start homing
static const uint16_t CW_RELATIVE   = 0x0040; // bit6: relative positioning
static const uint16_t CW_HALT       = 0x010F; // bit8: halt (controlled stop)
static const uint16_t CW_QUICKSTOP  = 0x0002; // bit2: quick stop
static const uint16_t CW_RESET      = 0x0080; // bit7: reset & clear error

JMCMotor::JMCMotor(JMCBus& bus, uint8_t slaveId) : _bus(bus), _slave(slaveId) {}

JMCResult JMCMotor::ctrl(uint16_t value) {
  _last = _bus.writeSingle(_slave, JMCReg::CONTROL, value);
  return _last;
}

// rps -> speed register counts. Scale follows drive P45 (see setSpeedScale):
// P45=0 (factory default) -> 1 count per rps; P45=1 -> 10 counts per rps.
int32_t JMCMotor::spdReg(float rps) const { return (int32_t)lroundf(rps * _speedScale); }

uint16_t JMCMotor::toAccelReg(float rps_s) {
  long v = lroundf(rps_s * 10.0f);
  if (v < 0)     v = 0;
  if (v > 65535) v = 65535;
  return (uint16_t)v;
}

JMCResult JMCMotor::write32(uint8_t slave, uint16_t reg, int32_t value) {
  uint16_t hi = (uint16_t)((uint32_t)value >> 16);
  uint16_t lo = (uint16_t)((uint32_t)value & 0xFFFF);
  uint16_t words[2];
  if (_formatMSBFirst) { words[0] = hi; words[1] = lo; }   // 0x6000 = 0
  else                 { words[0] = lo; words[1] = hi; }   // 0x6000 = 1
  _last = _bus.writeMultiple(slave, reg, words, 2);
  return _last;
}

JMCResult JMCMotor::read32(uint16_t reg, int32_t& out) {
  uint16_t words[2] = {0, 0};
  _last = _bus.readHolding(_slave, reg, words, 2);
  if (_last != JMC_OK) { out = 0; return _last; }
  uint32_t v;
  if (_formatMSBFirst) v = ((uint32_t)words[0] << 16) | words[1];
  else                 v = ((uint32_t)words[1] << 16) | words[0];
  out = (int32_t)v;
  return JMC_OK;
}

JMCResult JMCMotor::begin(float acceleration, float deceleration) {
  _accel = acceleration;
  _decel = deceleration;

  // Detect 32-bit word order (default 0 = MSB first).
  uint16_t fmt = 0;
  if (_bus.readHolding(_slave, JMCReg::FORMAT, &fmt, 1) == JMC_OK)
    _formatMSBFirst = (fmt == 0);

  ctrl(CW_INIT);
  ctrl(CW_BRAKE_OPEN);
  ctrl(CW_ENABLE);
  setOperationMode(MODE_POSITION);
  setAccelDecel(_accel, _decel);

  // Safe, slow homing defaults (zero speeds make the drive reject homing).
  setHomingAccel(_accel > 0.0f ? _accel : 5.0f);
  write32(_slave, JMCReg::HOME_SPEED,   spdReg(0.3f));
  write32(_slave, JMCReg::HOME_ZEROSPD, spdReg(0.3f));

  getStatusWord();          // presence probe -> sets _last
  return _last;
}

bool JMCMotor::isPresent() {
  getStatusWord();
  return _last == JMC_OK;
}

JMCResult JMCMotor::setAccelDecel(float accel_rps_s, float decel_rps_s) {
  _accel = accel_rps_s;
  _decel = decel_rps_s;
  JMCResult r = _bus.writeSingle(_slave, JMCReg::ACCEL, toAccelReg(accel_rps_s));
  if (r != JMC_OK) { _last = r; return r; }
  _last = _bus.writeSingle(_slave, JMCReg::DECEL, toAccelReg(decel_rps_s));
  // Quick-stop ramp (0x6085) defaults to 0 in the drive, which would make a
  // "fast brake" (quick-stop code 2) undefined - keep it at the normal decel
  // so B:2 always has a sane ramp. Override via bus if a harder stop is wanted.
  _bus.writeSingle(_slave, JMCReg::QS_DECEL, toAccelReg(decel_rps_s));
  return _last;
}

JMCResult JMCMotor::setOperationMode(OperationMode mode) {
  enable();
  _last = _bus.writeSingle(_slave, JMCReg::MODE, (uint16_t)mode);
  return _last;
}

JMCResult JMCMotor::loosenBrake() { return ctrl(CW_BRAKE_OPEN); }
JMCResult JMCMotor::enable()      { return ctrl(CW_ENABLE); }
JMCResult JMCMotor::alarmReset()  { return ctrl(CW_RESET); }

JMCResult JMCMotor::moveToPosition(long steps, float velocity_rps, bool absolute) {
  JMCResult r = preparePositionMove(steps, velocity_rps, absolute);
  if (r != JMC_OK) return r;
  r = enableMovement();
  if (r != JMC_OK) return r;
  return startMovement();
}

JMCResult JMCMotor::preparePositionMove(long steps, float velocity_rps, bool absolute) {
  setOperationMode(MODE_POSITION);
  _moveBase = absolute ? CW_ENABLE : (CW_ENABLE | CW_RELATIVE);
  // Relative moves use drive-side bit6 only - adding the current position in
  // software as well was the classic double-count bug.
  write32(_slave, JMCReg::TARGET_VEL, spdReg(fabs(velocity_rps)));
  write32(_slave, JMCReg::TARGET_POS, (int32_t)steps);
  return _last;
}

JMCResult JMCMotor::enableMovement() { return ctrl(_moveBase | CW_SAMPLE_BIT); }
JMCResult JMCMotor::startMovement()  { return ctrl(_moveBase); }

long JMCMotor::getPosition() {
  int32_t v = 0;
  read32(JMCReg::POS_ACTUAL, v);
  return (long)v;
}

JMCResult JMCMotor::setVelocity(float velocity_rps) {
  setOperationMode(MODE_VELOCITY);
  setTargetVelocity(velocity_rps);
  return enable();
}

JMCResult JMCMotor::setTargetVelocity(float velocity_rps) {
  return write32(_slave, JMCReg::TARGET_VEL, spdReg(velocity_rps));
}

JMCResult JMCMotor::prepareVelocityMode(float velocity_rps) {
  setOperationMode(MODE_VELOCITY);
  setTargetVelocity(velocity_rps);
  _moveBase = CW_ENABLE;
  return _last;
}

float JMCMotor::getVelocity() {
  int32_t v = 0;
  read32(JMCReg::VEL_ACTUAL, v);
  return v / _speedScale;         // same unit scale as the target registers
}

JMCResult JMCMotor::setHomingVelocity(float velocity_rps) {
  return write32(_slave, JMCReg::HOME_SPEED, spdReg(fabs(velocity_rps)));
}

JMCResult JMCMotor::setHomingZeroVelocity(float velocity_rps) {
  return write32(_slave, JMCReg::HOME_ZEROSPD, spdReg(fabs(velocity_rps)));
}

JMCResult JMCMotor::setHomingAccel(float accel_rps_s) {
  _last = _bus.writeSingle(_slave, JMCReg::HOME_ACCEL, toAccelReg(accel_rps_s));
  return _last;
}

JMCResult JMCMotor::setHomingOffset(long steps) {
  return write32(_slave, JMCReg::HOME_OFFSET, (int32_t)steps);
}

JMCResult JMCMotor::performHoming(uint8_t method) {
  if (method > 12) { _last = JMC_ERR_PARAM; return _last; }
  setOperationMode(MODE_HOMING);
  _last = _bus.writeSingle(_slave, JMCReg::HOME_METHOD, method);
  if (_last != JMC_OK) return _last;
  ctrl(CW_ENABLE);                          // ensure enabled, bit4 low
  return ctrl(CW_ENABLE | CW_SAMPLE_BIT);   // bit4 0->1 starts homing
}

JMCResult JMCMotor::setZeroPosition() { return setCurrentPosition(0); }

// Redefine the position counter without any movement. Homing method 0 declares
// the current residence the origin, and the home offset (0x607C) shifts the
// readout, so afterwards the current physical position reads <value>.
JMCResult JMCMotor::setCurrentPosition(long value) {
  setOperationMode(MODE_HOMING);
  _bus.writeSingle(_slave, JMCReg::HOME_METHOD, 0);   // method 0: no movement
  write32(_slave, JMCReg::HOME_OFFSET, (int32_t)value);
  ctrl(CW_ENABLE);
  return ctrl(CW_ENABLE | CW_SAMPLE_BIT);             // bit4 0->1 executes it
}

JMCResult JMCMotor::stop() { return ctrl(CW_HALT); }

JMCResult JMCMotor::quickStop(QuickStopMode mode) {
  _bus.writeSingle(_slave, JMCReg::QUICKSTOP, (uint16_t)mode);  // how to stop
  return ctrl(CW_QUICKSTOP);                                    // bit2: do it
}

JMCResult JMCMotor::eStop() { return quickStop(QUICK_STOP_IMMEDIATE); }

uint16_t JMCMotor::getStatusWord() {
  uint16_t s = 0;
  _last = _bus.readHolding(_slave, JMCReg::STATUS, &s, 1);
  return (_last == JMC_OK) ? s : 0;
}

JMCMotor::MotorStatus JMCMotor::getDetailedStatus() {
  MotorStatus st;
  uint16_t s = getStatusWord();
  st.valid = (_last == JMC_OK);
  st.readyToInit        = s & 0x0001;
  st.initComplete       = s & 0x0002;
  st.motorEnabled       = s & 0x0004;
  st.errorStatus        = s & 0x0008;
  st.driveWorking       = s & 0x0010;
  st.quickStopActive    = s & 0x0020;
  st.initState          = s & 0x0040;
  st.warning            = s & 0x0080;
  st.motorHalt          = s & 0x0100;
  st.motorRunning       = s & 0x0200;
  st.targetReached      = s & 0x0400;
  st.swMechanicalOrigin = s & 0x0800;
  st.homingComplete     = s & 0x1000;
  st.overPositionError  = s & 0x2000;
  st.cwLimit            = s & 0x4000;
  st.ccwLimit           = s & 0x8000;
  return st;
}

JMCMotor::ErrorFlags JMCMotor::getErrorFlags() {
  ErrorFlags e;
  uint16_t r = 0;
  _last = _bus.readHolding(_slave, JMCReg::ERROR, &r, 1);
  e.valid         = (_last == JMC_OK);
  e.raw           = r;
  e.generic       = r & 0x0001;
  e.current       = r & 0x0002;
  e.voltage       = r & 0x0004;
  e.temperature   = r & 0x0008;
  e.communication = r & 0x0010;
  e.overPosition  = r & 0x0020;
  e.phaseLoss     = r & 0x0080;
  return e;
}

bool JMCMotor::isRunning()       { uint16_t s = getStatusWord(); return (_last == JMC_OK) && (s & 0x0200); }
bool JMCMotor::isTargetReached() { uint16_t s = getStatusWord(); return (_last == JMC_OK) && (s & 0x0400); }
bool JMCMotor::hasFault()        { uint16_t s = getStatusWord(); return (_last == JMC_OK) && (s & 0x0008); }

JMCResult JMCMotor::broadcastPrepareMove(long steps, float velocity_rps, bool absolute) {
  _bus.writeSingle(0, JMCReg::MODE, MODE_POSITION);
  _moveBase = absolute ? CW_ENABLE : (CW_ENABLE | CW_RELATIVE);
  write32(0, JMCReg::TARGET_VEL, spdReg(fabs(velocity_rps)));
  write32(0, JMCReg::TARGET_POS, (int32_t)steps);
  return _last;
}

JMCResult JMCMotor::broadcastSetVelocity(float velocity_rps) {
  return write32(0, JMCReg::TARGET_VEL, spdReg(velocity_rps));
}

JMCResult JMCMotor::broadcastEnableMovement() {
  _last = _bus.writeSingle(0, JMCReg::CONTROL, _moveBase | CW_SAMPLE_BIT);
  return _last;
}

JMCResult JMCMotor::broadcastStartMovement() {
  _last = _bus.writeSingle(0, JMCReg::CONTROL, _moveBase);
  return _last;
}

JMCResult JMCMotor::broadcastStop() {
  _last = _bus.writeSingle(0, JMCReg::CONTROL, CW_HALT);
  return _last;
}

//==============================================================================
//  JMCController - status decode helpers (file-local)
//==============================================================================

// Drive state name per the manual's status-word state table.
static const char* driveStateName(uint16_t s) {
  switch (s & 0x000F) {
    case 0x0: return "Uninitialized";
    case 0x1: return "Initialized";
    case 0x3: return "Power on";
    case 0x7: return (s & 0x0020) ? "Device enable" : "Quick stop active";
    case 0xF: return "Fault alarm";
    case 0x8: return "Error state";
    default:  return "Unknown";
  }
}

static String flagsString(uint16_t s) {
  const char* names[16] = {
    "READY","INIT","ENABLED","FAULT","WORKING","QSTOP","INITSTATE","WARN",
    "HALT","RUN","REACHED","SWORIGIN","HOMED","POSERR","CWLIM","CCWLIM"
  };
  String out = "";
  for (int b = 0; b < 16; b++)
    if (s & (1 << b)) { if (out.length()) out += "|"; out += names[b]; }
  return out.length() ? out : String("NONE");
}

static String bitsString(uint16_t s) {
  String out = "";
  for (int b = 15; b >= 0; b--) {
    out += (s & (1 << b)) ? "1" : "0";
    if (b == 12 || b == 8 || b == 4) out += " ";
  }
  return out;
}

//==============================================================================
//  JMCController - lifecycle
//==============================================================================

JMCController::JMCController(HardwareSerial& rs485Serial) : _bus(rs485Serial) {
  // V3 network defaults; override with setNetwork().
  const byte defMac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  memcpy(_mac, defMac, 6);
  _localIp = IPAddress(192, 168, 1, 101);
  _pcIp    = IPAddress(192, 168, 1, 100);
  for (uint8_t i = 0; i < JMC_MAX_MOTORS; i++) {
    _m[i] = nullptr;
    _storedVel[i]    = _defVel;
    _homingOffset[i] = 0;
    _offlineTry[i]   = 0;
    _cache[i] = { false, 0, 0, false, false, false, false };
  }
}

void JMCController::setNetwork(const byte* mac,
                               IPAddress localIp, uint16_t localPort,
                               IPAddress pcIp,    uint16_t pcPort) {
  memcpy(_mac, mac, 6);
  _localIp   = localIp;
  _localPort = localPort;
  _pcIp      = pcIp;
  _pcPort    = pcPort;
}

void JMCController::setMotorCount(uint8_t count) {
  if (count >= 1 && count <= JMC_MAX_MOTORS) _count = count;
}

void JMCController::setMotionDefaults(float accel_rps_s, float decel_rps_s,
                                      float velocity_rps, float homingVel_rps) {
  _defAccel     = accel_rps_s;
  _defDecel     = decel_rps_s;
  _defVel       = velocity_rps;
  _defHomingVel = homingVel_rps;
}

void JMCController::setDriveP45(uint8_t p45) {
  // P45=0: speed register unit is 1 rps   -> 1 count per rps (factory default).
  // P45=1: speed register unit is 0.1 rps -> 10 counts per rps.
  _speedScale = (p45 == 0) ? 1.0f : 10.0f;
  for (uint8_t i = 0; i < JMC_MAX_MOTORS; i++)
    if (_m[i]) _m[i]->setSpeedScale(_speedScale);
}

uint8_t JMCController::begin(uint32_t baud) {
  _bus.beginControllino(baud);
  // Short per-attempt timeouts so a dead drive can't stall the loop long.
  _bus.setRetries(1);
  _bus.setResponseTimeout(30);

  Ethernet.begin(_mac, _localIp);
  _udp.begin(_localPort);

  uint8_t found = 0;
  for (uint8_t i = 0; i < _count; i++) {
    if (!_m[i]) _m[i] = new JMCMotor(_bus, i + 1);   // slave IDs 1..count
    _m[i]->setSpeedScale(_speedScale);               // match drive P45 unit
    _storedVel[i] = _defVel;

    // Fast presence probe first, so a bus configured for 32 motors with only
    // a few connected still boots in well under a second.
    _bus.setResponseTimeout(PROBE_TIMEOUT_MS);
    _bus.setRetries(0);
    bool present = _m[i]->isPresent();
    _bus.setResponseTimeout(30);
    _bus.setRetries(1);

    JMCResult r = JMC_ERR_TIMEOUT;
    if (present) {
      r = _m[i]->begin(_defAccel, _defDecel);
      _m[i]->setHomingVelocity(_defHomingVel);
      _m[i]->setHomingZeroVelocity(_defHomingVel);
    }
    _cache[i].online = (r == JMC_OK);
    if (r == JMC_OK) {
      found++;
      uint16_t s = _m[i]->getStatusWord();
      _cache[i].status = s;
      _cache[i].cw     = s & 0x4000;
      _cache[i].ccw    = s & 0x8000;
      _cache[i].fault  = s & 0x0008;
    }
    if (Serial) {
      Serial.print(F("JMC motor "));
      Serial.print(i + 1);
      Serial.print(F(" -> "));
      Serial.println(JMCBus::resultName(r));
    }
  }
  reply("READY:" + String(found) + "/" + String(_count));
  return found;
}

void JMCController::run() {
  handleUdp();       // always first: commands take priority
  serviceMonitor();  // one cheap status read per tick + event push
}

JMCMotor* JMCController::motor(uint8_t index) {
  return (index < _count) ? _m[index] : nullptr;
}

//==============================================================================
//  JMCController - comms & parsing helpers
//==============================================================================

void JMCController::reply(const String& msg) {
  _udp.beginPacket(_pcIp, _pcPort);
  _udp.write(msg.c_str());
  _udp.endPacket();
}

// Parse positional CSV ("1000,,2000"): value+active per motor slot.
int JMCController::parseFields(const String& csv, long* value, bool* active) {
  int count = 0;
  String work = csv + ",";
  int start = 0;
  for (uint8_t i = 0; i < _count; i++) {
    active[i] = false; value[i] = 0;
    int comma = work.indexOf(',', start);
    if (comma < 0) continue;
    String part = work.substring(start, comma); part.trim();
    if (part.length() > 0) { value[i] = atol(part.c_str()); active[i] = true; count++; }
    start = comma + 1;
  }
  return count;
}

// Parse an ID list ("1,3", "all", "" = all): sel[] per motor. Returns count.
int JMCController::parseIdList(const String& csv, bool* sel) {
  for (uint8_t i = 0; i < _count; i++) sel[i] = false;
  String s = csv; s.trim();
  if (s.length() == 0 || s.equalsIgnoreCase("all")) {
    for (uint8_t i = 0; i < _count; i++) sel[i] = true;
    return _count;
  }
  int count = 0;
  String work = s + ",";
  int start = 0;
  while (start < (int)work.length()) {
    int comma = work.indexOf(',', start);
    if (comma < 0) break;
    String part = work.substring(start, comma); part.trim();
    int id = part.toInt();
    if (id >= 1 && id <= _count && !sel[id - 1]) { sel[id - 1] = true; count++; }
    start = comma + 1;
  }
  return count;
}

//==============================================================================
//  JMCController - UDP command dispatch
//==============================================================================

void JMCController::handleUdp() {
  int packetSize = _udp.parsePacket();
  if (!packetSize) return;

  memset(_cmdBuf, 0, CMD_BUFFER_SIZE);
  _udp.read(_cmdBuf, CMD_BUFFER_SIZE - 1);

  String in = String(_cmdBuf);
  in.trim();
  if (!in.length()) return;

  String args;
  int colon = in.indexOf(':');
  String head = (colon < 0) ? in : in.substring(0, colon);
  args = (colon < 0) ? "" : in.substring(colon + 1);
  head.trim();

  // Exact-match multi-letter commands first, then single letters.
  if      (head == "PA") handlePositionAll(args);
  else if (head == "VA") handleVelocityAll(args);
  else if (head == "VS") handleVelocityStart();
  else if (head == "HV") handleHomingVelocity(args);
  else if (head == "HO") handleHomingOffset(args);
  else if (head == "SD") handleStatusDetail(args);
  else if (head == "BS") handleBrakeStatus();
  else if (head == "ZS") handleZeroStatus();
  else if (head == "ZO") handleZeroOffset(args);
  else if (head == "I")  handleInit();
  else if (head == "P")  handlePosition(args);
  else if (head == "V")  handleVelocity(args);
  else if (head == "S")  handleStatus(args);
  else if (head == "B")  handleBrake(args);
  else if (head == "L")  handleLoosen(args);
  else if (head == "Z")  handleZero(args);
  else if (head == "R")  handleReset(args);
  else if (head == "X") {
    reply("System restart initiated...");
    delay(50);
#if defined(__AVR__)
    asm volatile("jmp 0");            // soft reboot (AVR / Controllino)
#endif
  }
  else if (head.startsWith("H") && head.length() > 1 && isDigit(head[1]))
    handleHoming(head, args);
  else reply("INVALID COMMAND");
}

//==============================================================================
//  JMCController - background monitor (non-blocking, one motor per tick)
//==============================================================================

void JMCController::serviceMonitor() {
  uint32_t now = millis();
  if (now - _lastTick < MONITOR_TICK_MS) return;
  _lastTick = now;

  // Pick the next motor: online ones every sweep, offline ones only every
  // OFFLINE_RETRY_MS (so a dead drive can't eat bus time every cycle).
  int chosen = -1;
  for (uint8_t scan = 0; scan < _count; scan++) {
    _monIdx = (_monIdx + 1) % _count;
    if (_cache[_monIdx].online) { chosen = _monIdx; break; }
    if (now - _offlineTry[_monIdx] >= OFFLINE_RETRY_MS) { chosen = _monIdx; break; }
  }
  if (chosen < 0) return;

  MotorCache& c = _cache[chosen];

  uint16_t s;
  if (!c.online) {
    // Dead-drive probe: short timeout, no retries, so it can't stall the
    // sweep and delay limit detection on the live motors.
    _offlineTry[chosen] = now;
    _bus.setResponseTimeout(PROBE_TIMEOUT_MS);
    _bus.setRetries(0);
    s = _m[chosen]->getStatusWord();
    _bus.setResponseTimeout(30);
    _bus.setRetries(1);
  } else {
    s = _m[chosen]->getStatusWord();
  }
  bool ok = (_m[chosen]->lastResult() == JMC_OK);
  String mn = "M" + String(chosen + 1);

  if (!ok) {
    if (c.online && ++c.failCnt >= 2) {
      c.online = false;
      c.failCnt = 0;
      pushEvent("EVENT:OFFLINE:" + mn);
    }
    return;
  }

  c.failCnt = 0;
  if (!c.online) {
    c.online = true;
    // Drive (re)appeared - it may have been power-cycled, so re-initialise it
    // fully (enable sequence, accel/decel, homing speeds, stored HO offset).
    _m[chosen]->begin(_defAccel, _defDecel);
    _m[chosen]->setHomingVelocity(_defHomingVel);
    _m[chosen]->setHomingZeroVelocity(_defHomingVel);
    _m[chosen]->setHomingOffset(_homingOffset[chosen]);
    s = _m[chosen]->getStatusWord();
    pushEvent("EVENT:ONLINE:" + mn + " ST=0x" + String(s, HEX) + " (re-initialised)");
  }

  bool cw    = s & 0x4000;
  bool ccw   = s & 0x8000;
  bool fault = s & 0x0008;

  // Limit-switch edges: push immediately with position context.
  if (cw && !c.cw)
    pushEvent("EVENT:LIMIT:" + mn + ":CW POS=" + String(_m[chosen]->getPosition()) +
              " ST=0x" + String(s, HEX));
  if (!cw && c.cw)
    pushEvent("EVENT:LIMIT:" + mn + ":CW_CLEARED");
  if (ccw && !c.ccw)
    pushEvent("EVENT:LIMIT:" + mn + ":CCW POS=" + String(_m[chosen]->getPosition()) +
              " ST=0x" + String(s, HEX));
  if (!ccw && c.ccw)
    pushEvent("EVENT:LIMIT:" + mn + ":CCW_CLEARED");

  // Fault edges: decode the 0x1001 error register for the event.
  if (fault && !c.fault) {
    JMCMotor::ErrorFlags e = _m[chosen]->getErrorFlags();
    String detail = "";
    if (e.valid) {
      if (e.current)       detail += " CURRENT";
      if (e.voltage)       detail += " VOLTAGE";
      if (e.temperature)   detail += " TEMP";
      if (e.communication) detail += " COMM";
      if (e.overPosition)  detail += " OVERPOS";
      if (e.phaseLoss)     detail += " PHASE";
      if (e.generic && !detail.length()) detail = " GENERIC";
    }
    pushEvent("EVENT:FAULT:" + mn + " ST=0x" + String(s, HEX) +
              " ERR=0x" + String(e.raw, HEX) + detail);
  }
  if (!fault && c.fault)
    pushEvent("EVENT:FAULT_CLEARED:" + mn);

  c.cw = cw; c.ccw = ccw; c.fault = fault; c.status = s;
}

//==============================================================================
//  JMCController - command handlers (JMC_CONTROLLINO_V3)
//==============================================================================

// ---- P : absolute positions (reply "OK") ------------------------------------
void JMCController::handlePosition(const String& args) {
  if (!args.length()) { reply("ERR:P needs positions"); return; }

  // '?' slots -> selective position read (P:? = M1, P:,,? = M3, ...).
  if (args.indexOf('?') >= 0) { handlePositionQuery(args); return; }

  long target[JMC_MAX_MOTORS];
  bool active[JMC_MAX_MOTORS];
  int  n;

  if (args.indexOf(',') < 0) {            // single value => motor 1
    for (uint8_t i = 0; i < _count; i++) active[i] = false;
    target[0] = atol(args.c_str());
    active[0] = true;
    n = 1;
  } else {
    n = parseFields(args, target, active);
  }
  if (n == 0) { reply("ERR:no positions"); return; }

  // Synchronised start: prepare all (targets differ per motor), then launch.
  for (uint8_t i = 0; i < _count; i++)
    if (active[i]) _m[i]->preparePositionMove(target[i], _storedVel[i], true);

  if (n == (int)_count) {
    // Every motor is moving: launch with TWO broadcast frames (slave 0) so
    // all drives start in the same instant - no per-motor stagger even at 32.
    _m[0]->broadcastEnableMovement();
    delayMicroseconds(300);
    _m[0]->broadcastStartMovement();
  } else {
    // Partial selection: per-motor launch (broadcast would also retrigger
    // the motors that were NOT commanded).
    for (uint8_t i = 0; i < _count; i++)
      if (active[i]) _m[i]->enableMovement();
    delayMicroseconds(200);
    for (uint8_t i = 0; i < _count; i++)
      if (active[i]) _m[i]->startMovement();
  }

  reply("OK");
}

// Report positions of the selected motors: "M1=1500,M3=200" (+ WARN: on errors).
void JMCController::replyPositions(const bool* sel) {
  String out = "";
  bool anyError = false, first = true;
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    if (!first) out += ",";
    long p = _m[i]->getPosition();
    if (_m[i]->lastResult() != JMC_OK) { out += "M" + String(i + 1) + "=ERROR"; anyError = true; }
    else                                out += "M" + String(i + 1) + "=" + String(p);
    first = false;
  }
  reply(anyError ? "WARN: " + out : out);
}

// P:? queries are positional - only motors whose slot contains '?' are read:
//   P:?    -> motor 1        P:,,?  -> motor 3        P:?,?  -> motors 1 and 2
void JMCController::handlePositionQuery(const String& args) {
  bool sel[JMC_MAX_MOTORS];
  for (uint8_t i = 0; i < _count; i++) sel[i] = false;

  String work = args + ",";
  int start = 0, n = 0;
  for (uint8_t i = 0; i < _count; i++) {
    int comma = work.indexOf(',', start);
    if (comma < 0) break;
    String part = work.substring(start, comma); part.trim();
    if (part == "?") { sel[i] = true; n++; }
    start = comma + 1;
  }
  if (n == 0) { reply("ERR:no '?' fields in P query"); return; }
  replyPositions(sel);
}

// ---- PA : broadcast move / PA:? read-all -------------------------------------
void JMCController::handlePositionAll(const String& args) {
  if (!args.length()) { reply("ERR:PA needs position"); return; }

  if (args.indexOf('?') >= 0) {           // PA:? -> positions of ALL motors
    bool sel[JMC_MAX_MOTORS];
    for (uint8_t i = 0; i < _count; i++) sel[i] = true;
    replyPositions(sel);
    return;
  }

  long pos = atol(args.c_str());
  _m[0]->broadcastPrepareMove(pos, _storedVel[0], true);
  _m[0]->broadcastEnableMovement();
  delayMicroseconds(500);
  _m[0]->broadcastStartMovement();
  reply("OK");
}

// ---- S : status in many formats ------------------------------------------------
String JMCController::compactStatus(int i) {
  uint16_t s = _m[i]->getStatusWord();
  if (_m[i]->lastResult() != JMC_OK) return "M" + String(i + 1) + ":ERROR";
  String t1 = (s & 0x0008) ? "ERR" : ((s & 0x0004) ? "RUN" : "OFF");
  String t2 = (s & 0x0200) ? "MOV" : "STP";
  String t3 = (s & 0x0400) ? "TR"  : "NR";
  return "M" + String(i + 1) + ":" + t1 + "|" + t2 + "|" + t3 +
         "|POS=" + String(_m[i]->getPosition()) +
         "|VEL=" + String(_m[i]->getVelocity(), 1);
}

void JMCController::sendDetailStatus(int i) {
  uint16_t s = _m[i]->getStatusWord();
  String mn = "M" + String(i + 1);
  if (_m[i]->lastResult() != JMC_OK) {
    reply(mn + ": NO REPLY (" + String(JMCBus::resultName(_m[i]->lastResult())) + ")");
    return;
  }
  String out = mn + " (ID " + String(_m[i]->slaveId()) + ")\n";
  out += "  Status: 0x" + String(s, HEX) + " (" + driveStateName(s) + ")\n";
  out += "  Flags:  " + flagsString(s) + "\n";
  out += "  Bits:   " + bitsString(s) + "\n";
  out += "  Pos:    " + String(_m[i]->getPosition()) + "\n";
  out += "  Vel:    " + String(_m[i]->getVelocity(), 1) + " rps";
  if (s & 0x0008) {
    JMCMotor::ErrorFlags e = _m[i]->getErrorFlags();
    out += "\n  Error reg 0x1001 = 0x" + String(e.raw, HEX);
  }
  reply(out);
}

void JMCController::handleStatus(const String& args) {
  // "n[,FORMAT]" - n = 0 (all) or 1.._count.
  String a = args; a.trim();
  String fmt = "RAW";
  int sel = 0;
  int comma = a.indexOf(',');
  if (comma >= 0) {
    fmt = a.substring(comma + 1); fmt.trim(); fmt.toUpperCase();
    a = a.substring(0, comma); a.trim();
  }
  sel = a.toInt();
  if (sel < 0 || sel > _count) { reply("ERR:S motor 0.." + String(_count)); return; }

  int from = (sel == 0) ? 0 : sel - 1;
  int to   = (sel == 0) ? _count - 1 : sel - 1;

  if (fmt == "RAW" || fmt == "") {
    String out = "STATUS:";
    for (int i = from; i <= to; i++) {
      if (i > from) out += ",";
      uint16_t s = _m[i]->getStatusWord();
      if (_m[i]->lastResult() != JMC_OK) out += "M" + String(i + 1) + "=ERROR";
      else out += "M" + String(i + 1) + "=0x" + String(s, HEX);
    }
    reply(out);
  }
  else if (fmt == "COMPACT") {
    String out = "";
    for (int i = from; i <= to; i++) { if (i > from) out += ","; out += compactStatus(i); }
    reply(out);
  }
  else if (fmt == "STATE") {
    String out = "STATES:";
    for (int i = from; i <= to; i++) {
      if (i > from) out += ",";
      uint16_t s = _m[i]->getStatusWord();
      out += "M" + String(i + 1) + ":";
      out += (_m[i]->lastResult() != JMC_OK) ? "ERROR" : driveStateName(s);
    }
    reply(out);
  }
  else if (fmt == "BITS") {
    String out = "";
    for (int i = from; i <= to; i++) {
      uint16_t s = _m[i]->getStatusWord();
      if (i > from) out += "\n";
      out += "M" + String(i + 1) + ":";
      out += (_m[i]->lastResult() != JMC_OK) ? "ERROR" : bitsString(s);
    }
    reply(out);
  }
  else if (fmt == "FLAGS") {
    String out = "";
    for (int i = from; i <= to; i++) {
      uint16_t s = _m[i]->getStatusWord();
      if (i > from) out += "\n";
      out += "M" + String(i + 1) + ":";
      out += (_m[i]->lastResult() != JMC_OK) ? "ERROR" : flagsString(s);
    }
    reply(out);
  }
  else if (fmt == "DETAIL") {
    for (int i = from; i <= to; i++) sendDetailStatus(i);   // one packet per motor
  }
  else reply("ERR:S formats RAW|COMPACT|DETAIL|STATE|BITS|FLAGS");
}

// ---- SD : detailed status in plain language -------------------------------------
void JMCController::sendHumanStatus(int i) {
  uint16_t s = _m[i]->getStatusWord();
  String mn = "MOTOR " + String(i + 1);

  if (_m[i]->lastResult() != JMC_OK) {
    reply("=== " + mn + " === NOT RESPONDING (" +
          String(JMCBus::resultName(_m[i]->lastResult())) +
          ") - check power, slave ID, RS485 wiring");
    return;
  }

  long  pos = _m[i]->getPosition();
  float vel = _m[i]->getVelocity();

  String out = "=== " + mn + " (Slave ID " + String(_m[i]->slaveId()) +
               ") Status 0x" + String(s, HEX) + " ===\n";

  out += "State: " + String(driveStateName(s));
  if (s & 0x0040) out += " (entering initialization)";
  out += "\n";

  out += "Motor: ";
  out += (s & 0x0004) ? "ENABLED" : "DISABLED";
  if (s & 0x0200)      out += ", RUNNING";
  else                 out += ", stopped";
  if (s & 0x0100)      out += ", HALT active";
  if (s & 0x0020)      out += ", QUICK-STOP active";
  out += "\n";

  out += "Position: " + String(pos) + " steps (target ";
  out += (s & 0x0400) ? "REACHED" : "NOT reached";
  out += ")\n";
  out += "Velocity: " + String(vel, 1) + " rps\n";

  out += "Homing: ";
  out += (s & 0x1000) ? "complete" : "not done";
  out += " | Origin switch: ";
  out += (s & 0x0800) ? "ON" : "off";
  out += "\n";

  out += "Limits: CW ";
  out += (s & 0x4000) ? "*** ACTIVE ***" : "clear";
  out += " | CCW ";
  out += (s & 0x8000) ? "*** ACTIVE ***" : "clear";
  out += "\n";

  if (s & 0x2000) out += "OVER-POSITION ERROR (bit13)\n";
  if (s & 0x0080) out += "WARNING active (bit7)\n";

  if (s & 0x0008) {
    JMCMotor::ErrorFlags e = _m[i]->getErrorFlags();
    out += "FAULT:";
    bool named = false;
    if (e.valid) {
      if (e.current)       { out += " over-current";          named = true; }
      if (e.voltage)       { out += " voltage-error";         named = true; }
      if (e.temperature)   { out += " over-temperature";      named = true; }
      if (e.communication) { out += " communication-error";   named = true; }
      if (e.overPosition)  { out += " over-position-protect"; named = true; }
      if (e.phaseLoss)     { out += " motor-phase-loss";      named = true; }
      if (e.generic && !named) { out += " generic-error";     named = true; }
    }
    if (!named) out += " unknown";
    if (e.valid) out += " (0x1001=0x" + String(e.raw, HEX) + ")";
    out += " -> send R:" + String(i + 1) + " to reset\n";
  } else {
    out += "Health: OK, no faults\n";
  }

  reply(out);
}

void JMCController::handleStatusDetail(const String& args) {
  bool sel[JMC_MAX_MOTORS];
  String a = args; a.trim();

  if (!a.length()) {                        // SD -> all motors
    for (uint8_t i = 0; i < _count; i++) sel[i] = true;
  }
  else if (a.indexOf('?') >= 0) {           // SD:? / SD:,,? -> positional slots
    for (uint8_t i = 0; i < _count; i++) sel[i] = false;
    String work = a + ",";
    int start = 0, n = 0;
    for (uint8_t i = 0; i < _count; i++) {
      int comma = work.indexOf(',', start);
      if (comma < 0) break;
      String part = work.substring(start, comma); part.trim();
      if (part == "?") { sel[i] = true; n++; }
      start = comma + 1;
    }
    if (n == 0) { reply("ERR:no '?' fields in SD query"); return; }
  }
  else {                                    // SD:3 -> motor number also accepted
    int id = a.toInt();
    if (id < 1 || id > _count) { reply("ERR:SD use SD, SD:? or SD:1.." + String(_count)); return; }
    for (uint8_t i = 0; i < _count; i++) sel[i] = false;
    sel[id - 1] = true;
  }

  for (uint8_t i = 0; i < _count; i++)
    if (sel[i]) sendHumanStatus(i);         // one packet per motor
}

// ---- B : brakes -----------------------------------------------------------------
void JMCController::handleBrake(const String& args) {
  String a = args; a.trim();
  int type = a.toInt();                     // "" -> 0

  for (uint8_t i = 0; i < _count; i++) {
    if      (type == 1) _m[i]->quickStop(JMCMotor::QUICK_STOP_DECEL);
    else if (type == 2) _m[i]->quickStop(JMCMotor::QUICK_STOP_FAST);
    else if (type == 3) _m[i]->quickStop(JMCMotor::QUICK_STOP_IMMEDIATE);
    else                _m[i]->stop();
  }
  if      (type == 1) reply("Deceleration brake applied to all motors");
  else if (type == 2) reply("Fast brake applied to all motors");
  else if (type == 3) reply("Immediate brake applied to all motors");
  else                reply("All motors stopped with deceleration");
}

// ---- L : loosen brakes -------------------------------------------------------------
void JMCController::handleLoosen(const String& args) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseIdList(args, sel);
  if (n == 0) { reply("ERR:L ids 1.." + String(_count) + " or all"); return; }

  String done = "";
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    _m[i]->loosenBrake();
    if (done.length()) done += ",";
    done += String(i + 1);
  }
  reply((n == (int)_count) ? "All motor brakes loosened"
                           : "Motor " + done + " brake loosened");
}

// ---- BS : brake status --------------------------------------------------------------
void JMCController::handleBrakeStatus() {
  String out = "BRAKE STATUS: ";
  int braked = 0, freed = 0, errors = 0;
  for (uint8_t i = 0; i < _count; i++) {
    if (i) out += ",";
    uint16_t s = _m[i]->getStatusWord();
    if (_m[i]->lastResult() != JMC_OK) { out += "M" + String(i + 1) + ":ERROR"; errors++; continue; }
    bool brakeEngaged = (s & 0x0100) || !(s & 0x0004);   // halted or not enabled
    bool running      = s & 0x0200;
    out += "M" + String(i + 1) + ":" + (brakeEngaged ? "BRAKE" : "FREE") + "/" +
           (running ? "RUN" : "STOP");
    if (brakeEngaged) braked++; else freed++;
  }
  out += " | SUMMARY: ";
  if (errors == (int)_count)  out += "ERROR";
  else if (braked && !freed)  out += "BRAKED";
  else if (freed && !braked)  out += "FREE";
  else                        out += "MIXED";
  reply(out);
}

// ---- Z : current position becomes 0 ---------------------------------------------------
void JMCController::handleZero(const String& args) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseIdList(args, sel);
  if (n == 0) { reply("ERR:Z ids 1.." + String(_count) + " or all"); return; }

  String done = "";
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    _m[i]->setZeroPosition();
    delay(20);
    _m[i]->setOperationMode(JMCMotor::MODE_POSITION);
    _cache[i].zeroSet = true;
    if (done.length()) done += ",";
    done += String(i + 1);
  }
  reply((n == (int)_count) ? "All motors set to zero position"
                           : "Motor " + done + " set to zero position");
}

// ---- ZO : redefine current position (no movement) --------------------------------------
// ZO:10000,1000  ->  motor 1's CURRENT position now reads 10000, motor 2's 1000.
void JMCController::handleZeroOffset(const String& args) {
  if (!args.length()) { reply("ERR:ZO needs values"); return; }

  long value[JMC_MAX_MOTORS];
  bool active[JMC_MAX_MOTORS];
  int  n;

  if (args.indexOf(',') < 0) {              // single value => motor 1
    for (uint8_t i = 0; i < _count; i++) active[i] = false;
    value[0] = atol(args.c_str());
    active[0] = true;
    n = 1;
  } else {
    n = parseFields(args, value, active);
  }
  if (n == 0) { reply("ERR:no values"); return; }

  String out = "ZO: ";
  bool first = true;
  for (uint8_t i = 0; i < _count; i++) {
    if (!active[i]) continue;
    _m[i]->setCurrentPosition(value[i]);
    delay(20);                              // let homing method 0 latch
    _m[i]->setOperationMode(JMCMotor::MODE_POSITION);
    _cache[i].zeroSet = true;
    if (!first) out += ",";
    out += "M" + String(i + 1) + "=" + String(value[i]);
    first = false;
  }
  reply(out + " (current position redefined)");
}

// ---- ZS : zero status ---------------------------------------------------------------------
void JMCController::handleZeroStatus() {
  String out = "ZERO STATUS: ";
  bool allSet = true;
  for (uint8_t i = 0; i < _count; i++) {
    if (i) out += ",";
    long p = _m[i]->getPosition();
    bool ok = (_m[i]->lastResult() == JMC_OK);
    if (!ok) { out += "M" + String(i + 1) + ":ERROR"; allSet = false; continue; }
    out += "M" + String(i + 1) + ":" + (_cache[i].zeroSet ? "ZERO" : "NOT_SET") +
           "(" + String(p) + ")";
    if (!_cache[i].zeroSet) allSet = false;
  }
  out += allSet ? " | ALL_ZERO_SET" : " | ZERO_NOT_SET";
  reply(out);
}

// ---- R : reset ------------------------------------------------------------------------------
void JMCController::handleReset(const String& args) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseIdList(args, sel);
  if (n == 0) { reply("ERR:R ids 1.." + String(_count) + " or all"); return; }

  String done = "";
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    _m[i]->alarmReset();
    JMCResult r = _m[i]->begin(_defAccel, _defDecel);
    _cache[i].online = (r == JMC_OK);
    _cache[i].fault  = false;
    if (done.length()) done += ",";
    done += String(i + 1) + (r == JMC_OK ? "" : "(FAIL)");
  }
  reply((n == (int)_count) ? "All motors reset complete"
                           : "Motor " + done + " reset complete");
}

// ---- I : detect + initialise ---------------------------------------------------------------
void JMCController::handleInit() {
  String out = "MOTOR INITIALIZATION:\n";
  int found = 0;
  for (uint8_t i = 0; i < _count; i++) {
    out += "Motor " + String(i + 1) + " (Slave ID " + String(_m[i]->slaveId()) + "): ";
    JMCResult r = _m[i]->begin(_defAccel, _defDecel);
    _cache[i].online = (r == JMC_OK);
    if (r == JMC_OK) {
      found++;
      uint16_t s = _m[i]->getStatusWord();
      out += "DETECTED - Initialized in Position mode\n";
      out += "  Status: 0x" + String(s, HEX) + " (" + driveStateName(s) + ")\n";
      out += "  Position: " + String(_m[i]->getPosition()) + "\n";
    } else {
      out += "NOT DETECTED (" + String(JMCBus::resultName(r)) + ")\n";
      out += "  Check: Power, Address, RS485 wiring\n";
    }
  }
  out += "Total motors detected: " + String(found) + " of " + String(_count);
  if (found == (int)_count) out += "\nSUCCESS: All expected motors detected and initialized!";
  reply(out);
}

//==============================================================================
//  JMCController - extension commands
//==============================================================================

void JMCController::handleVelocity(const String& args) {
  if (!args.length()) { reply("ERR:V needs argument"); return; }
  String s = args;

  if (s.indexOf('?') >= 0) {
    String out = "V: ";
    for (uint8_t i = 0; i < _count; i++) { if (i) out += ","; out += String(_storedVel[i]); }
    reply(out);
    return;
  }

  if (s.indexOf(',') < 0) {
    _storedVel[0] = s.toFloat();
    reply("Set Motor 1 V: " + String(_storedVel[0]));
    return;
  }

  String work = s + ","; int start = 0; String out = "Set V: ";
  for (uint8_t i = 0; i < _count; i++) {
    if (i) out += ",";
    int comma = work.indexOf(',', start);
    if (comma < 0) continue;
    String part = work.substring(start, comma); part.trim();
    if (part.length()) { _storedVel[i] = part.toFloat(); out += String(_storedVel[i]); }
    start = comma + 1;
  }
  reply(out);
}

void JMCController::handleVelocityAll(const String& args) {
  String s = args; s.trim();
  if (!s.length()) { reply("ERR:VA needs velocity"); return; }
  if (s.indexOf('?') >= 0) { handleVelocity("?"); return; }
  float v = s.toFloat();
  if (v < 0 || v > 1000.0f) { reply("ERR:VA range 0-1000 rps"); return; }
  for (uint8_t i = 0; i < _count; i++) _storedVel[i] = v;
  _m[0]->broadcastSetVelocity(v);
  reply("VA: Set all motors velocity to " + String(v) + " rps (BROADCAST)");
}

void JMCController::handleVelocityStart() {
  // Per-motor speeds, then a broadcast launch so all motors start together.
  for (uint8_t i = 0; i < _count; i++) _m[i]->prepareVelocityMode(_storedVel[i]);
  _m[0]->broadcastEnableMovement();
  delayMicroseconds(300);
  _m[0]->broadcastStartMovement();
  reply("VS: velocity mode started (stop with B)");
}

void JMCController::handleHomingVelocity(const String& args) {
  if (!args.length()) { reply("ERR:HV needs argument"); return; }
  String s = args;
  if (s.indexOf(',') < 0) {
    float v = s.toFloat();
    _m[0]->setHomingVelocity(v);        // 0x6099 search speed
    _m[0]->setHomingZeroVelocity(v);    // 0x609B zero-find speed
    reply("Set Motor 1 HV: " + String(v));
    return;
  }
  String work = s + ","; int start = 0; String out = "Set HV: ";
  for (uint8_t i = 0; i < _count; i++) {
    if (i) out += ",";
    int comma = work.indexOf(',', start);
    if (comma < 0) continue;
    String part = work.substring(start, comma); part.trim();
    if (part.length()) {
      float v = part.toFloat();
      _m[i]->setHomingVelocity(v);
      _m[i]->setHomingZeroVelocity(v);
      out += part;
    }
    start = comma + 1;
  }
  reply(out);
}

// ---- HO : homing offset --------------------------------------------------------
// Steps the motor moves AFTER the origin sensor triggers, before declaring
// zero (drive register 0x607C). Sign sets the direction. Positional like P:
//   HO:1000        motor 1 -> 1000 steps past the sensor
//   HO:1000,2000   motors 1+2
//   HO:,,500       motor 3 only
//   HO:?           read the stored offsets
// The offset is stored AND written to the drive, and re-applied automatically
// every time an H<m> homing command runs.
void JMCController::handleHomingOffset(const String& args) {
  if (!args.length()) { reply("ERR:HO needs steps (HO:1000 or HO:?)"); return; }
  String s = args;

  if (s.indexOf('?') >= 0) {                 // HO:? -> report stored offsets
    String out = "HO: ";
    for (uint8_t i = 0; i < _count; i++) {
      if (i) out += ",";
      out += "M" + String(i + 1) + "=" + String(_homingOffset[i]);
    }
    reply(out);
    return;
  }

  long value[JMC_MAX_MOTORS];
  bool active[JMC_MAX_MOTORS];
  int  n;

  if (s.indexOf(',') < 0) {                  // single value => motor 1
    for (uint8_t i = 0; i < _count; i++) active[i] = false;
    value[0] = atol(s.c_str());
    active[0] = true;
    n = 1;
  } else {
    n = parseFields(s, value, active);
  }
  if (n == 0) { reply("ERR:no offset values"); return; }

  String out = "HO: ";
  bool first = true;
  for (uint8_t i = 0; i < _count; i++) {
    if (!active[i]) continue;
    _homingOffset[i] = value[i];
    _m[i]->setHomingOffset(value[i]);
    if (!first) out += ",";
    out += "M" + String(i + 1) + "=" + String(value[i]);
    first = false;
  }
  reply(out + " steps after sensor (used at next homing)");
}

void JMCController::handleHoming(const String& head, const String& args) {
  int method = head.substring(1).toInt();
  if (method < 1 || method > 12) { reply("ERR:homing method H1..H12"); return; }

  if (!args.length()) {
    for (uint8_t i = 0; i < _count; i++) {
      _m[i]->setHomingOffset(_homingOffset[i]);   // re-apply stored HO offset
      _m[i]->performHoming(method);
    }
    reply("Homing method " + String(method) + " started for all motors");
    return;
  }
  long v[JMC_MAX_MOTORS]; bool active[JMC_MAX_MOTORS];
  parseFields(args, v, active);
  String list = "";
  for (uint8_t i = 0; i < _count; i++) {
    if (!active[i]) continue;
    _m[i]->setHomingOffset(_homingOffset[i]);     // re-apply stored HO offset
    _m[i]->performHoming(method);
    if (list.length()) list += ",";
    list += String(i + 1);
  }
  if (list.length()) reply("Homing method " + String(method) + " started for M" + list);
  else               reply("ERR:no motors selected");
}

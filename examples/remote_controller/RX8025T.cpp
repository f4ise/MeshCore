#include "RX8025T.h"

#ifndef likely
#ifdef __GNUC__
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#else
#define likely(x) (x)
#define unlikely(x) (x)
#endif
#endif

// RAII class for data transferring to/from DS1302
namespace {
  class TransferHelper {
    uint8_t _ce, _sck;

    static constexpr uint8_t CE_TO_SCK_SETUP = 4;
    static constexpr uint8_t CE_INACTIVE_TIME = 4;

  public:
    TransferHelper(uint8_t ce, uint8_t sck) : _ce {ce}, _sck {sck} {
      digitalWrite(_sck, LOW);
      digitalWrite(_ce, HIGH);
      delayMicroseconds(CE_TO_SCK_SETUP);
    }

    ~TransferHelper() {
      digitalWrite(_ce, LOW);
      delayMicroseconds(CE_INACTIVE_TIME);
    }
  };
} // namespace

static constexpr uint8_t bcd2bin(uint8_t val) {
  return val - 6 * (val >> 4);
}

static constexpr uint8_t bin2bcd(uint8_t val) {
  return val + 6 * (val / 10);
}

static void i2c_rtc_write(TwoWire &wire, uint8_t dev, uint8_t addr, uint8_t val) {
  wire.beginTransmission(dev);
  wire.write(addr);
  wire.write(val);
  wire.endTransmission();
}

static uint8_t i2c_rtc_read(TwoWire &wire, uint8_t dev, uint8_t addr) {
  wire.beginTransmission(dev);
  wire.write(addr);
  wire.endTransmission();

  wire.requestFrom(dev, uint8_t {1});
  return wire.read();
}

#define MASK_BOOL_REG_BITS(addr, bits, en)        \
  do {                                            \
    uint8_t __mask = (en) ? (bits) : 0;           \
    uint8_t __val = readReg(addr);                \
    if ((__val & (bits)) != __mask) {             \
      writeReg(addr, (__val & ~(bits)) | __mask); \
    }                                             \
  } while (0)

RX8025T::RX8025T(TwoWire &wire) : _wire {wire} {}

bool RX8025T::setup() {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_FLAG);
  if (_wire.endTransmission() != 0) {
    return false;
  }

  _wire.requestFrom(ADDRESS, uint8_t {1});
  uint8_t flag = _wire.read();
  _wire.endTransmission();

  // check VLF
  if (flag & 0x02) {
    static constexpr uint8_t PROGMEM init_regs[] {
        REG_SEC,
        0x00, // SEC
        0x00, // MIN
        0x00, // HOUR
        0x40, // WEEK
        0x01, // DAY
        0x01, // MONTH
        0x00, // YEAR
        0x00, // RAM
        0x00, // AL_MIN
        0x00, // AL_HOUR
        0x00, // AL_WK_D
        0x00, // TIM0
        0x00, // TIM1
        0x00, // EXT
        0x00, // FLAG
        0x40, // CTRL
    };

    // reinit all
    _wire.beginTransmission(ADDRESS);
    for (uint8_t i = 0; i < sizeof(init_regs); ++i) {
      _wire.write(pgm_read_byte(init_regs + i));
    }
    _wire.endTransmission();
  }

  return true;
}

uint8_t RX8025T::readReg(RegAddr addr) {
  return i2c_rtc_read(_wire, ADDRESS, addr);
}

void RX8025T::writeReg(RegAddr addr, uint8_t val) {
  i2c_rtc_write(_wire, ADDRESS, addr, val);
}

void RX8025T::getTime(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_SEC);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t {7});
  timeptr->tm_sec = bcd2bin(_wire.read() & 0x7f);
  timeptr->tm_min = bcd2bin(_wire.read() & 0x7f);
  timeptr->tm_hour = bcd2bin(_wire.read() & 0x3f);
  timeptr->tm_wday = __builtin_ctz(_wire.read());
  timeptr->tm_mday = bcd2bin(_wire.read() & 0x3f);
  timeptr->tm_mon = bcd2bin(_wire.read() & 0x1f) - 1;
  timeptr->tm_year = bcd2bin(_wire.read()) + 100;
}

void RX8025T::setTime(const tm *t) {
  const uint8_t write_buf[] {
      REG_SEC,
      bin2bcd(t->tm_sec),
      bin2bcd(t->tm_min),
      bin2bcd(t->tm_hour),
      static_cast<uint8_t>(1U << t->tm_wday),
      bin2bcd(t->tm_mday),
      bin2bcd(t->tm_mon + 1),
      bin2bcd(t->tm_year - 100),
  };

  _wire.beginTransmission(ADDRESS);
  _wire.write(write_buf, sizeof(write_buf));
  _wire.endTransmission();
}

bool RX8025T::isRunning() {
  return (readReg(REG_CTRL) & 0x01) == 0;
}

void RX8025T::setRunning(bool running) {
  MASK_BOOL_REG_BITS(REG_CTRL, 0x01, !running);
}

RX8025T::TempCompIntv RX8025T::getTempCompInterval() {
  return static_cast<TempCompIntv>(readReg(REG_CTRL) & 0xc0);
}

void RX8025T::setTempCompIntv(TempCompIntv interval) {
  writeReg(REG_CTRL, (readReg(REG_CTRL) & 0x3f) | interval);
}

uint8_t RX8025T::getRAM() {
  return readReg(REG_RAM);
}

void RX8025T::setRAM(uint8_t val) {
  writeReg(REG_RAM, val);
}

RX8025T::TimerFreq RX8025T::getTimerFreq() {
  uint8_t ext = readReg(REG_EXT);

  if ((ext & 0x10) == 0) {
    // TE bit is 0
    return TF_OFF;
  } else {
    return static_cast<TimerFreq>(ext & 0x03);
  }
}

void RX8025T::setTimerFreq(TimerFreq freq) {
  if (freq == TF_OFF) {
    MASK_BOOL_REG_BITS(REG_EXT, 0x10, 0);
  } else {
    writeReg(REG_EXT, (readReg(REG_EXT) & 0xfc) | freq);
  }
}

bool RX8025T::isTimerIntrEnabled() {
  return (readReg(REG_CTRL) & 0x10) != 0;
}

void RX8025T::setTimerIntrEnabled(bool enabled) {
  MASK_BOOL_REG_BITS(REG_CTRL, 0x10, enabled);
}

bool RX8025T::getTimerFlag() {
  return (readReg(REG_FLAG) & 0x10) != 0;
}

void RX8025T::clearTimerFlag() {
  MASK_BOOL_REG_BITS(REG_FLAG, 0x10, 0);
}

RX8025T::FOUTFreq RX8025T::getFOUT() {
  uint8_t freq = readReg(REG_CTRL) & 0x0c;
  if (freq == 0x0c) {
    // 2'b11 is also 32768Hz
    return FOUT_32768HZ;
  } else {
    return static_cast<FOUTFreq>(freq);
  }
}

void RX8025T::setFOUT(FOUTFreq freq) {
  writeReg(REG_CTRL, (readReg(REG_CTRL) & 0xf3) | freq);
}

bool RX8025T::getVLF() {
  return (readReg(REG_FLAG) & 0x02) != 0;
}

void RX8025T::clearVLF() {
  MASK_BOOL_REG_BITS(REG_FLAG, 0x02, 0);
}

bool RX8025T::getVDET() {
  return (readReg(REG_FLAG) & 0x01) != 0;
}

void RX8025T::clearVDET() {
  MASK_BOOL_REG_BITS(REG_FLAG, 0x01, 0);
}

bool RX8025T::getUpdateFlag() {
  return (readReg(REG_FLAG) & 0x20) != 0;
}

void RX8025T::clearUpdateFlag() {
  MASK_BOOL_REG_BITS(REG_FLAG, 0x20, 0);
}

bool RX8025T::getUSEL() {
  return (readReg(REG_EXT) & 0x20) != 0;
}

void RX8025T::setUSEL(bool usel) {
  MASK_BOOL_REG_BITS(REG_EXT, 0x20, usel);
}

uint16_t RX8025T::getTimer() {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_TIM0);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t {2});
  uint16_t val = _wire.read();
  val |= _wire.read() << 8;
  return val;
}

void RX8025T::setTimer(uint16_t val) {
  const uint8_t write_buf[] {
      REG_TIM0,
      uint8_t(val & 0xff),
      uint8_t(val >> 8),
  };

  _wire.beginTransmission(ADDRESS);
  _wire.write(write_buf, sizeof(write_buf));
  _wire.endTransmission();
}

void RX8025T::getAlarm(tm *timeptr) {
  _wire.beginTransmission(ADDRESS);
  _wire.write(REG_AL_MIN);
  _wire.endTransmission();

  _wire.requestFrom(ADDRESS, uint8_t {6});
  uint8_t min = _wire.read();
  uint8_t hour = _wire.read();
  uint8_t day = _wire.read();
  _wire.read(); // Timer/Counter 0
  _wire.read(); // Timer/Counter 1
  uint8_t ext = _wire.read();

  bool wada = ext & 0x40;

  timeptr->tm_min = (min & 0x80) ? -1 : bcd2bin(min & 0x7f);
  timeptr->tm_hour = (hour & 0x80) ? -1 : bcd2bin(hour & 0x3f);

  if (day & 0x80) {
    timeptr->tm_wday = -1;
    timeptr->tm_mday = -1;
  } else if (wada) {
    timeptr->tm_wday = -1;
    timeptr->tm_mday = bcd2bin(day & 0x3f);
  } else {
    timeptr->tm_wday = day;
    timeptr->tm_mday = -1;
  }
}

void RX8025T::setAlarm(const tm *timeptr) {
  uint8_t min = (timeptr->tm_min == -1) ? 0x80 : bin2bcd(timeptr->tm_min);
  uint8_t hour = (timeptr->tm_hour == -1) ? 0x80 : bin2bcd(timeptr->tm_hour);
  int8_t day = timeptr->tm_mday;
  int8_t wday = timeptr->tm_wday;
  bool wada = false;

  if ((day == -1 && wday == -1) || (day != -1 && wday != -1)) {
    // does not match DAY/WEEK
    day = static_cast<int8_t>(0x80);
  } else if (day != -1) {
    // sets DAY as target of alarm function
    day = bin2bcd(day & 0x3f);
    wada = true;
  } else {
    // sets WEEK as target of alarm function
    day = wday & 0x7f;
  }

  const uint8_t write_buf[] {
      REG_AL_MIN,
      min,
      hour,
      static_cast<uint8_t>(day),
  };

  _wire.beginTransmission(ADDRESS);
  _wire.write(write_buf, sizeof(write_buf));
  _wire.endTransmission();

  if ((day & 0x80) == 0) {
    MASK_BOOL_REG_BITS(REG_EXT, 0x40, wada);
  }
}

bool RX8025T::isAlarmIntrEnabled() {
  return (readReg(REG_CTRL) & 0x08) != 0;
}

void RX8025T::setAlarmIntrEnabled(bool enabled) {
  MASK_BOOL_REG_BITS(REG_CTRL, 0x08, enabled);
}

bool RX8025T::getAlarmFlag() {
  return (readReg(REG_FLAG) & 0x08) != 0;
}

void RX8025T::clearAlarmFlag() {
  MASK_BOOL_REG_BITS(REG_FLAG, 0x08, 0);
}
#ifndef __RTCLIB_H__
#define __RTCLIB_H__

#include <stdint.h>
#include <time.h>
#include <Arduino.h>
#include <Wire.h>

namespace __rtclib_details {
  template <typename RTC>
  class RAMRef {
    RTC *_rtc;
    uint8_t _index;

  public:
    RAMRef(RTC *rtc, uint8_t index) : _rtc {rtc}, _index {index} {}

    operator uint8_t() { return _rtc->readRAM(_index); }
    operator int() { return _rtc->readRAM(_index); }

    RAMRef &operator=(uint8_t val) {
      _rtc->writeRAM(_index, val);
      return *this;
    }
    RAMRef &operator+=(uint8_t in) { return *this = *this + in; }
    RAMRef &operator-=(uint8_t in) { return *this = *this - in; }
    RAMRef &operator*=(uint8_t in) { return *this = *this * in; }
    RAMRef &operator/=(uint8_t in) { return *this = *this / in; }
    RAMRef &operator^=(uint8_t in) { return *this = *this ^ in; }
    RAMRef &operator%=(uint8_t in) { return *this = *this % in; }
    RAMRef &operator&=(uint8_t in) { return *this = *this & in; }
    RAMRef &operator|=(uint8_t in) { return *this = *this | in; }
    RAMRef &operator<<=(uint8_t in) { return *this = *this << in; }
    RAMRef &operator>>=(uint8_t in) { return *this = *this >> in; }

    // Prefix increment
    RAMRef &operator++() { return *this += 1; }
    // Prefix decrement
    RAMRef &operator--() { return *this -= 1; }

    // Postfix increment
    uint8_t operator++(int) {
      uint8_t ret = *this;
      ++*this;
      return ret;
    }

    // Postfix decrement
    uint8_t operator--(int) {
      uint8_t ret = *this;
      --*this;
      return ret;
    }
  };

  template <typename RTC>
  class RAMPtr {
    RTC *_rtc;
    uint8_t _index;

  public:
    RAMPtr(RTC *rtc, uint8_t index) : _rtc {rtc}, _index {index} {}

    explicit operator uint8_t() const { return _index; }
    explicit operator int() const { return _index; }

    bool operator==(const RAMPtr &other) const { return _index == other._index; }
    bool operator!=(const RAMPtr &other) const { return _index != other._index; }
    bool operator<(const RAMPtr &other) const { return _index < other._index; }
    bool operator<=(const RAMPtr &other) const { return _index <= other._index; }
    bool operator>(const RAMPtr &other) const { return _index > other._index; }
    bool operator>=(const RAMPtr &other) const { return _index >= other._index; }

    RAMPtr &operator=(uint8_t index) {
      _index = index;
      return *this;
    }
    RAMPtr &operator+=(int8_t off) {
      _index += off;
      return *this;
    }
    RAMPtr &operator-=(int8_t off) {
      _index -= off;
      return *this;
    }
    RAMPtr operator+(int8_t off) const { return RAMPtr {_rtc, _index + off}; }
    friend RAMPtr operator+(int8_t off, const RAMPtr &other) { return other + off; }
    RAMPtr operator-(int8_t off) const { return RAMPtr {_rtc, _index - off}; }
    int8_t operator-(const RAMPtr &other) const {
      return static_cast<int8_t>(_index) - static_cast<int8_t>(other._index);
    }

    RAMRef<RTC> operator*() { return RAMRef<RTC> {_rtc, _index}; }

    RAMPtr operator++(int) { return RAMPtr {_rtc, _index++}; }
    RAMPtr operator--(int) { return RAMPtr {_rtc, _index--}; }
    RAMPtr &operator++() {
      ++_index;
      return *this;
    }
    RAMPtr &operator--() {
      --_index;
      return *this;
    }
  };
} // namespace __rtclib_details

// RX8025T: only basic timekeeping functions are stable
// other functions are subject to change
class RX8025T {
  using RAMRef = __rtclib_details::RAMRef<RX8025T>;
  using RAMPtr = __rtclib_details::RAMPtr<RX8025T>;

  friend class __rtclib_details::RAMRef<RX8025T>;
  uint8_t readRAM(uint8_t) { return getRAM(); }
  void writeRAM(uint8_t, uint8_t val) { setRAM(val); }

  TwoWire &_wire;

public:
  enum RegAddr : uint8_t {
    REG_SEC = 0x00,
    REG_MIN = 0x01,
    REG_HOUR = 0x02,
    REG_WDAY = 0x03,
    REG_MDAY = 0x04,
    REG_MON = 0x05,
    REG_YEAR = 0x06,
    REG_RAM = 0x07,
    REG_AL_MIN = 0x08,
    REG_AL_HOUR = 0x09,
    REG_AL_DAY = 0x0a,
    REG_TIM0 = 0x0b,
    REG_TIM1 = 0x0c,
    REG_EXT = 0x0d,
    REG_FLAG = 0x0e,
    REG_CTRL = 0x0f,
  };

  enum TempCompIntv : uint8_t {
    TC_0_5S = 0x00,
    TC_2S = 0x40,
    TC_10S = 0x80,
    TC_30S = 0xc0,
  };

  enum AlarmDay : uint8_t {
    AL_SUN = 0x81,
    AL_MON = 0x82,
    AL_TUE = 0x84,
    AL_WED = 0x88,
    AL_THU = 0x90,
    AL_FRI = 0xa0,
    AL_SAT = 0xc0,
    AL_EVERY_DAY = 0xff,
  };

  enum TimerFreq : uint8_t {
    TF_4096HZ = 0x00,
    TF_64HZ = 0x01,
    TF_1HZ = 0x02,
    TF_MINUTE = 0x03,
    TF_OFF = 0xff,
  };

  enum FOUTFreq : uint8_t {
    FOUT_32768HZ = 0x00,
    FOUT_1024HZ = 0x04,
    FOUT_1HZ = 0x08,
  };

  static constexpr uint8_t ADDRESS = 0x32;
  static constexpr uint8_t RAM_SIZE = 1;

  explicit RX8025T(TwoWire &wire = Wire);
  RX8025T(const RX8025T &) = delete;
  RX8025T &operator=(const RX8025T &) = delete;

  bool setup();

  uint8_t readReg(RegAddr addr);
  void writeReg(RegAddr addr, uint8_t val);

  void getTime(tm *timeptr);
  void setTime(const tm *timeptr);

  bool isRunning();
  void setRunning(bool running);

  TempCompIntv getTempCompInterval();
  void setTempCompIntv(TempCompIntv interval);

  uint8_t getRAM();
  void setRAM(uint8_t val);

  uint16_t getTimer();
  void setTimer(uint16_t val);
  TimerFreq getTimerFreq();
  void setTimerFreq(TimerFreq freq);
  bool isTimerIntrEnabled();
  void setTimerIntrEnabled(bool enabled);
  bool getTimerFlag();
  void clearTimerFlag();

  FOUTFreq getFOUT();
  void setFOUT(FOUTFreq freq);

  bool getVLF();
  void clearVLF();
  bool getVDET();
  void clearVDET();
  bool getUpdateFlag();
  void clearUpdateFlag();
  bool getUSEL();
  void setUSEL(bool usel);

  // alarm api is subject to change
  void getAlarm(tm *timeptr);
  void setAlarm(const tm *timeptr);
  bool isAlarmIntrEnabled();
  void setAlarmIntrEnabled(bool enabled);
  bool getAlarmFlag();
  void clearAlarmFlag();

  RAMPtr begin() { return RAMPtr {this, 0}; }
  RAMPtr end() { return RAMPtr {this, RAM_SIZE}; }
  RAMRef operator[](uint8_t index) { return RAMRef {this, index}; }
};

#endif

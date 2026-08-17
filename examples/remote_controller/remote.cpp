//
// Created by Wizado F4ISE on 11/08/2026.
//
#include "remote.h"

#include "helpers/RTC_RX8025T.h"
#include "config.h"
#include "d578.h"
#include "dtmf.h"

#include <M24M01.h>
#include <RTClib.h>
#include <TCA9548.h>
#include <TCA9555.h>
#include <Wire.h>

M24M01  eeprom(Wire, 0x50);
PCA9546 muxi2c(0x70, &Wire1);
TCA9555 pioRC(0x20, &Wire1);
RTC_RX8025T rtc(Wire);

// Data RTC
const bool DO_SET_TIME = true;
const int SET_YEAR = 2026;  // full year, e.g. 2026
const int SET_MONTH = 1;    // 1-12
const int SET_DAY = 1;      // 1-31
const int SET_DAY_OF_WEEK = 0;
const int SET_HOUR = 0;    // 0-23
const int SET_MIN = 0;      // 0-59
const int SET_SEC = 0;      // 0-59

// Polling Interrupt
bool irqNSTQ = false;
bool irqRTC = false;

//--------------------------------------------------------------------------
//-  SETUP REMOTE CONTROLLER
//--------------------------------------------------------------------------
void remoteInit(void) {
  Serial.print(F("Remote Controller - Version: "));
  Serial.println(VERSION);

  // E2PROM M24M01
  if (eeprom.begin()) {
    Serial.println(F("M24M01 Initialized"));
  }
  else
    Serial.println(F("M24M01 NOT Found"));

  // RTC RX8025T
  if (rtc.setup()) {
    if (DO_SET_TIME) {
      struct tm t = {};
      t.tm_year = SET_YEAR - 1900;
      t.tm_mon = SET_MONTH - 1;
      t.tm_mday = SET_DAY;
      t.tm_hour = SET_HOUR;
      t.tm_min = SET_MIN;
      t.tm_sec = SET_SEC;
      t.tm_wday = SET_DAY_OF_WEEK;

      rtc.setTime(&t);
      Serial.println("Time set.");
    }
    Serial.println(F("RTC Initialized"));
  }
  else
    Serial.println(F("RTC NOT Found"));

  // Init I2C1
  Wire1.begin();

  // MUX I2C
  if (muxi2c.begin()) {
    delay(100);
    muxi2c.disableAllChannels();
    Serial.println(F("MUXI2C Initialized"));
  }
  else
    Serial.println(F("MUXI2C NOT Found"));

  // PIO Remote Controller
  if (pioRC.begin()) {
    delay(100);
    pioRC.pinMode16(0xFFFF);

    for (uint8_t i = 0; i < 16; i++) {
      if ((i < 8) || (i > 12)) {
        pioRC.pinMode1(i, OUTPUT);
        pioRC.write1(i, LOW);
      }
    }
    Serial.println(F("PIO Initialized"));
  }
  else
    Serial.println(F("PIO NOT Found"));

  // Init relays
  //

  // Init D578
  initD578();
  Serial.println(F("D578 Initialized"));
}

//--------------------------------------------------------------------------
//-  LOOP REMOTE CONTROLLER
//--------------------------------------------------------------------------
void remoteLoop(void) {
  bool cmdReady = receiveDTMF();
  if (cmdReady) {
    cmdDTMF();
  }
}

//--------------------------------------------------------------------------
//-  FUNCTIONS HARDWARE REMOTE CONTROLLER
//--------------------------------------------------------------------------

void setChanAudio(bool state) {
  // SET SRC D578
  if (state == HIGH) {
    pioRC.write1(AUDIO2, LOW);
    pioRC.write1(AUDIO1, HIGH);
  }
  // DEFAULT SRC CW BEACON
  else {
    pioRC.write1(AUDIO1, LOW);
    pioRC.write1(AUDIO2, HIGH);
  }
}

void setRelayState(uint8_t num, bool state)
{
  if(state == HIGH)
  {
    pioRC.write1((num * 2), HIGH);
    delay(PULSE_DURATION);
    pioRC.write1((num * 2), LOW);
  }
  else
  {
    pioRC.write1((num * 2) + 1, HIGH);
    delay(PULSE_DURATION);
    pioRC.write1((num * 2) + 1, LOW);
  }
}

char readDTMF(void) {
  uint8_t valRead = 0;
  char valDTMF = 0;

  if ((pioRC.read1(NSTQ) == false) && (irqNSTQ == false)) {
    valRead = 0 | (pioRC.read1(Q4) << 3) | (pioRC.read1(Q3) << 2) | (pioRC.read1(Q2) << 1) | (pioRC.read1(Q1));
    valDTMF = charDTMF[valRead];
    irqNSTQ = true;
  }

  if ((pioRC.read1(NSTQ) == true) && (irqNSTQ == true)) {
    irqNSTQ = false;
  }

  return valDTMF;
}

void setTelcoPTT(bool state) {
  if (state == HIGH)
    pioRC.write1(TELCO_PTT, HIGH);
  else
    pioRC.write1(TELCO_PTT, LOW);
}

void readRTCTime(void) {
  struct tm t;
  rtc.getTime(&t);

  char buf[40];
  snprintf(buf, sizeof(buf), "%d %04d-%02d-%02d %02d:%02d:%02d",
           t.tm_wday, t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
           t.tm_hour, t.tm_min, t.tm_sec);
  Serial.println(buf);
}

void sendACK(void) {
  Serial.println(F("Send ACK"));
  setTelcoPTT(HIGH);
  delay(PULSE_DURATION);
  setTelcoPTT(LOW);
}

void sendError(void) {
  Serial.println(F("Send Error"));
  setTelcoPTT(HIGH);
  delay(PULSE_DURATION);
  setTelcoPTT(LOW);
}
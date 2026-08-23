//
// Created by Wizado F4ISE on 11/08/2026.
//
#include <Wire.h>

#include <M24M01.h>
#include <TCA9548.h>
#include <TCA9555.h>

#include "config.h"
#include "d578.h"
#include "dtmf.h"
#include "MeshCore.h"
#include "remote.h"

M24M01  eeprom(Wire, 0x50);
PCA9546 muxi2c(0x70, &Wire1);
TCA9555 pioRC(0x20, &Wire1);
TCA9555 pioExt1(0x21, &Wire1);
TCA9555 pioExt2(0x21, &Wire1);
TCA9555 pioUSB(0x22, &Wire1);


//#define WRITE_EEPROM
#define READ_EEPROM

const uint32_t TEST_ADDRESS = 0x0000;

// Polling Interrupt
bool irqNSTQ = false;
bool irqRTC = false;

//--------------------------------------------------------------------------
//-  SETUP REMOTE CONTROLLER
//--------------------------------------------------------------------------
void remoteInit(void) {
  Serial.print(F("Remote Controller - Version: "));
  Serial.println(VERSION);

  const char message[] = "Hello EEPROM!";
  size_t len = sizeof(message);

  // E2PROM M24M01
  if (eeprom.begin()) {
    #ifdef WRITE_EEPROM
    if (!eeprom.write(TEST_ADDRESS, (const uint8_t *)message, len)) {
      Serial.println("Write failed.");
      return;
    }
    Serial.println("Write OK.");
    #endif
    #ifdef READ_EEPROM
    char readBack[sizeof(message)] = {0};
    if (!eeprom.read(TEST_ADDRESS, (uint8_t *)readBack, len)) {
      Serial.println("Read failed.");
      return;
    }

    Serial.print("Read back: ");
    Serial.println(readBack);
    #endif
    MESH_DEBUG_PRINTLN("M24M01 Initialized");
  }
  else
    MESH_DEBUG_PRINTLN("M24M01 NOT Found");

  // I/O Interrupt
  pinMode(INT_PIO, INPUT);
  pinMode(INT_RTC, INPUT);

  // Init I2C1
  Wire1.begin();

  // MUX I2C
  if (muxi2c.begin()) {
    delay(100);
    muxi2c.disableAllChannels();
    MESH_DEBUG_PRINTLN("MUXI2C Initialized");
  }
  else
    MESH_DEBUG_PRINTLN("MUXI2C NOT Found");

  // PIO Extensions
  muxi2c.enableChannel(BUSEXT1);
  if (pioExt1.begin()) {
    delay(100);

    for (uint8_t i = 0; i < 16; i++) {
      pioExt1.pinMode1(i, OUTPUT);
      pioExt1.write1(i, LOW);
    }
    MESH_DEBUG_PRINTLN("PIO Ext.1 Initialized");
  }
  else
    MESH_DEBUG_PRINTLN("PIO Ext.1 NOT Found");
  muxi2c.disableChannel(BUSEXT1);

  muxi2c.enableChannel(BUSEXT2);
  if (pioExt2.begin()) {
    delay(100);

    for (uint8_t i = 0; i < 16; i++) {
      pioExt2.pinMode1(i, OUTPUT);
      pioExt2.write1(i, LOW);
    }
    MESH_DEBUG_PRINTLN("PIO Ext.2 Initialized");
  }
  else
    MESH_DEBUG_PRINTLN("PIO Ext.2 NOT Found");
  muxi2c.disableChannel(BUSEXT2);

  muxi2c.enableChannel(BUSUSB);
  if (pioUSB.begin()) {
    delay(100);

    for (uint8_t i = 0; i < 16; i++) {
      pioUSB.pinMode1(i, OUTPUT);
      pioUSB.write1(i, LOW);
    }
    MESH_DEBUG_PRINTLN("PIO USB Initialized");
  }
  else
    MESH_DEBUG_PRINTLN("PIO USB NOT Found");
  muxi2c.disableChannel(BUSUSB);

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
    MESH_DEBUG_PRINTLN("PIO R.C Initialized");
  }
  else
    MESH_DEBUG_PRINTLN("PIO R.C NOT Found");

  // Init Relays

  // Init D578
  initD578();
  MESH_DEBUG_PRINTLN("D578 Initialized");
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
    MESH_DEBUG_PRINTLN("Set CHAN Audio D578");
  }
  // DEFAULT SRC CW BEACON
  else {
    pioRC.write1(AUDIO1, LOW);
    pioRC.write1(AUDIO2, HIGH);
    MESH_DEBUG_PRINTLN("Set CHAN Audio CW");
  }
}

void setRelayStateRC(uint8_t num, bool state)
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

void setRelayStateExt1(uint8_t num, bool state) {
  muxi2c.enableChannel(BUSEXT1);
  if ((num == 0) || (num == 2) || (num == 4) || (num == 6)){
    if(state == HIGH)
    {
      pioExt1.write1(num, HIGH);
      delay(PULSE_DURATION);
      pioExt1.write1(num, LOW);
    }
    else
    {
      pioExt1.write1(num + 1, HIGH);
      delay(PULSE_DURATION);
      pioExt1.write1(num + 1, LOW);
    }
  }
  else {
    if(state == HIGH)
    {
      pioExt1.write1((15 - num), HIGH);
      delay(PULSE_DURATION);
      pioExt1.write1((15 - num), LOW);
    }
    else
    {
      pioExt1.write1((15 - num) + 1, HIGH);
      delay(PULSE_DURATION);
      pioExt1.write1((15 - num) + 1, LOW);
    }
  }
  muxi2c.disableChannel(BUSEXT1);
}

void setRelayStateExt2(uint8_t num, bool state)
{
  muxi2c.enableChannel(BUSEXT2);
  if ((num == 0) || (num == 2) || (num == 4) || (num == 6)){
    if(state == HIGH)
    {
      pioExt2.write1(num, HIGH);
      delay(PULSE_DURATION);
      pioExt2.write1(num, LOW);
    }
    else
    {
      pioExt2.write1(num + 1, HIGH);
      delay(PULSE_DURATION);
      pioExt2.write1(num + 1, LOW);
    }
  }
  else {
    if(state == HIGH)
    {
      pioExt2.write1((15 - num), HIGH);
      delay(PULSE_DURATION);
      pioExt2.write1((15 - num), LOW);
    }
    else
    {
      pioExt2.write1((15 - num) + 1, HIGH);
      delay(PULSE_DURATION);
      pioExt2.write1((15 - num) + 1, LOW);
    }
  }
  muxi2c.disableChannel(BUSEXT2);
}

void setRelayStateUSB(uint8_t num, bool state)
{
  muxi2c.enableChannel(BUSUSB);
  if(state == HIGH)
  {
    pioUSB.write1((num * 2), HIGH);
    delay(PULSE_DURATION);
    pioUSB.write1((num * 2), LOW);
  }
  else
  {
    pioUSB.write1((num * 2) + 1, HIGH);
    delay(PULSE_DURATION);
    pioUSB.write1((num * 2) + 1, LOW);
  }
  muxi2c.disableChannel(BUSUSB);
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

void readRTCTime(void) {
  //
}

void setTelcoPTT(bool state) {
  if (state == HIGH) {
    pioRC.write1(TELCO_PTT, HIGH);
    MESH_DEBUG_PRINTLN("Enable PTT Telco");
  }
  else {
    pioRC.write1(TELCO_PTT, LOW);
    MESH_DEBUG_PRINTLN("Disable PTT Telco");
  }
}

void sendACK(void) {
  MESH_DEBUG_PRINTLN("Send ACK");
  setChanAudio(LOW);
  setTelcoPTT(HIGH);
  delay(PULSE_DURATION);
  setTelcoPTT(LOW);
}

void sendError(void) {
  MESH_DEBUG_PRINTLN("Send Error");
  setChanAudio(LOW);
  setTelcoPTT(HIGH);
  delay(PULSE_DURATION);
  setTelcoPTT(LOW);
}
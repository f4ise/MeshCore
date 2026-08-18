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
    MESH_DEBUG_PRINTLN("M24M01 Initialized");
  }
  else
    MESH_DEBUG_PRINTLN("M24M01 NOT Found");

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
    MESH_DEBUG_PRINTLN("PIO Initialized");
  }
  else
    MESH_DEBUG_PRINTLN("PIO NOT Found");

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
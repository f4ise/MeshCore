//
// Created by Wizado F4ISE on 11/08/2026.
//
#include <Arduino.h>

#include "dtmf.h"
#include "remote.h"

bool statePTT = false;

char charRxDTMF[6] = {0, 0, 0, 0, 0, 0};
bool cmdReady = false;
int cmdParsed = 0;
uint8_t cmdIdx = 0;

bool receiveDTMF(void) {
  char valDTMF = readDTMF();
  if (valDTMF != 0) {
    Serial.print(valDTMF);
    if (valDTMF == '#') {
      cmdReady = false;
      flushBuffer();
      cmdIdx = 0;
    }
    if ((valDTMF == '*') && (cmdIdx == 5)) {
      cmdReady = true;
    }
    charRxDTMF[cmdIdx] = valDTMF;
    if(cmdIdx < 6) {
      cmdIdx++;
    }
  }
  return cmdReady;
}

void cmdDTMF(void) {
  Serial.println();

  bool statusACK = false;
  cmdReady = false;

  cmdParsed = parseCmd();
  Serial.print("cmdParsed: ");
  Serial.println(cmdParsed);

  switch (cmdParsed) {
    case 10:
        Serial.println(F("CMD 0010"));
        setRelayState(0, false);
        statusACK = true;
        break;
    case 11:
        Serial.println(F("CMD 0011"));
        setRelayState(0, true);
        statusACK = true;
        break;
    case 20:
        Serial.println(F("CMD 0020"));
        setRelayState(1, false);
        statusACK = true;
        break;
    case 21:
        Serial.println(F("CMD 0021"));
        setRelayState(1, true);
        statusACK = true;
        break;
    case 30:
        Serial.println(F("CMD 0030"));
        setRelayState(2, false);
        statusACK = true;
        break;
    case 31:
        Serial.println(F("CMD 0031"));
        setRelayState(2, true);
        statusACK = true;
        break;
    case 40:
        Serial.println(F("CMD 0040"));
        setRelayState(3, false);
        statusACK = true;
        break;
    case 41:
        Serial.println(F("CMD 0041"));
        setRelayState(3, true);
        statusACK = true;
        break;
    default:
        Serial.println(F("CMD Not recognized"));
        readRTCTime();
        break;
  }
  if (statusACK)
    sendACK();
  else
    sendError();
}

int parseCmd(void) {
  int parsed = 0;
  parsed = ((charRxDTMF[1] - 48) * 1000) + ((charRxDTMF[2] - 48) * 100) + ((charRxDTMF[3] - 48) * 10) + (charRxDTMF[4] - 48);
  return parsed;
}

void flushBuffer(void) {
  for (uint8_t i = 0; i < 6; i++) {
    charRxDTMF[i] = 0;
  }
}
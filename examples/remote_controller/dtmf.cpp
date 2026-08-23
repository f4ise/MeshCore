//
// Created by Wizado F4ISE on 11/08/2026.
//
#include <Arduino.h>

#include <MeshCore.h>

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
  MESH_DEBUG_PRINTLN("cmdParsed: %d", cmdParsed);

  switch (cmdParsed) {
    // Remote Controller
    case 0010:
        MESH_DEBUG_PRINTLN("CMD 0010");
        setRelayStateRC(0, false);
        statusACK = true;
        break;
    case 0011:
        MESH_DEBUG_PRINTLN("CMD 0011");
        setRelayStateRC(0, true);
        statusACK = true;
        break;
    case 0020:
        MESH_DEBUG_PRINTLN("CMD 0020");
        setRelayStateRC(1, false);
        statusACK = true;
        break;
    case 0021:
        MESH_DEBUG_PRINTLN("CMD 0021");
        setRelayStateRC(1, true);
        statusACK = true;
        break;
    case 0030:
        MESH_DEBUG_PRINTLN("CMD 0030");
        setRelayStateRC(2, false);
        statusACK = true;
        break;
    case 0031:
        MESH_DEBUG_PRINTLN("CMD 0031");
        setRelayStateRC(2, true);
        statusACK = true;
        break;
    case 0040:
        MESH_DEBUG_PRINTLN("CMD 0040");
        setRelayStateRC(3, false);
        statusACK = true;
        break;
    case 0041:
        MESH_DEBUG_PRINTLN("CMD 0041");
        setRelayStateRC(3, true);
        statusACK = true;
        break;

    // Extension 1
    case 1010:
        MESH_DEBUG_PRINTLN("CMD 1010");
        setRelayStateExt1(0, false);
        statusACK = true;
        break;
    case 1011:
        MESH_DEBUG_PRINTLN("CMD 1011");
        setRelayStateExt1(0, true);
        statusACK = true;
        break;
    case 1020:
        MESH_DEBUG_PRINTLN("CMD 1020");
        setRelayStateExt1(1, false);
        statusACK = true;
        break;
    case 1021:
        MESH_DEBUG_PRINTLN("CMD 1021");
        setRelayStateExt1(1, true);
        statusACK = true;
        break;
    case 1030:
        MESH_DEBUG_PRINTLN("CMD 1030");
        setRelayStateExt1(2, false);
        statusACK = true;
        break;
    case 1031:
        MESH_DEBUG_PRINTLN("CMD 1031");
        setRelayStateExt1(2, true);
        statusACK = true;
        break;
    case 1040:
        MESH_DEBUG_PRINTLN("CMD 1040");
        setRelayStateExt1(3, false);
        statusACK = true;
        break;
    case 1041:
        MESH_DEBUG_PRINTLN("CMD 1041");
        setRelayStateExt1(3, true);
        statusACK = true;
        break;
    case 1050:
        MESH_DEBUG_PRINTLN("CMD 1050");
        setRelayStateExt1(4, false);
        statusACK = true;
        break;
    case 1051:
        MESH_DEBUG_PRINTLN("CMD 1051");
        setRelayStateExt1(4, true);
        statusACK = true;
        break;
    case 1060:
        MESH_DEBUG_PRINTLN("CMD 1060");
        setRelayStateExt1(5, false);
        statusACK = true;
        break;
    case 1061:
        MESH_DEBUG_PRINTLN("CMD 1061");
        setRelayStateExt1(5, true);
        statusACK = true;
        break;
    case 1070:
        MESH_DEBUG_PRINTLN("CMD 1070");
        setRelayStateExt1(6, false);
        statusACK = true;
        break;
    case 1071:
        MESH_DEBUG_PRINTLN("CMD 1071");
        setRelayStateExt1(6, true);
        statusACK = true;
        break;
    case 1080:
        MESH_DEBUG_PRINTLN("CMD 1080");
        setRelayStateExt1(7, false);
        statusACK = true;
        break;
    case 1081:
        MESH_DEBUG_PRINTLN("CMD 1081");
        setRelayStateExt1(7, true);
        statusACK = true;
        break;

    // Extension 2
    case 2010:
        MESH_DEBUG_PRINTLN("CMD 2010");
        setRelayStateExt2(0, false);
        statusACK = true;
        break;
    case 2011:
        MESH_DEBUG_PRINTLN("CMD 2011");
        setRelayStateExt2(0, true);
        statusACK = true;
        break;
    case 2020:
        MESH_DEBUG_PRINTLN("CMD 2020");
        setRelayStateExt2(1, false);
        statusACK = true;
        break;
    case 2021:
        MESH_DEBUG_PRINTLN("CMD 2021");
        setRelayStateExt2(1, true);
        statusACK = true;
        break;
    case 2030:
        MESH_DEBUG_PRINTLN("CMD 2030");
        setRelayStateExt2(2, false);
        statusACK = true;
        break;
    case 2031:
        MESH_DEBUG_PRINTLN("CMD 2031");
        setRelayStateExt2(2, true);
        statusACK = true;
        break;
    case 2040:
        MESH_DEBUG_PRINTLN("CMD 2040");
        setRelayStateExt2(3, false);
        statusACK = true;
        break;
    case 2041:
        MESH_DEBUG_PRINTLN("CMD 2041");
        setRelayStateExt2(3, true);
        statusACK = true;
        break;
    case 2050:
        MESH_DEBUG_PRINTLN("CMD 2050");
        setRelayStateExt2(4, false);
        statusACK = true;
        break;
    case 2051:
        MESH_DEBUG_PRINTLN("CMD 2051");
        setRelayStateExt2(4, true);
        statusACK = true;
        break;
    case 2060:
        MESH_DEBUG_PRINTLN("CMD 2060");
        setRelayStateExt2(5, false);
        statusACK = true;
        break;
    case 2061:
        MESH_DEBUG_PRINTLN("CMD 2061");
        setRelayStateExt2(5, true);
        statusACK = true;
        break;
    case 2070:
        MESH_DEBUG_PRINTLN("CMD 2070");
        setRelayStateExt2(6, false);
        statusACK = true;
        break;
    case 2071:
        MESH_DEBUG_PRINTLN("CMD 2071");
        setRelayStateExt2(6, true);
        statusACK = true;
        break;
    case 2080:
        MESH_DEBUG_PRINTLN("CMD 2080");
        setRelayStateExt2(7, false);
        statusACK = true;
        break;
    case 2081:
        MESH_DEBUG_PRINTLN("CMD 2081");
        setRelayStateExt2(7, true);
        statusACK = true;
        break;

    // Extension USB
    case 3010:
        MESH_DEBUG_PRINTLN("CMD 3010");
        setRelayStateUSB(0, false);
        statusACK = true;
        break;
    case 3011:
        MESH_DEBUG_PRINTLN("CMD 3011");
        setRelayStateUSB(0, true);
        statusACK = true;
        break;
    case 3020:
        MESH_DEBUG_PRINTLN("CMD 3020");
        setRelayStateUSB(1, false);
        statusACK = true;
        break;
    case 3021:
        MESH_DEBUG_PRINTLN("CMD 3021");
        setRelayStateUSB(1, true);
        statusACK = true;
        break;
    case 3030:
        MESH_DEBUG_PRINTLN("CMD 3030");
        setRelayStateUSB(2, false);
        statusACK = true;
        break;
    case 3031:
        MESH_DEBUG_PRINTLN("CMD 3031");
        setRelayStateUSB(2, true);
        statusACK = true;
        break;
    case 3040:
        MESH_DEBUG_PRINTLN("CMD 3040");
        setRelayStateUSB(3, false);
        statusACK = true;
        break;
    case 3041:
        MESH_DEBUG_PRINTLN("CMD 3041");
        setRelayStateUSB(3, true);
        statusACK = true;
        break;
    case 3050:
        MESH_DEBUG_PRINTLN("CMD 3050");
        setRelayStateUSB(4, false);
        statusACK = true;
        break;
    case 3051:
        MESH_DEBUG_PRINTLN("CMD 3051");
        setRelayStateUSB(4, true);
        statusACK = true;
        break;
    case 3060:
        MESH_DEBUG_PRINTLN("CMD 3060");
        setRelayStateUSB(5, false);
        statusACK = true;
        break;
    case 3061:
        MESH_DEBUG_PRINTLN("CMD 3061");
        setRelayStateUSB(5, true);
        statusACK = true;
        break;
    case 3070:
        MESH_DEBUG_PRINTLN("CMD 3070");
        setRelayStateUSB(6, false);
        statusACK = true;
        break;
    case 3071:
        MESH_DEBUG_PRINTLN("CMD 3071");
        setRelayStateUSB(6, true);
        statusACK = true;
        break;
    case 3080:
        MESH_DEBUG_PRINTLN("CMD 3080");
        setRelayStateUSB(7, false);
        statusACK = true;
        break;
    case 3081:
        MESH_DEBUG_PRINTLN("CMD 3081");
        setRelayStateUSB(7, true);
        statusACK = true;
        break;

    // Command Error
    default:
        MESH_DEBUG_PRINTLN("CMD Not recognized");
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
//
// Created by Wizado F4ISE on 12/08/2026.
//

#include "config.h"
#include "d578.h"

uint8_t bufferTX[8] = {0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06};

void initD578(void)
{
  micSerial.begin(micSpeed);
  delay(2000);
}

int Pressed(uint8_t bp)
{
  int ackD578 = 0;

  bufferTX[4] = bp;
  bufferTX[2] = 0x01;
  micSerial.write(bufferTX, 8);
  delay(200);
  bufferTX[2] = 0x00;
  micSerial.write(bufferTX, 8);

  return ackD578;
}

int longPressed(uint8_t bp)
{
  int ackD578 = 0;

  bufferTX[4] = bp;
  bufferTX[3] = 0x01;
  bufferTX[2] = 0x01;
  micSerial.write(bufferTX, 8);
  delay(200);
  bufferTX[3] = 0x00;
  bufferTX[2] = 0x00;
  micSerial.write(bufferTX, 8);

  return ackD578;
}

void pttPressed(bool state)
{
  // Enable PTT
  bufferTX[4] = bpPTT;
  bufferTX[3] = 0;
  bufferTX[2] = 0;

  if(state)
  {
    bufferTX[1] = 0x01;
  }
  else
  {
    bufferTX[1] = 0;
  }
  micSerial.write(bufferTX, 8);
}

void sendKeepAlive(void)
{
  micSerial.write(0x06);
}
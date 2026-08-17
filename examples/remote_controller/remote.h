//
// Created by Wizado F4ISE on 11/08/2026.
//

#ifndef REMOTE_H
#define REMOTE_H

#include <Arduino.h>

void remoteInit(void);
void remoteLoop(void);

void setChanAudio(bool state);
void setRelayState(uint8_t num, bool state);
void setTelcoPTT(bool state);

char readDTMF(void);
void readRTCTime(void);

void sendACK(void);
void sendError(void);

#endif // REMOTE_H

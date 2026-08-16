//
// Created by Wizado F4ISE on 12/08/2026.
//

#ifndef D578_H
#define D578_H

#include <Arduino.h>
// Adress
#define bpPTT       0x00
#define bp0         0x01
#define bp1         0x02
#define bp2         0x03
#define bp3         0x04
#define bp4         0x05
#define bp5         0x06
#define bp6         0x07
#define bp7         0x08
#define bp8         0x09
#define bp9         0x0A
#define bpST        0x0B
#define bpDH        0x0C
#define bpSAB       0x0D

#define bpUP        0x10
#define bpDN        0x11
#define bpA         0x1A
#define bpB         0x1B
#define bpC         0x1C
#define bpD         0x1D

void initD578(void);

int Pressed(uint8_t bp);
int longPressed(uint8_t bp);
void pttPressed(bool state);
void sendKeepAlive(void);


#endif // MESHCORE_D578_H

//
// Created by Wizado F4ISE on 11/08/2026.
//

#ifndef CONFIG_H
#define CONFIG_H

// APPS
#define VERSION             "0.1A"

// I/O BOARD
#define INT_PIO             2
#define INT_RTC             3

// RELAYS
#define PULSE_DURATION      110

// DTMF
#define Q1                  8
#define Q2                  9
#define Q3                  10
#define Q4                  11
#define NSTQ                12

// ANYTONE D578
#define micSerial           Serial2
#define micSpeed            115200

// TELCO
#define TELCO_PTT           15

// AUDIO
#define AUDIO1              13      // AM D578
#define AUDIO2              14      // CW BALISE

#endif // CONFIG_H

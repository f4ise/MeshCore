//
// Created by Wizado F4ISE on 11/08/2026.
//

#ifndef CONFIG_H
#define CONFIG_H

// APPS
#define VERSION             "0.1A"

// AUDIO
#define AUDIO1              13      // AM D578
#define AUDIO2              14      // CW BALISE

// DTMF
#define Q1                  8
#define Q2                  9
#define Q3                  10
#define Q4                  11
#define NSTQ                12

// I/O BOARD
#define INT_PIO             2
#define INT_RTC             3

// MUX
#define BUSEXT1             0
#define BUSEXT2             1
#define BUSUSB              0

// RELAYS
#define PULSE_DURATION      110

// ANYTONE D578
#define micSerial           Serial2
#define micSpeed            115200

// TELCO
#define TELCO_PTT           15


#endif // CONFIG_H

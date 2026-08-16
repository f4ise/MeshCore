//
// Created by Wizado F4ISE on 11/08/2026.
//

#ifndef DTMF_H
#define DTMF_H

const char charDTMF[16] = {'D','1','2','3','4','5','6','7','8','9','0','*','#','A','B','C'};

bool receiveDTMF(void);
void cmdDTMF(void);
int parseCmd(void);
void flushBuffer(void);


#endif // DTMF_H

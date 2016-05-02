//
// Created by juraj on 10.04.16..
//

#ifndef PROJECT_SERIAL_SUPPORT_H
#define PROJECT_SERIAL_SUPPORT_H

#include <iostream>
#include <tuw_self_localization/fixedpoint.h>
#include <tuw_self_localization/rs232.h>
#include <tuw_self_localization/config.h>


extern int cport_nr;

void sendInt(unsigned int p);
int receiveInt();
int receiveToken(const char *synctoken);
void syncFPGA();
bool initComport();
unsigned char FPGAgetchar();
void getFPGAmsg();

template<class msg>
void sendFPGAstruct(msg m);


inline unsigned char FPGAgetchar() {
    unsigned char c;
    while(RS232_PollComport(cport_nr, &c, 1) < 1)
        usleep(READTIMEOUT);
    return c;
}



template<class msg>
inline void sendFPGAstruct(msg m) {

    int i;
    for(i = 0; i < sizeof(m); i++ ) {
        unsigned char c = *(((unsigned char *) &m) + i);
        RS232_SendBuf(cport_nr, &c, 1);
        //printf("Sent byte %d: %x \n", i, c);
        usleep(SENDTIMEOUT);
    }
    //RS232_SendBuf(cport_nr, (unsigned char *) &m, sizeof(m));
    //printf("Sent %d chars\n", i);
}

template<class msg>
inline void readFPGAstruct(msg &m) {

    int i;
    for(i = 0; i < sizeof(m); i++ ) {
        *(((unsigned char *) &m) + i) = FPGAgetchar();
    }
}

inline void sendFPGAstring(const char *m) {


    while(*m != '\0') {
        RS232_SendBuf(cport_nr, (const unsigned char *) m++, 1);
        usleep(SENDTIMEOUT);
    }
}


#endif //PROJECT_SERIAL_SUPPORT_H

//
// Created by juraj on 10.04.16..
//

#ifndef SERIAL_SUPPORT_H
#define SERIAL_SUPPORT_H

#include <iostream>
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
    int i = 0;  int cnt = 0;
    while (i < sizeof(m)) {
        cnt++;
        i += RS232_SendBuf(cport_nr, (unsigned char *) &m + i, sizeof(m) - i);
        if(i < sizeof(m)) usleep(WRITETIMEOUT);
    }
    if(cnt > 1) std::cout << "send: waited " << cnt << "times" << std::endl;
}

template<class msg>
inline void readFPGAstruct(msg &m) {
    int i = 0; int cnt = 0;
    while (i < sizeof(m)) {
        cnt++;
        i += RS232_PollComport(cport_nr, (unsigned char *) &m + i, sizeof(m) - i);
        if(i < sizeof(m)) usleep(READTIMEOUT);
    }
    if(cnt > 1) std::cout << "read: waited " << cnt << "times" << std::endl;
}

inline void sendFPGAstring(const char *m) {
    RS232_cputs(cport_nr, m);
}

#endif //SERIAL_SUPPORT_H

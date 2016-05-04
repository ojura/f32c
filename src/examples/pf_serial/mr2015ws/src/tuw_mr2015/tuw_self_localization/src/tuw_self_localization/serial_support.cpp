//
// Created by juraj on 10.04.16..
//

#include <tuw_self_localization/serial_support.h>


using std::cin; using std::cout; using std::endl;
int cport_nr;
// serial helper functions
void sendInt(unsigned int p) {

    unsigned char *c =  (unsigned char *) &p;

    //std::cout << "Sending " <<  (int) c[0]  << " "<< (int) c[1] << " " << (int) c[2] << " " << (int) c[3] << std::endl;
    RS232_SendBuf(cport_nr,  (unsigned char *) &p, 4);
}

int receiveToken(const char *synctoken) {

    unsigned char c;
    int p = 0;

    while(p != 4) {

        int tries = 15;
        while(RS232_PollComport(cport_nr, &c, 1) < 1 && tries) {
            usleep(READTIMEOUT);
            tries--;
        }

        if(tries==0) return false;

        if(synctoken[p] == c) p++;
        else return false;
    }

    return true;
}

void syncFPGA() {
    int synced = 0;

    while(!synced) {
        RS232_flushRXTX(cport_nr);
        sendFPGAstring("sync");
        synced=receiveToken("sack");
        if(!synced) usleep(SYNCTIMEOUT);
    }

}


bool initComport() {
    char mode[] = {'8', 'N', '1', 0};

    printf("init comport\n");
    //if (comportinited) RS232_CloseComport(cport_nr);
    cport_nr = 16;
    while (RS232_OpenComport(cport_nr, COM_BAUDRATE, mode)) {
        printf("tried comport %d\n", cport_nr);
        cport_nr++;

        if (cport_nr == 20) exit(1);
    }

    printf("comport inited on %d\n", cport_nr);
    return true;
}

void getFPGAmsg() {

    unsigned char c;

    int endc = 0;
    const char endtoken[] = "!end!\r\n";
    while(endc != strlen(endtoken)) {
        c = FPGAgetchar();

        if (c == endtoken[endc]) endc++;
        else {
            for (int i = 0; i < endc; i++)
                std::cout << endtoken[i];
            endc = 0;
            std::cout << c;
        }
    }
}

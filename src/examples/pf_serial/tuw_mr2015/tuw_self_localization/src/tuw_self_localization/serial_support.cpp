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

int receiveInt() {

    unsigned char c[4];
    int p = 0;

    while(p < 4) {
        p += RS232_PollComport(cport_nr, c+p, 4-p);
        usleep(READTIMEOUT);
        //cout << "waiting " << p << endl;
    }

    //std::cout <<  (int) c[0]  << " "<< (int) c[1] << " " << (int) c[2] << " " << (int) c[3] << std::endl;
    return *((int *) c);
}

int receiveToken(const char *synctoken) {

    //cout << "Beginning receiveToken()" << endl;
    unsigned char c;
    int p = 0;

    while(p != 4) {

        int tries = 10;
        while(RS232_PollComport(cport_nr, &c, 1) < 1 && tries) {
            usleep(READTIMEOUT);
            tries--;
        }

        if(tries==0) return false;
        //cout << "Received " <<  c << " p= " << p << endl;

        if(synctoken[p] == c) p++;
        else return false;
    }

    return true;
}

void syncFPGA() {
    int synced = 0;

    while(!synced) {
        RS232_flushRXTX(cport_nr);
        RS232_cputs(cport_nr, "sync");
        synced=receiveToken("sack");
    }

}


bool initComport() {
    int bdrate = 3000000;
    char mode[] = {'8', 'N', '1', 0};

    printf("init comport\n");
    //if (comportinited) RS232_CloseComport(cport_nr);
    cport_nr = 16;
    while (RS232_OpenComport(cport_nr, bdrate, mode)) {
        printf("tried comport %d\n", cport_nr);
        cport_nr++;

        if (cport_nr == 20) exit(1);
    }

    printf("comport inited on %d\n", cport_nr);
    return true;
}


unsigned char FPGAgetchar() {
    unsigned char c;
    while(RS232_PollComport(cport_nr, &c, 1) < 1)
        usleep(READTIMEOUT);
    return c;

}

void getFPGAmsg() {

    unsigned char c;

    int endc = 0;
    const char endtoken[] = "!end!\n";
    while(endc != strlen(endtoken)-1) {
        c = FPGAgetchar();

        if (c == endtoken[endc]) endc++;
        else {
            for (int i = 0; i < endc; i++)
                std::cout << endtoken[endc];
            endc = 0;
            std::cout << c;
        }
    }
}

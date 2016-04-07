extern "C" {
    #include <stdio.h>
    #include <io.h>
    #include <mips/io.h>
    #include <mips/_fpmath.h>
    #include <stdarg.h>
    #include <string.h>
}

#include "/home/juraj/projects/catkin/mr2015ws/src/tuw_mr2015/tuw_self_localization/include/tuw_self_localization/fixedpoint.h"


static unsigned int _tprev = 0;
inline void eventPress() {
    
    int in;
    unsigned int t;
    do {
        RDTSC(t);
        INB(in, IO_PUSHBTN);
    } while (in == 0 || (t-_tprev) < 2000000 );
    
    _tprev = t;
    
}

inline void eventdePress() {
    int in;
    unsigned int t;
    do {
        RDTSC(t);
        INB(in, IO_PUSHBTN);
    } while (in != 0 || (t-_tprev) < 2000000 );
    
    _tprev = t;
}


extern char sio_rxbuf[];
extern uint8_t sio_rxbuf_tail;

void sync() {
    // sync
    int sync = 0;
    //printf("Beginning sync...\n");
    const char synctoken[] = "sync";
    while(sync != 4) {
        char p = getchar();
        
        //printf("Received: %x \n", p);
        
        if(p==synctoken[sync]) sync++;
        else sync = 0;
        
        //printf("Sync = %d\n", sync);
        
    }
    printf("sack");
    
}

//template<class tip> int asint(tip a) { return int(a); }
void print(fixed a) { printf("%f", float(a)); }
void print(int a) { printf("%d", a); }
void print(bool a) { printf("%d", a); }


void main(void)
{
    
    while(1) {
        
        sync();
    
        char ctmp[5];
        for(int i = 0; i < 4; i++)
            ctmp[i] = getchar();
        
        ctmp[4] = '\0';
              
        if(strcmp(ctmp, "pars")) {
            printf("\nSomething wrong, expected pars, got %s!\n",ctmp);
            continue; }
        else printf("\nReceiving pars on FPGA...\n");
        
        struct {
            bool enable_resample;
            bool enable_weighting;
            bool enable_update;
            fixed alpha1;
            fixed alpha2;
            fixed alpha3;
            fixed alpha4;
            fixed alpha5;
            fixed alpha6;
            int nr_of_samples;
            int nr_of_beams;
        } pars;
        
        
        unsigned char *p = (unsigned char *) &pars;
        for(unsigned int i = 0; i<sizeof(pars); i++) {
            *p = getchar();
            printf("Received byte %d/%d: %x\n", i+1, sizeof(pars), *p);
            p++;
        }
            

        
        printf("Received data:\n");
       
        #define setparam(x) printf(#x); printf(" = "); print(pars.x); printf("\n")
        
        setparam(enable_resample);
        setparam(enable_weighting);
        setparam(enable_update);
        setparam(alpha1);
        setparam(alpha2);
        setparam(alpha3);
        setparam(alpha4);
        setparam(alpha5);
        setparam(alpha6);
        setparam(nr_of_samples);
        setparam(nr_of_beams);
        
        #undef setparam        
        
                
        printf("!end!\n");
                
        
    }
    
}
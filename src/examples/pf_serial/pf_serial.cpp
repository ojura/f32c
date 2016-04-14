

extern "C" {
    #include <stdio.h>
    #include <stdlib.h>
    #include <dev/io.h>
    #include <mips/io.h>
    #include <dev/sio.h>
    #include <mips/_fpmath.h>
    #include <stdarg.h>
    #include <string.h>
    #include <fcntl.h>    
    #include <unistd.h>
}

#include <fatfs/ff.h>
    
    
#include "tuw_mr2015/tuw_self_localization/include/tuw_self_localization/fixedpoint.h"


////////////////
// debouncing for button inputs
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

////////////////


// waits for "sync" on serial, checks if RX buffer is clear 
// to ensure synchronisation, answers with "sack".
bool sync_serial() {
    // sync
    int sync = 0;
    //printf("Beginning sync...\n");
    const char synctoken[] = "sync";
    while(sync != 4) {
        char p = getchar();
        
        //printf("Received: %x \n", p);
        
        if(p==synctoken[sync]) sync++;
        else 
            return false;
        
    }
    if (!sio_rxempty()) 
        return false;
    
    printf("sack");
    return true;
    
}


#include "tuw_mr2015/tuw_self_localization/include/tuw_self_localization/com_structs.h"


// helper functions for printing various data types 
void print(fixed a) { printf("%f", float(a)); }
void print(int a) { printf("%d", a); }
void print(unsigned int a) { printf("%u", a); }
void print(bool a) { printf("%d", a); }
void print(likelihoodLookuptable a) {
        printf("[");
        for(int i = 0; i<5; i++)
            printf("%.5f ", float(a.gausspdf[i]));
        printf("...]");
}


// generic function for reading a struct via serial link from master
template<class msg> void readstruct(msg& m) {
    unsigned char *p = (unsigned char *) &m;
    for(unsigned int i = 0; i<sizeof(m); i++) {
        *p = getchar(); p++;
    }

}

// distance-transformed map is stored in heap 
unsigned char *map_mem;
unsigned int map_size;
// TODO: making map size dynamic.
unsigned int mapx = 558; unsigned int mapy = 558;


// 2D array-like function for accessing map value at pixel coordinates c,r
unsigned char& map(int c, int r) {
    return *(map_mem + r + c * mapx);    
}


msgtype_sample *samples_mem;
unsigned int allocated_samples = 0;


void main(void)
{
    
    sio_setbaud(1152000);
    int f = open("d:likelihood.map", O_RDONLY);
    map_size = lseek(f, 0, SEEK_END);
        
    map_mem = (unsigned char*) malloc(map_size);
    
    if(map_mem == NULL)  {
        printf("Could not allocate memory for map!\n");
        return;
    }
        
    
    lseek(f, 0, SEEK_SET);
    
    read(f, map_mem, map_size);
    

    printf("Read likelihood.map, handle %d, size %u\n", f, map_size);
    
    // TODO: read map dimensions from file
           
    // 2D array pointer
    
    //for(int i = 0; i < 20; i++)
    //    printf("%x ", map(0,i));
    
    //printf("\n");
    
    while(1) {
        
        while(!sync_serial()) { };
        
        char ctmp[5];
        for(int i = 0; i < 4; i++)
            ctmp[i] = getchar();
        
        ctmp[4] = '\0';
        
        if(strcmp(ctmp, "pars") == 0) {
            printf("Receiving pars on FPGA...\n");
            
            readstruct(msg_params);
            
            printf("Received pars:\n");
            
            #define member(t,x,y) printf(#t " " #x " = "); print(msg_params.x); printf("\n")
            com_params;
            #undef member
            
            if(msg_params.nr_of_samples != allocated_samples) {
                samples_mem = (msgtype_sample*) realloc(samples_mem, sizeof(msgtype_sample) * msg_params.nr_of_samples);
                if(samples_mem == NULL)  {
                    printf("Could not allocate memory for %d samples!\n", msg_params.nr_of_samples);
                    return;
                }
                else printf("Allocated %d samples.\n", msg_params.nr_of_samples);
            }
                
                printf("!end!\n");
            }

        else if(strcmp(ctmp, "data") == 0) {
            
            printf("Receiving data on FPGA...\n");
            
            readstruct(msg_frame_data);
            
            printf("Received data:\n");
            
            #define member(t,x,y) printf(#t " " #x " = "); print(msg_frame_data.x); printf("\n")
            com_frame_data;
            #undef member
            
            printf("!end!\n");
            
        }  
        
        else printf("\nSomething wrong, expected a message id, got %s!\n",ctmp);
        continue; }
        
        
}



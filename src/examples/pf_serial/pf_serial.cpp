

extern "C" {
    #include <stdio.h>
    #include <stdlib.h>
    #include <dev/io.h>
    #include <mips/io.h>
    #include <dev/sio.h>
    #include <sys/fcntl.h>
    #include <string.h>
    #include <unistd.h>
    #include <sys/param.h>
}

#include "sort_adapter.h"
#include <fatfs/ff.h>
    
#include "tuw_mr2015/tuw_self_localization/include/tuw_self_localization/fixedpoint.h"

gaussian_random gauss_;


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


msgtype_sample *samples_mem, *samples_new_mem;
unsigned int allocated_samples = 0;
unsigned int used_samples = 0;
lfsr uniform(864386345);

void resample() {
/**
 * @ToDo Resample
 * implement a resample weel
 **/

// M is the number the samples to destroy
int M = double(msg_params.resample_rate * fixed(used_samples)), N = used_samples;
#define MWEAKEST 0
#define LOWVARIANCE 1

if(msg_params.resample_strategy == MWEAKEST) {
    
    struct { bool operator()(const msgtype_sample &a, const msgtype_sample &b) { return a.weight < b.weight; } } cmp;
    sort( samples_mem, samples_mem + used_samples, cmp);

    fixed sigma_position = msg_params.sigma_static_position*msg_frame_data.duration_last_update;
    fixed sigma_orientation = msg_params.sigma_static_orientation*msg_frame_data.duration_last_update;
    
    for(int i = 0, k = N - 1; i < MIN(M, N/2); i++, k--) {
        // napravi kopiju...
        //samples_mem[i] = samples_mem[k];
        // i dodaj random
        //normal ( samples[i], *samples[i], config_.sigma_static_position*dt, config_.sigma_static_orientation*dt );
        
        samples_mem[i].x = gauss_.generate(samples_mem[k].x, sigma_position);
        samples_mem[i].y = gauss_.generate(samples_mem[k].y, sigma_position);
        
        samples_mem[i].theta = gauss_.generate(samples_mem[k].theta, sigma_orientation);

    }
} 


else if(msg_params.resample_strategy == LOWVARIANCE) {
    
    // TODO
    /*
    // destroying M samples like in the first algorithm == sampling N-M samples
    // M is now the number of samples to draw!
    M = N-M;

    double r = d(generator_) / M;
    double c = samples[0]->weight();
    int i = 0;
    
    for(int m = 0; m<M; m++) {
        double U = r + m / (double) M;
        
        while(U>c) {
            if(i == N-1) {
                goto escape; // can't remember when I've last used goto :)
            }
            i++;
            c += samples[i]->weight();
        }
        
        newsamples.push_back(std::make_shared<Sample>(*samples[i]));
        normal ( newsamples.back(), *newsamples.back(), config_.sigma_static_position*dt, config_.sigma_static_orientation*dt );
    }
    
    samples = newsamples;
    
    escape: { };
    */
}

/// update number of samples
while ( allocated_samples > used_samples ) {
    printf("TODO fill new samples\n");
 /*   fixed p = uniform.generate() >> (32-FIXED_FRACPART);
    size_t j = 0;
    j = rand() % samples.size();
    samples.push_back ( std::make_shared<Sample> ( *samples[j] ) );
    SamplePtr &s  = samples.back();
    normal ( s, *s, config_.sigma_static_position*dt, config_.sigma_static_orientation*dt );
    */
    } 
}


void main(void)
{
    
    sio_setbaud(3000000);
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
                samples_new_mem = (msgtype_sample*) realloc(samples_new_mem, sizeof(msgtype_sample) * msg_params.nr_of_samples);
                if(samples_mem == NULL || samples_new_mem == NULL)  {
                    printf("Could not allocate memory for %d samples!\n", msg_params.nr_of_samples);
                    return;
                }
                else printf("Allocated %d samples.\n", msg_params.nr_of_samples);
                
                if(allocated_samples < used_samples) used_samples = allocated_samples;
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
        
        else if(strcmp(ctmp, "usam") == 0) {
            
            printf("Receiving initial samples on FPGA...\n");
            
                    
            for(unsigned int i = 0; i < msg_params.nr_of_samples; i++) {
                readstruct(samples_mem[i]);
            }
            used_samples = msg_params.nr_of_samples;
          
            printf("Received data of the first sample: \n");
            
            #define member(t,x,y) printf(#t " " #x " = "); print(samples_mem[0].x); printf("\n")
            com_sample;
            #undef member
            
            printf("!end!\n");
            
        }  
        
        else printf("\nSomething wrong, expected a message id, got %s!\n",ctmp);
         
        
    }
        
        
}





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

#include "mr2015ws/src/tuw_mr2015/tuw_self_localization/include/tuw_self_localization/fixedpoint.h"

gaussian_random gauss;


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


#include "mr2015ws/src/tuw_mr2015/tuw_self_localization/include/tuw_self_localization/com_structs.h"


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
void print(fixed33mat &a) {
  printf("[");
  for(int i = 0; i<3; i++){
    for(int j = 0; i<5; i++)
      printf("%.5f ", float(a.mat[i][j]));
    if(i < 2) printf("\n");
  }
  printf("]");
}


// generic function for reading a struct via serial link from master
template<class msg> void readstruct(msg& m) {
  unsigned char *p = (unsigned char *) &m;
  for(unsigned int i = 0; i<sizeof(m); i++) {
    *p = getchar(); p++;
  }
  
}

// generic function for reading a struct via serial link from master
template<class msg> void sendstruct(msg& m) {
  unsigned char *p = (unsigned char *) &m;
  for(unsigned int i = 0; i<sizeof(m); i++) {
    putchar(*p); p++;
  }
 
}


// distance-transformed map is stored in heap 
unsigned char *map_mem = nullptr;
unsigned int map_size;
// TODO: making map size dynamic.
unsigned int mapx = 558; unsigned int mapy = 558;


// 2D array-like function for accessing map value at pixel coordinates c,r
unsigned char& map(int c, int r) {
  return *(map_mem + r + c * mapx);    
}


msgtype_sample *samples_mem = nullptr, *samples_new_mem = nullptr;
unsigned int allocated_samples = 0;
unsigned int used_samples = 0;
lfsr uniform(864386345);

msgtype_measurement *measurement_mem = nullptr;
unsigned int allocated_measurements = 0;


// make a new sample by adding noise to an existing one
inline void newNormalSample(msgtype_sample &target, const msgtype_sample &src, const fixed &sigma_position, const fixed &sigma_orientation) {
  target.x = gauss.generate(src.x, sigma_position);
  target.y = gauss.generate(src.y, sigma_position);
  target.theta = gauss.generate(src.theta, sigma_orientation);
}


struct { inline bool operator()(const msgtype_sample &a, const msgtype_sample &b) { return a.weight < b.weight; } } cmp_sample;

void resample() {
  // implement a resample weel
  
  // M is the number the samples to destroy
  unsigned int M = int(msg_params.resample_rate * fixed(used_samples)), N = used_samples;
  #define MWEAKEST 0
  #define LOWVARIANCE 1
  
  fixed sigma_position = msg_params.sigma_static_position*msg_frame_data.duration_last_update;
  fixed sigma_orientation = msg_params.sigma_static_orientation*msg_frame_data.duration_last_update;
  
  if(msg_params.resample_strategy == MWEAKEST) {
    
    sort( samples_mem, samples_mem + used_samples, cmp_sample);
    
    
    for(unsigned int i = 0, k = N - 1; i < MIN(M, N/2); i++, k--) {       
      newNormalSample(samples_mem[i], samples_mem[k], sigma_position, sigma_orientation);
    }
  } 
  
  
  else if(msg_params.resample_strategy == LOWVARIANCE) {
    
    
    // destroying M samples like in the first algorithm == sampling N-M samples
    // M is now the number of samples to draw!
    M = N-M;
    
    fixed r = uniform.generate();
    fixed c = samples_mem[0].weight;
    unsigned int i = 0;
    
    for(unsigned int m = 0; m<M; m++) {
      fixed U = r + fixed(m) * fixed(M).inv();
      
      while(U>c) {
        if(i == N-1) {
          goto escape;
        }
        i++;
        c += samples_mem[i].weight;
      }
      
      newNormalSample(samples_new_mem[m], samples_mem[i], sigma_position, sigma_orientation);
      
    }
    
    escape: { };
    
    // swap two sample buffers
    msgtype_sample *t;
    t = samples_mem;
    samples_mem = samples_new_mem;
    samples_new_mem = t;
    
    
  }
  
  /// update number of samples by adding new ones from randomly chosen
  while ( allocated_samples > used_samples ) {
    
    int p = uniform.generate32();
    size_t j = 0;
    j = p % used_samples;
    newNormalSample(samples_mem[used_samples], samples_mem[j], sigma_position, sigma_orientation);
    used_samples++;
  }   
}


void weighting () {
  
  
  unsigned int used_beams[msg_params.nr_of_beams];
  
  if ( msg_params.random_beams )  {
    // select random beams indexes with Fisher-Yates shuffle
    
    unsigned int all_beams_permutation[allocated_measurements];
    
    for(unsigned int i=0; i < allocated_measurements; ++i)
      all_beams_permutation[i] = i;
    
    for (unsigned int i = allocated_measurements-1; i >= 0; --i) {
      //generate a random number [0, n-1]
      unsigned int j = uniform.generate32() % (i+1);
      
      //swap the last element with element at random index
      int temp = all_beams_permutation[i];
      all_beams_permutation[i] = all_beams_permutation[j];
      all_beams_permutation[j] = temp;
    }
    
    // read first nr_of_beams of the full permutation as the beams to use
    for(unsigned int i = 0; i < msg_params.nr_of_beams; i++)
      used_beams[i] = all_beams_permutation[i];
    
  } else {
    // select equally distributed beams indexes
    
    unsigned int spacing = allocated_samples / msg_params.nr_of_beams;
    
    unsigned int i = spacing / 2; unsigned int c = 0;
    while(c < msg_params.nr_of_beams) {
      i += spacing; c++;
    }
  }
  
  for ( unsigned int idx = 0; idx < used_samples; idx++ ) {
    msgtype_sample &s = samples_mem[idx];
    // compute the weight for each particle
    
    s.weight = fixed(1);
    
    for(unsigned int k : used_beams) {
      msgtype_measurement &beam = measurement_mem[k];
      
      if(beam.length < msg_params.z_max) {
        
        // beam endpoint
        fixed &x = beam.endpoint_x; fixed &y = beam.endpoint_y;
        
        fixed stf[3][3];
        fixed c_ = fcos( s.theta),  s_ = fsin( s.theta );
        
        
        // matrica transformacije za sample
        stf[0][0] = c_; stf[0][1] = -s_; stf[0][2] = s.x; stf[1][0] = s_; stf[1][1] = c_; stf[1][2] = s.y;
        stf[2][0] = fixed(0.); stf[2][1] = fixed(0.); stf[2][2] = fixed(1.);
        
        auto &ztf = msg_frame_data.ztf.mat;
        auto &ftf = msg_params.map_tf.mat;
        
        int rx = int(ftf[0][2] + ftf[0][0]*stf[0][2] + ftf[0][1]*stf[1][2] + x*(ztf[0][0]*(ftf[0][0]*stf[0][0]
        + ftf[0][1]*stf[1][0]) + ztf[1][0]*(ftf[0][0]*stf[0][1] + ftf[0][1]*stf[1][1])) + y*(ztf[0][1]*(ftf[0][0]*stf[0][0] 
        + ftf[0][1]*stf[1][0]) + ztf[1][1]*(ftf[0][0]*stf[0][1] + ftf[0][1]*stf[1][1])) + ztf[0][2]*(ftf[0][0]*stf[0][0] 
        + ftf[0][1]*stf[1][0]) + ztf[1][2]*(ftf[0][0]*stf[0][1] + ftf[0][1]*stf[1][1]));
        
        int ry = int(ftf[1][2] + ftf[1][0]*stf[0][2] + ftf[1][1]*stf[1][2] + x*(ztf[0][0]*(ftf[1][0]*stf[0][0] 
        + ftf[1][1]*stf[1][0]) + ztf[1][0]*(ftf[1][0]*stf[0][1] + ftf[1][1]*stf[1][1])) + y*(ztf[0][1]*(ftf[1][0]*stf[0][0] 
        + ftf[1][1]*stf[1][0]) + ztf[1][1]*(ftf[1][0]*stf[0][1] + ftf[1][1]*stf[1][1])) + ztf[0][2]*(ftf[1][0]*stf[0][0] 
        + ftf[1][1]*stf[1][0]) + ztf[1][2]*(ftf[1][0]*stf[0][1] + ftf[1][1]*stf[1][1]));
        
        if (rx>=0 && rx < (int) mapx && ry >= 0  && ry < (int) mapy)
          
          s.weight = s.weight * fixed( msg_params.likelihoodLookup.gausspdf[map(ry, rx)] );
        
        else { //std::cout << "izletio ";
          s.weight = fixed(0); }
      }
      
      
    }
    
    
    //samples_weight_sum += s->weight();
    printf("Weight: %f\n", float(s.weight));
  }
  
  
  
  /// sort and normalize particles weights
  //     sort ( samples_mem,  samples_mem + used_samples, cmp_sample );
  //     fixed samples_weight_max = fixed(0);
  //     for ( size_t i = 0; i < used_samples; i++ ) {
  //         msgtype_sample &s = samples_mem[i];
  //         //s.weight /= samples_weight_sum;
  //         if ( samples_weight_max < s.weight ) samples_weight_max = s.weight;
  //     }
  
}

void update() {
  for (unsigned int i = 0; i < used_samples; i++ ) {
    // MotionModel
    // implement the forward sample_motion_velocity alogrithm and be aware that w can be zero!!
    // use the config_.alpha1 - config_.alpha6 as noise parameters
    
    fixed &dt = msg_frame_data.duration_last_update;
    msgtype_sample &s = samples_mem[i];
    
    fixed v = msg_frame_data.v, w = msg_frame_data.w;
    fixed fv_w = v * w.inv();
    
    fixed fv = gauss.generate(fixed(v), fixed(msg_params.alpha1*v*v + msg_params.alpha2*w*w));
    fixed fw = gauss.generate(fixed(w), fixed(msg_params.alpha3*v*v + msg_params.alpha4*w*w));
    fixed fgamma = gauss.generate(fixed(0), fixed(msg_params.alpha5*v*v + msg_params.alpha6*w*w));
    
    
    if(w > fixed(0.05) || w < fixed(-0.05)) {
      s.x = s.x + fv_w * (  - (fsin(s.theta)) + (fsin(s.theta + fw * dt))  );
      s.y = s.y + fv_w * (    (fcos(s.theta)) - (fcos(s.theta + fw * dt)) );
    } else {
      s.x = s.x  + fv * dt * (fcos(s.theta));
      s.y = s.y  + fv * dt * (fsin(s.theta));
    }
    s.theta = s.theta + fw * dt + fgamma;
    
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
      
      else if(strcmp(ctmp, "data") == 0) {
        
        printf("Receiving frame data on FPGA...\n");
        
        readstruct(msg_frame_data);
        
        printf("Received data:\n");
        
        #define member(t,x,y) printf(#t " " #x " = "); print(msg_frame_data.x); printf("\n")
        com_frame_data;
        #undef member
        
        if(msg_frame_data.total_beam_count != allocated_measurements) {
          measurement_mem = (msgtype_measurement*) realloc(measurement_mem, sizeof(msgtype_measurement) * msg_frame_data.total_beam_count);
          if( measurement_mem == NULL ) {
            printf("Could not allocate memory for %d measurements!", msg_frame_data.total_beam_count );
            return;
          } else printf( "Allocated memory for %d measurements", msg_frame_data.total_beam_count );
          
          allocated_measurements = msg_frame_data.total_beam_count; 
        }
        
        printf("!end!\n");
        
        for(unsigned int i = 0; i < allocated_measurements; i++) {
          readstruct(measurement_mem[i]);             
        }
        
        
        printf("Received %d measurements. First one: \n", allocated_measurements);
        #define member(t,x,y) printf(#t " " #x " = "); print(measurement_mem[0].x); printf("\n")
        com_measurement;
        #undef member  
        
        if ( msg_params.enable_resample ) resample();
        if ( msg_params.enable_update ) update();
        if ( msg_params.enable_weighting ) weighting();
        
        printf("!end!\n");

      
      }
       
      else if(strcmp(ctmp, "dsam") == 0) {
              
        
        for(unsigned int i = 0; i < msg_params.nr_of_samples; i++) {
          sendstruct(samples_mem[i]);
        }

       if(msg_params.nr_of_samples != used_samples)
          printf("Sanity check failed: used samples != nr_of_samples");
        
        printf("Uploaded %d samples \n", used_samples);
        
        printf("!end!\n");
        
      }
           
      else printf("\nSomething wrong, expected a message id, got %s!\n", ctmp);
      
      
    }
    
    
  }
  
  
  
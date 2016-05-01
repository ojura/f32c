

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


#include "../pf_serial/include_tuw_slmaster/fixedpoint.h"

lfsr uniform(864386345);

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

void main(void)
{
  
  sio_setbaud(3000000);
  
while(1) {
  eventPress();
  
  for(int i=0; i<10000; i++) {
    fixed v = uniform.generate();
    printf("%d %lf\n", i, double(v));
    DELAY(1000);
  }
  printf("end\n");
  
  }

}



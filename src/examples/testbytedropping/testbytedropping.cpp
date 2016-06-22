

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
#include "../pf_serial/include_tuw_slmaster/config.h"

//lfsr uniform(864386345);

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

// char bigbuf[1024];
// int bigp = 0;
// int caller[1024]; int curcaller;

//
// generic function for reading a struct via serial link from master
template<class msg> void readstruct(msg& m) {
  unsigned char *p = (unsigned char *) &m;
  for(unsigned int i = 0; i<sizeof(m); i++) {
    *p = getchar();
//     bigbuf[bigp] = *p;
//     caller[bigp] = curcaller;  bigp++;
    p++;
  }
  
}

// generic function for sending a struct via serial link from master
template<class msg> void sendstruct(msg& m) {
  unsigned char *p = (unsigned char *) &m;
  for(unsigned int i = 0; i<sizeof(m); i++) {
    putchar(*p); p++;
  }
  
}

bool sync_serial() {
  // sync
  int sync = 0;
  const char synctoken[] = "sync";
  
  while(sync < 4) {
    //curcaller = sync;
    char p; readstruct(p);
    
    if(p==synctoken[sync])
      sync++;
    else 
      return false;
  }
  
  if (!sio_rxempty()) 
    return false;
  
  printf("sack");
  return true;
}

struct { int index; int podaci[3]; } msg[10000];

void main(void)
{
  
  sio_setbaud(COM_BAUDRATE);

  while(1) {
  int i = 0;
  while(!sync_serial()) { i++; };

  
  int count = 1500;
  
  for(int i=0; i<count; i++) {
    readstruct(msg[i]);
  }
 
  printf("Hello! %d\n",i);
  i=0;
  
   for(int i=0; i<count; i++) {
     if( i != msg[i].index || msg[i].podaci[0] != 1 || msg[i].podaci[1] != 2 || msg[i].podaci[2] != 3)
      printf("%d %d\n", i, msg[i].index);
   }
  
  printf("Received all %d\n", count);
  
  
  /*for(int i = 0; i<bigp; i++) {
    printf("%d Got %d (%c), caller: %d\n", i, bigbuf[i], bigbuf[i], caller[i]);
  }
  */  
    
  printf("!end!\n");
  

  }

}



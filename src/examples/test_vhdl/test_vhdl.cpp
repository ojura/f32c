/*
 * Print a message on serial console and blink LEDs until a button is pressed.
 *
 * $Id$
 */

#include <stdio.h>
#include <string.h>
#include <dev/io.h>
#include <mips/io.h>
#include "../examples/pf_serial/include_tuw_slmaster/fixedpoint.h"
#include <stdio.h>
#include <stdlib.h>
#include <dev/io.h>
#include <mips/io.h>
#include <dev/sio.h>
#include <sys/fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/param.h>

#ifdef __mips__
static const char *arch = "mips";
#elif defined(__riscv__)
static const char *arch = "riscv";
#else
static const char *arch = "unknown";
#endif

#define	BTN_ANY	(BTN_CENTER | BTN_UP | BTN_DOWN | BTN_LEFT | BTN_RIGHT)

#define IO_LFSR IO_ADDR(0x580)
#define IO_GAUSS IO_ADDR(0x590)

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

inline void eventClick() {
  eventPress();
  eventdePress();
}

lfsr lfsrs[4];
gaussian_random gauss;

void
main(void)
{
  eventClick();
  
  sio_setbaud(COM_BAUDRATE);
  
  printf("!Hello, f32c/%s world!\n", arch);
  
        lfsrs[0] = 0x2996EF15;
        lfsrs[1] = 0x70630F8F;
        lfsrs[2] = 0x59C2ED18;
        lfsrs[3] = 0x291945;
  
        
        OUTW(IO_GAUSS, 0x0);
        OUTW(IO_GAUSS+(1<<2), 0x2996EF15);
        OUTW(IO_GAUSS+(1<<2), 0x70630F8F);
        OUTW(IO_GAUSS+(1<<2), 0x59C2ED18);
        OUTW(IO_GAUSS+(1<<2), 0x291945);

        /*
  int polje[10], counts[10];   
           for(int i = 0; i < 10; i++) {
             INB(counts[i], IO_LFSR+4);
             INW(polje[i], IO_LFSR);          
}

for(int i = 0; i < 10; i++)
  printf("%d: %d %x\n", i, counts[i], polje[i]);

while(true) { };*/
  
  
  for(int i = 0; i < 30000; i++) {
    
//     for(unsigned int j=0; j < 4; j++) {
//     printf("%d SW%d: %x\n",i,j, lfsrs[j].generate31());
//     
//     if(j == 0) INW(in, IO_GAUSS);
//     if(j == 1) INW(in, IO_GAUSS+(1<<2));
//     if(j == 2) INW(in, IO_GAUSS+(2<<2));
//     if(j == 3) INW(in, IO_GAUSS+(3<<2));
//         printf("%d HW%d: %x\n",i, j, in);   
//     
//      }
    
    int in;
    OUTW(IO_GAUSS+(2<<2), fixed(0).val);
    OUTW(IO_GAUSS+(3<<2), fixed(1).val);    
    INW(in, IO_GAUSS);
    printf("%d SW: %lf \n", i, double(gauss.generate(fixed(0), fixed(1))));
    printf("%d HW: %lf \n", i, double(fixed(in, true)));
    
  }
  
  while(true) { };
}


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

#define IO_FSIN IO_ADDR(0x5A0)


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

void dbg(int i) {
    fixed a = fixed(i*0.1*M_PI);
    fixed r;
    printf("SW sin %lf %lf\n", double(i * 0.1), double(cos(double(a))));
    //printf("SW fsin %lf %lf\n", double(i*0.1), double(fsin(a)));
    OUTW(IO_FSIN+4, a.val);
    INW(r.val, IO_FSIN+4);
    printf("HW %lf %lf\n",  double(i*0.1), r.val / double( pow(2.0, 31) ) );
}


void
main(void)
{
  eventClick();
  
  sio_setbaud(COM_BAUDRATE);
  printf("!Hello, f32c/%s world!\n", arch);

  for(int i = -21; i < 21; i++) { 
    dbg(i);
    printf("-----------------\n");
  }
  for (int j = 0; j < 4; j++)
    dbg(10);
    
  while(true) { };
}


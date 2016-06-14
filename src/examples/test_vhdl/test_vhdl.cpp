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
     
  
  for(int i = 0; i < 10; i++) {    
    printf("%d HW: %x \n", i, gauss.generate(fixed(0), fixed(1)).val );
  }
  
  while(true) { };
}


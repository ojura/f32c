#ifndef CONFIG_H
#define CONFIG_H

#ifndef PF_SLAVE
  //#define USEFIXED
  #define USEFPGA

  #define READTIMEOUT (0)
  #define WRITETIMEOUT (0)
  #define SYNCTIMEOUT (1)
#else
  #include <dev/io.h>
  #define IO_GAUSS IO_ADDR(0x590)
  #define IO_FSIN IO_ADDR(0x5A0)
#endif

// valid baudrates: 3000000, 2500000, 2000000, 1500000, 1152000, 1000000,
//                  921600, 576000, 500000, 460800, 230400, 115200
#define COM_BAUDRATE 1500000

#endif

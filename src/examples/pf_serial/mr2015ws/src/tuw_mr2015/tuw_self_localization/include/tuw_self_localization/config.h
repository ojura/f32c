#ifndef CONFIG_H
#define CONFIG_H

#ifndef PF_SLAVE
  //#define USEFIXED
  #define USEFPGA

  #define READTIMEOUT (100*10)
  #define WRITETIMEOUT (100*10)
  #define SYNCTIMEOUT 100*100
#endif

// valid baudrates: 3000000, 2500000, 2000000, 1500000, 1152000, 1000000,
//                  921600, 576000, 500000, 460800, 230400, 115200
#define COM_BAUDRATE 1500000

#endif

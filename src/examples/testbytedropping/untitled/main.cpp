#include <iostream>
#include <tuw_self_localization/rs232.h>
#include <tuw_self_localization/config.h>
#include <tuw_self_localization/serial_support.h>

using namespace std;

struct { int index; int podaci[3]; } msg;

int main() {
      
    initComport();

    int r = syncFPGA();
  
    cout << "synced " << r << endl;

    msg.podaci[0] = 1; msg.podaci[1] = 2; msg.podaci[2] = 3;
    
    for(int i = 0; i < 1500; i++) {
      msg.index = i;
      sendFPGAstruct(msg);      
      //getFPGAmsg();
    }
    
    getFPGAmsg();
    
    return 0;
}
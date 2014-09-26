#include <magnetometer_lsm303/lsm303.hpp>
#include <iostream>
#include <stdio.h>

using namespace std;

int main(int argc, char** argv){
  if (argc < 2) {
    printf("Please specify device, e.g. %s serial:///dev/ttyUSB0:57600\n",argv[0]);
    return -1;
  }

  magnetometer_lsm303::Driver* d = new magnetometer_lsm303::Driver();
  d->setReadTimeout(base::Time::fromSeconds(20));
  cout << "Opening serial device" << endl;
  d->open(argv[1]);
  while(1){
    try{
      d->read();
    }
    catch(iodrivers_base::TimeoutError){
      cout << "Timeout error" << endl;
    }
    printf("Dev %1i: ACC(% 6.2f|% 6.2f|% 6.2f) MAG(% 6.2f|% 6.2f|% 6.2f)",d->getDevNo(),
        d->getAccX(),
        d->getAccY(),
        d->getAccZ(),
        d->getMagX() * 1e6,
        d->getMagY() * 1e6,
        d->getMagZ() * 1e6);
    d->getDevNo() ? printf("\n") : printf("\t");
  }
}

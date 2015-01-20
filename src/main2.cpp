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
  d->setReadTimeout(base::Time::fromSeconds(2));
  cout << "Opening serial device" << endl;
  d->open(argv[1]);
  while(1){
    try{
      d->read();
    }
    catch(iodrivers_base::TimeoutError){
      cout << "Timeout error" << endl;
    }

    d->getDevNo() == 0 ? printf("\n") : printf("\t");
    printf("%i %06i %06i %06i %06i %06i %06i",d->getDevNo(),
            d->getRawAccX(),
            d->getRawAccY(),
            d->getRawAccZ(),
            d->getRawMagX(),
            d->getRawMagY(),
            d->getRawMagZ());
  }
}

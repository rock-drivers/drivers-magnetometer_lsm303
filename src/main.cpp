#include <magnetometer_lsm303/lsm303.hpp>
#include <iostream>
#include <stdio.h>

using namespace std;

int main(int argc, char** argv){
  magnetometer_lsm303::Driver* d = new magnetometer_lsm303::Driver();
  d->setReadTimeout(base::Time::fromSeconds(20));
  cout << "Opening serial device" << endl;
  d->open("serial:///dev/ttyUSB0:57600");
  while(1){
    try{
      d->read();
    }
    catch(iodrivers_base::TimeoutError){
      cout << "Timeout error" << endl;
    }
    printf("Dev No %i: ACC(%f|%f|%f) MAG(%f|%f|%f)\n",d->getDevNo(),
        d->getAccX(),
        d->getAccY(),
        d->getAccZ(),
        d->getMagX(),
        d->getMagY(),
        d->getMagZ());
  }
}

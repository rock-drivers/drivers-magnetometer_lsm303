#include <magnetometer_lsm303/lsm303.hpp>
#include <iostream>

using namespace std;

int main(int argc, char** argv){
  magnetometer_lsm303::Driver* d = new magnetometer_lsm303::Driver();
  d->setReadTimeout(base::Time::fromSeconds(2));
  cout << "Opening serial device" << endl;
  d->open("serial:///dev/ttyUSB0:19200");
  while(1){
    try{
      d->read();
    }
    catch(iodrivers_base::TimeoutError){
      cout << "Timeout error" << endl;
    }
  }
}

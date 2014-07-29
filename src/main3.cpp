#include <magnetometer_lsm303/lsm303.hpp>
#include <iostream>
#include <stdio.h>
#include <Eigen/Dense>

using namespace std;

int main(int argc, char** argv){
  magnetometer_lsm303::Driver* d = new magnetometer_lsm303::Driver();
  d->readAccCalibrationParameters(argv[1]);
  Eigen::Vector3d acc, res;
  acc << 0, 0, -16000;
  res = d->correctAccReading(acc);
  std::cout << res << std::endl;

}

#include <magnetometer_lsm303/lsm303.hpp>
#include <iostream>
#include <stdio.h>
#include <Eigen/Dense>

using namespace std;

int main(int argc, char** argv){

    magnetometer_lsm303::Driver* d = new magnetometer_lsm303::Driver();

    Eigen::Matrix<double,10,3>Y, w;

    Y << 0, 0, -1, 
         0, 0, -1,
         0, 0, -1,
         0, 0, -1,
         0, 0, -1,
         0, 0, -1,
         0, 0, -1,
         0, 0, -1,
         0, 0, -1,
         0, 0, -1;

    w << 3.55000000e+02,   1.13000000e+02,  -1.47510000e+04,
         3.54000000e+02,  -5.40000000e+01,  -1.47340000e+04,
         4.42000000e+02,  -5.10000000e+01,  -1.46300000e+04,
         4.17000000e+02,  -5.00000000e+00,  -1.47410000e+04,
         4.10000000e+02,   3.10000000e+01,  -1.48550000e+04,
         3.88000000e+02,   6.00000000e+01,  -1.45750000e+04,
         3.26000000e+02,   0.00000000e+00,  -1.47820000e+04,
         3.83000000e+02,   4.50000000e+01,  -1.46930000e+04,
         3.43000000e+02,   2.10000000e+01,  -1.48460000e+04,
         3.09000000e+02,   1.90000000e+01,  -1.46370000e+04; 

    Eigen::Matrix<double,4,3>X;

    X = d->computeAccCalibrationMatrix(w,Y);

    return 0;
}


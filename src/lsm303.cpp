#include "lsm303.hpp"
#include <stdio.h>
#include <string.h>
#include <boost/crc.hpp>
#include <string>
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#define MAG_RESOLUTION 0.08   /// mgauss/LSB according to LSM303D data sheet 
#define ACC_RESOLUTION 0.061  /// mg/LSB according to LSM303D data sheet
#define STANDARD_GRAVITY 9.80665 /// standard gravity according to CGPM CR70
#define NDEV 5 ///number of magnetometers attached

using namespace magnetometer_lsm303;
using namespace Eigen;

Driver::Driver() : iodrivers_base::Driver(10000),
                   ax(0),
                   ay(0),
                   az(0),
                   mx(0),
                   my(0),
                   mz(0),
                   dev_no(255)
{
  //Set scale factor in accelerometer and magnetometer correction matrices to default values
    CalibrationMatrix m;
    m.setIdentity();

    double acc_scale = ACC_RESOLUTION * STANDARD_GRAVITY * 1.0e-3;
    m(0,0) = acc_scale;
    m(1,1) = acc_scale;
    m(2,2) = acc_scale;
    AccCalibrationMatrix.assign(NDEV,m);
    
    double mag_scale = MAG_RESOLUTION * 1.0e-7;
    m(0,0) = mag_scale;
    m(1,1) = mag_scale;
    m(2,2) = mag_scale;
    MagCalibrationMatrix.assign(NDEV,m);
}

/// Open the device using openURI
void Driver::open(std::string const& uri){
  openURI(uri);
}

/// Read Packet from device
void Driver::read(){
  uint8_t buffer[1000];
  int packet_size = readPacket(buffer,10000);

  parsePacket(buffer, packet_size);
}

/** Function to parse a complete LSM303D packet with valid CRC (checked before in extractPacket()).
  * The function reassembles the int16_t values for accelerometer and
  * magnetometer values */
void Driver::parsePacket(uint8_t const* buffer, size_t size){
  dev_no = buffer[1];
  ax = (int16_t) buffer[3] << 8 | buffer [2];
  ay = (int16_t) buffer[5] << 8 | buffer [4];
  az = (int16_t) buffer[7] << 8 | buffer [6];
  mx = (int16_t) buffer[9] << 8 | buffer [8];
  my = (int16_t) buffer[11] << 8 | buffer [10];
  mz = (int16_t) buffer[13] << 8 | buffer [12];
}

/** Trying to extract a packet with valid startbyte, correct length and CRC checksum.
  * CRC is computed using 16bit CCITT. */
int Driver::extractPacket(uint8_t const* buffer, size_t size) const {
  //printf("extract Packet called with %i bytes\n",(int)size);
  uint8_t *start;

  //search for start char '*'
  start = (uint8_t*) memchr(buffer,'*',size);

  if(start == NULL){
    //printf("Start char not found\n");
    return -size; // not start char
  }
  else if(buffer - start != 0){
    //printf("Start char found at pos %i, but not at first byte\n",(int) (buffer-start));
    return (int) (buffer-start); //start char, but not at first byte   
  }
  else if(buffer - start == 0 && size < 16){
    //printf("Start char at 0, but not complete yet\n");
    return 0; //start char at [0] but not complete
  }
  else { // we have a full packet
      //printf("Full packet, size: %i\n", (int)size);
    //check crc
    boost::crc_ccitt_type ccitt;
    ccitt.process_bytes(buffer,14);
    //printf("boost CCITT computed checksum: %i\n",ccitt.checksum()); 
    //printf("transmitted checksum: %i", *(uint16_t*)(&buffer[14]));
    if(ccitt.checksum() == *(uint16_t*)(&buffer[14])){ //crc ok
      //printf("crc ok\n");
      return 16;
    }
    else {
        //printf("wrong crc, returning %i\n",(int)-size);
        return -size;
    }
  }
  return -size;
}

/// returns raw x component of magnetic field 
int16_t Driver::getRawMagX(void){
  return mx;
}

/// returns raw y component of magnetic field 
int16_t Driver::getRawMagY(void){
  return my;
}

/// returns raw z component of magnetic field 
int16_t Driver::getRawMagZ(void){
  return mz;
}

/// returns raw x component of acceleration
int16_t Driver::getRawAccX(void){
  return ax;
}

/// returns raw y component of acceleration
int16_t Driver::getRawAccY(void){
  return ay;
}

/// returns raw z component of acceleration
int16_t Driver::getRawAccZ(void){
  return az;
}

/** Function returns corrected accelerometer readings using a 4x3 correction matrix.
  * The matrix incorporates misalignment- , scale and offset-errors as outlined
  * in ST application note AN3192 */
Vector3d Driver::getAcc(){
  Matrix<double,1,4> x;
  x << ax, ay, az, 1.0;
  return  (x * AccCalibrationMatrix.at(dev_no)).transpose();
}

/** Function returns corrected magnetometer readings using a 4x3 correction matrix.
  * The matrix incorporates hardiron and softiron errors as outlined
  * in ST application note AN3192 */
Vector3d Driver::getMag(){
  Matrix<double,1,4> x;
  x << mx, my, mz, 1.0;
  return  (x * MagCalibrationMatrix.at(dev_no)).transpose();
}

/// Returns the number of the device, which sent the data (useful in case of multiple / chained LSM303 on one microcontroller)
uint8_t Driver::getDevNo(void){
  return dev_no;
}

/** Compute an accelerometer correction matrix X from raw accelerometer samples w 
 * (nx4 matrix, homogeneous form required). Matrix Y is the matrix of expected calibrated values in the form nx3
 */
Matrix<double,4,3,DontAlign> Driver::computeAccCalibrationMatrix(const MatrixX3d &w, const MatrixX3d &Y){
    // preparing Y = w * X, with Y being the expected acc values for known positions (nx3),
    // w the raw value readings (nx4, homogeneous) and X the 4x3 calibration matrix to be computed

    //TODO check for right shape of w and Y

    std::cout << "w:\n" << w << std::endl;
    std::cout << "Y:\n" << Y << std::endl;

    std::cout << "homogeneous w:\n" << w.rowwise().homogeneous() << std::endl;

    // least square solving w*X = Y (often Ax=b)
    Matrix<double,4,3,DontAlign>X = w.rowwise().homogeneous().colPivHouseholderQr().solve(Y);
    std::cout << "Solved wX = Y with X:\n" << X << std::endl;
    return X;
}

/// Set the correction matrix for accelerometer n
void Driver::setAccCalibrationMatrix(int n,Matrix<double,4,3,DontAlign> m){
  AccCalibrationMatrix.at(n) = m;
}

/// Set the correction matrix for magnetometer n
void Driver::setMagCalibrationMatrix(int n, Matrix<double,4,3,DontAlign> m){
  MagCalibrationMatrix.at(n) = m;
}

/** Convenience function to change scale per axis of accelerometer 
 * correction matrix for device number n
 */
void Driver::setAccScale(int n, double x, double y, double z){
  AccCalibrationMatrix.at(n)(0,0) = x;
  AccCalibrationMatrix.at(n)(1,1) = y;
  AccCalibrationMatrix.at(n)(2,2) = z;
}

/** Convenience function to change scale per axis of magnetometer 
 * correction matrix for device number n
 */
void Driver::setMagScale(int n, double x, double y, double z){
  MagCalibrationMatrix.at(n)(0,0) = x;
  MagCalibrationMatrix.at(n)(1,1) = y;
  MagCalibrationMatrix.at(n)(2,2) = z;
} 

/** Convenience function to change scale of accelerometer 
 * correction matrix for device number n
 */
void Driver::setAccScale(int n, double s){
  setAccScale(n,s,s,s);
}
  
/** Convenience function to change scale of magnetometer 
 * correction matrix for device number n
 */
void Driver::setMagScale(int n, double s){
  setMagScale(n,s,s,s);
}

/** Set accelerometer correction matrix offset per axis
 * in m/s^2 for device  number n
 */
void Driver::setAccOffset(int n, double x, double y, double z){
  AccCalibrationMatrix.at(n)(3,0) = x;
  AccCalibrationMatrix.at(n)(3,1) = y;
  AccCalibrationMatrix.at(n)(3,2) = z;
}

/** Compute direction mean of all attached magnetometer devices */    
Vector3d magnetometer_lsm303::computeDirectionMean(const std::vector<Vector3d> &directionSamples){
    Vector3d meanDir;
    meanDir.setZero();

    for(auto const& v: directionSamples){
        if(!v.isZero()) meanDir += v.normalized(); // only add if not zero-vector -> no direction
    }
    return meanDir.isZero() ? Vector3d::Zero() : meanDir.normalized();
}

/** Compute mean vector */
Vector3d magnetometer_lsm303::computeVectorMean(const std::vector<Vector3d> &samples){
    Vector3d meanVec;
    meanVec.setZero();

    for(auto const& v: samples){
        meanVec += v;
    }
    return meanVec / samples.size();
}

/** Compute direction dispersion of a multi device magnetometer 
 * sample using a DispersionMetric m */
double magnetometer_lsm303::computeDirectionDispersion(const std::vector<Vector3d> &directionSamples, DispersionMetric m){
    //TODO throw error if zero or only one sample given -> no dispersion (directionSamples.size() < 2)
    Vector3d resultingVector;
    resultingVector.setZero();

    int N = 0;
    for(auto const& v: directionSamples){
        if(!v.isZero()) {
            resultingVector += v.normalized(); // only add if not zero-vector -> no direction
            N++;
        }
    }

    switch (m){
        // calculate precision parameter k of Fisher-Distribution
        case MISES_FISHER_K:    {
                                    return (N - 1) / (N - resultingVector.norm()); // TODO Attention to division by zero if all samples have the exact same direction
                                    break;         
                                }
        case MISES_FISHER_S2:   { //aka variance of delta angles
                                    double s2 = 0.0;
                                    double delta = 0.0;
                                    N = 0;
                                    //TODO use running sample variance http://www.johndcook.com/blog/standard_deviation/
                                    for(auto const& v: directionSamples){
                                        delta = acos((v.dot(resultingVector)) / (v.norm() * resultingVector.norm())); // compute delta angle axb/|a|*|b| between mean and sample direction
                                        s2 += delta*delta; // accumulate squared delta angles 
                                        N++;
                                    }
                                    return s2 / (N-1); // unbiased sample variance (i.e. with Bessel's correction)
                                    break;
                                }
        case KENT: return 0; break;
        case BINGHAM: return 0; break;
        default: return 0; break;
    }
}

double Driver::computeHeading(){
    Eigen::Vector3d acc = Driver::getAcc();
    Eigen::Vector3d mag = Driver::getMag();

    // Get quaternion, that removes pitch and roll from body
    Eigen::Quaternion<double> q = Eigen::Quaternion<double>::FromTwoVectors(acc,Eigen::Vector3d::UnitZ());

    //rotate mag into horizontal plane
    Eigen::Vector3d mag_h = q * mag;
    return atan2(mag_h.y(),mag_h.x());
}

    



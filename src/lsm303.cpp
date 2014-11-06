/** @file */

#include "lsm303.hpp"
#include <stdio.h>
#include <string.h>
#include <boost/crc.hpp>
//#include <boost/lexical_cast.hpp>
#include <string>
#include <iostream>
//#include <fstream>
#include <Eigen/Geometry>
//#include <base/Pose.hpp>

#define MAG_RESOLUTION 0.08   /// mgauss/LSB according to LSM303D data sheet 
#define ACC_RESOLUTION 0.061  /// mg/LSB according to LSM303D data sheet
#define STANDARD_GRAVITY 9.80665 /// standard gravity according to CGPM CR70
#define NDEV 5 ///number of magnetometers attached

using namespace magnetometer_lsm303;

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

void Driver::open(std::string const& uri){
  openURI(uri);
}

void Driver::read(){
  uint8_t buffer[1000];
  int packet_size = readPacket(buffer,10000);
  parsePacket(buffer, packet_size);
}

/** Function to parse a complete packet with valid CRC (checked before).
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
  //printf("extract Packet called\n");
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
  else if(start == 0 && size < 16){
    //printf("Start char at 0, but not complete yet\n");
    return 0; //start char at [0] but not complete
  }
  else { // we have a full packet
    //check crc
    boost::crc_ccitt_type ccitt;
    ccitt.process_bytes(buffer,14);
    if(ccitt.checksum() == *(uint16_t*)(&buffer[14])){ //crc ok
      return 16;
    }
    else return -size;
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

/** Function returns correct accelerometer readings using a 4x3 correction matrix.
  * The matrix incorporates misalignment- , scale and offset-errors as outlined
  * in ST application note AN3192 */
Eigen::Vector3d Driver::getAcc(){
  Eigen::Matrix<double,1,4> x;
  x << ax, ay, az, 1.0;
  return  (x * AccCalibrationMatrix.at(dev_no)).transpose();
}

/** Function returns correct magnetometer readings using a 4x3 correction matrix.
  * The matrix incorporates hardiron and softiron errors as outlined
  * in ST application note AN3192 */
Eigen::Vector3d Driver::getMag(){
  Eigen::Matrix<double,1,4> x;
  x << mx, my, mz, 1.0;
  return  (x * MagCalibrationMatrix.at(dev_no)).transpose();
}

/// Returns the number of the device, which sent the data (useful in case of multiple / chained LSM303 on one microcontroller)
uint8_t Driver::getDevNo(void){
  return dev_no;
}

void Driver::setAccCalibrationMatrix(int n,Eigen::Matrix<double,4,3,Eigen::DontAlign> m){
  AccCalibrationMatrix.at(n) = m;
}

void Driver::setMagCalibrationMatrix(int n, Eigen::Matrix<double,4,3,Eigen::DontAlign> m){
  MagCalibrationMatrix.at(n) = m;
}

void Driver::setAccScale(int n, double x, double y, double z){
  AccCalibrationMatrix.at(n)(0,0) = x;
  AccCalibrationMatrix.at(n)(1,1) = y;
  AccCalibrationMatrix.at(n)(2,2) = z;
}

void Driver::setMagScale(int n, double x, double y, double z){
  MagCalibrationMatrix.at(n)(0,0) = x;
  MagCalibrationMatrix.at(n)(1,1) = y;
  MagCalibrationMatrix.at(n)(2,2) = z;
}

void Driver::setAccScale(int n, double s){
  setAccScale(n,s,s,s);
}

void Driver::setMagScale(int n, double s){
  setMagScale(n,s,s,s);
}

/// set accelerometer offset in m/s^2
void Driver::setAccOffset(int n, double x, double y, double z){
  AccCalibrationMatrix.at(n)(3,0) = x;
  AccCalibrationMatrix.at(n)(3,1) = y;
  AccCalibrationMatrix.at(n)(3,2) = z;
}
     

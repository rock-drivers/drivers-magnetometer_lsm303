/** @file */

#include "lsm303.hpp"
#include <stdio.h>
#include <string.h>
#include <boost/crc.hpp>
#include <boost/lexical_cast.hpp>
#include <string>
#include <iostream>
#include <fstream>

#define MAG_RESOLUTION 0.08   /// mgauss/LSB according to LSM303D data sheet 
#define ACC_RESOLUTION 0.061  /// mg/LSB according to LSM303D data sheet
#define STANDARD_GRAVITY 9.80665 /// standard gravity according to CGPM CR70

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
  //Set scale factor in accelerometer correction matrix to put out m/s^2
  AccCalibrationMatrix.setIdentity();
  setAccScale(ACC_RESOLUTION * STANDARD_GRAVITY * 1.0e-3);
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

/// returns x component of magnetic field in Tesla
double Driver::getMagX(void){
  return Driver::adc2tesla(mx);
}

/// returns y component of magnetic field in Tesla
double Driver::getMagY(void){
  return Driver::adc2tesla(my);
}

/// returns z component of magnetic field in Tesla
double Driver::getMagZ(void){
  return Driver::adc2tesla(mz);
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
  return  (x * AccCalibrationMatrix).transpose();
}

/// Returns the number of the device, which sent the data (useful in case of multiple / chained LSM303 on one microcontroller)
uint8_t Driver::getDevNo(void){
  return dev_no;
}
/// Helper function to get tesla from adc raw reading depending on MAG_RESOLUTION
inline double Driver::adc2tesla(int16_t  val){
  return val * MAG_RESOLUTION * 1.0e-7;
} 
/// Helper function to get m/s^2 units from raw adc accelerometer readings, depending on ACC_RESOLUTION
//inline double Driver::adc2meter_per_second_squared(int16_t val){
//  return val * ACC_RESOLUTION * STANDARD_GRAVITY * 1.0e-3;
//}

void Driver::setAccCalibrationParameters(double acc11, double acc12, double acc13,
                                         double acc21, double acc22, double acc23,
                                         double acc31, double acc32, double acc33,
                                         double acc10, double acc20, double acc30){
  
  AccCalibrationMatrix << acc11, acc21, acc31, 
                          acc12, acc22, acc32,
                          acc13, acc23, acc33,
                          acc10, acc20, acc30;
} 
void Driver::setAccScale(double x, double y, double z){
  AccCalibrationMatrix(0,0) = x;
  AccCalibrationMatrix(1,1) = y;
  AccCalibrationMatrix(2,2) = z;
}

void Driver::setAccScale(double s){
  setAccScale(s,s,s);
}

/// set accelerometer offset in m/s^2
void Driver::setAccOffset(double x, double y, double z){
  AccCalibrationMatrix(3,0) = x;
  AccCalibrationMatrix(3,1) = y;
  AccCalibrationMatrix(3,2) = z;
}

/** Function to read a correction parameter matrix for the accelerometers of the ST LSM303
  * from a file. The file must contain 12 values in 4x3 shape as outlined in ST application
  * note AN3192. */
void Driver::readAccCalibrationParameters(char* filename){
  std::vector<std::string> words;
  std::string str;
  std::ifstream fin(filename);
  while (fin >> str){
    words.push_back(str);
  }
  fin.close();

  if(words.size() != 12){
    //error
  }
  else {
    for (int i = 0; i < 12; ++i){
      AccCalibrationMatrix(i / 3, i % 3) =  boost::lexical_cast<double>(words[i]);
    }
  }
}


    
    
     

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
                   dev_no(255),
                   AccCalibrationMatrix(Eigen::Matrix<double,4,3>::Identity())
{
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

/// Returns x component of acceleration in m/s^2
double Driver::getAccX(void){
  return Driver::adc2meter_per_second_squared(ax);
}

/// Returns y component of acceleration in m/s^2
double Driver::getAccY(void){
  return Driver::adc2meter_per_second_squared(ay);
}

/// Returns z component of acceleration in m/s^2
double Driver::getAccZ(void){
  return Driver::adc2meter_per_second_squared(az);
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

/// Returns the number of the device, which sent the data (useful in case of multiple / chained LSM303 on one microcontroller)
uint8_t Driver::getDevNo(void){
  return dev_no;
}
/// Helper function to get tesla from adc raw reading depending on MAG_RESOLUTION
inline double Driver::adc2tesla(int16_t  val){
  return val * MAG_RESOLUTION * 1.0e-7;
} 
/// Helper function to get m/s^2 units from raw adc accelerometer readings, depending on ACC_RESOLUTION
inline double Driver::adc2meter_per_second_squared(int16_t val){
  return val * ACC_RESOLUTION * STANDARD_GRAVITY * 1.0e-3;
}

/** Function to correct accelerometer readings using a 4x3 correction matrix.
  * The matrix incorporates misalignment- , scale and offset-errors as outlined
  * in ST application note AN3192 */
Eigen::Vector3d Driver::correctAccReading(Eigen::Vector3d acc){
  Eigen::Matrix<double,1,4> x;
  x << acc(0), acc(1), acc(2), 1.0;
  return  (x * AccCalibrationMatrix).transpose();
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


    
    
     

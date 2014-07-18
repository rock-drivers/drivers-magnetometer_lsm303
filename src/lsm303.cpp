#include "lsm303.hpp"
#include <stdio.h>
#include <string.h>
#include <boost/crc.hpp>

#define MAG_RESOLUTION 0.08   // mgauss/LSB according to LSM303D data sheet 
#define ACC_RESOLUTION 0.061  // mg/LSB according to LSM303D data sheet
#define STANDARD_GRAVITY 9.80665 // standard gravity according to CGPM CR70

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
}

void Driver::open(std::string const& uri){
  openURI(uri);
}

void Driver::read(){
  uint8_t buffer[1000];
  int packet_size = readPacket(buffer,10000);
  parsePacket(buffer, packet_size);
}

void Driver::parsePacket(uint8_t const* buffer, size_t size){
//  uint8_t start = buffer[0];
  dev_no = buffer[1];
  ax = (int16_t) buffer[3] << 8 | buffer [2];
  ay = (int16_t) buffer[5] << 8 | buffer [4];
  az = (int16_t) buffer[7] << 8 | buffer [6];
  mx = (int16_t) buffer[9] << 8 | buffer [8];
  my = (int16_t) buffer[11] << 8 | buffer [10];
  mz = (int16_t) buffer[13] << 8 | buffer [12];
 // printf("Device No. %i: ACC(%+05i|%+05i|%+05i) MAG(%+05i|%+05i|%+05i)\n",dev_no,ax, ay, az, mx, my, mz);
}

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

// returns x component of magnetic field in Tesla
double Driver::getMagX(void){
  return Driver::adc2tesla(mx);
}

// returns y component of magnetic field in Tesla
double Driver::getMagY(void){
  return Driver::adc2tesla(my);
}

// returns z component of magnetic field in Tesla
double Driver::getMagZ(void){
  return Driver::adc2tesla(mz);
}

double Driver::getAccX(void){
  return Driver::adc2meter_per_second_squared(ax);
}

double Driver::getAccY(void){
  return Driver::adc2meter_per_second_squared(ay);
}

double Driver::getAccZ(void){
  return Driver::adc2meter_per_second_squared(az);
}

uint8_t Driver::getDevNo(void){
  return dev_no;
}

inline double Driver::adc2tesla(int16_t  val){
  return val * MAG_RESOLUTION * 1.0e-7;
} 

inline double Driver::adc2meter_per_second_squared(int16_t val){
  return val * ACC_RESOLUTION * STANDARD_GRAVITY * 1.0e-3;
}

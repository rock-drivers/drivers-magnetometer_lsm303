#include "lsm303.hpp"
#include <stdio.h>
#include <string.h>
#include <boost/crc.hpp>

using namespace magnetometer_lsm303;

Driver::Driver() : iodrivers_base::Driver(10000),
                   ax(0),
                   ay(0),
                   az(0),
                   mx(0),
                   my(0),
                   mz(0)
{
  foo = 3;
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
  foo = 7;
  ax = (int16_t) buffer[2] << 8 | buffer [1];
  ay = (int16_t) buffer[4] << 8 | buffer [3];
  az = (int16_t) buffer[6] << 8 | buffer [5];
  mx = (int16_t) buffer[8] << 8 | buffer [7];
  my = (int16_t) buffer[10] << 8 | buffer [9];
  mz = (int16_t) buffer[12] << 8 | buffer [11];
  printf("parsed ACC(%+05i|%+05i|%+05i) MAG(%+05i|%+05i|%+05i)\n",ax, ay, az, mx, my, mz);
}

int Driver::extractPacket(uint8_t const* buffer, size_t size) const {
  //printf("extract Packet called\n");
  uint8_t *start;

  //search for start char '*'
  start = (uint8_t*) memchr(buffer,'*',size);

  if(start == NULL){
    printf("Start char not found\n");
    return -size; // not start char
  }
  else if(buffer - start != 0){
    printf("Start char found at pos %i, but not at first byte\n",(int) (buffer-start));
    return (int) (buffer-start); //start char, but not at first byte   
  }
  else if(start == 0 && size < 15){
    printf("Start char at 0, but not complete yet\n");
    return 0; //start char at [0] but not complete
  }
  else { // we have a full packet
    //check crc
    boost::crc_ccitt_type result;
    result.process_bytes(buffer,size);
    printf("WE have a full package, crc: %X\n",(uint16_t) result.checksum());
    if(result.checksum() == 0){ //crc ok
      return 15;
    }
    else return -size;
  }
  return 0;
}


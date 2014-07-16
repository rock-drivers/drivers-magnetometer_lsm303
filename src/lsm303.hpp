#ifndef _LSM303_HPP_
#define _LSM303_HPP_

#include <iodrivers_base/Driver.hpp>
#include <base/samples/IMUSensors.hpp>
#include <inttypes.h>

namespace magnetometer_lsm303 {

  class Driver : public iodrivers_base::Driver{
    public:
      Driver();
      void open(std::string const& uri);
      void read();
      int extractPacket(uint8_t const* buffer, size_t size) const;
      float getMagX(void);
      float getMagY(void);
      float getMagZ(void);
      float getAccX(void);
      float getAccY(void);
      float getAccZ(void);
      uint8_t getDevNo(void);
    private:
      void parsePacket(uint8_t const *buffer, size_t size);
      int16_t ax, ay, az, mx, my, mz;
      uint8_t dev_no;
  };
}
#endif

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
      double foo;
    private:
      void parsePacket(uint8_t const *buffer, size_t size);
      int16_t ax, ay, az, mx, my, mz;
  };
}
#endif

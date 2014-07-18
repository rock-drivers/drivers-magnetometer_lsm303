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
      double getMagX(void);
      double getMagY(void);
      double getMagZ(void);
      double getAccX(void);
      double getAccY(void);
      double getAccZ(void);
      uint8_t getDevNo(void);
    private:
      void parsePacket(uint8_t const *buffer, size_t size);
      inline static double adc2tesla(int16_t);
      inline static double adc2meter_per_second_squared(int16_t);
      int16_t ax, ay, az, mx, my, mz;
      uint8_t dev_no;
  };
}
#endif

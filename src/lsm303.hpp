#ifndef _LSM303_HPP_
#define _LSM303_HPP_

#include <iodrivers_base/Driver.hpp>
//#include <base/samples/IMUSensors.hpp>
#include <inttypes.h>
#include <Eigen/Dense>

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
      int16_t getRawMagX(void);
      int16_t getRawMagY(void);
      int16_t getRawMagZ(void);
      int16_t getRawAccX(void);
      int16_t getRawAccY(void);
      int16_t getRawAccZ(void);
      uint8_t getDevNo(void);
      Eigen::Vector3d correctAccReading(Eigen::Vector3d acc);
      void readAccCalibrationParameters(char* filename);
    private:
      void parsePacket(uint8_t const *buffer, size_t size);
      inline static double adc2tesla(int16_t);
      inline static double adc2meter_per_second_squared(int16_t);
      int16_t ax, ay, az, mx, my, mz;
      uint8_t dev_no;
      Eigen::Matrix<double,4,3> AccCalibrationMatrix;
  };
}
#endif

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
      int16_t getRawMagX(void);
      int16_t getRawMagY(void);
      int16_t getRawMagZ(void);
      int16_t getRawAccX(void);
      int16_t getRawAccY(void);
      int16_t getRawAccZ(void);
      uint8_t getDevNo(void);
      Eigen::Vector3d getAcc(void);
      void readAccCalibrationParameters(char* filename);
      void setAccCalibrationParameters(double acc11, double acc12, double acc13,
                                       double acc21, double acc22, double acc23,
                                       double acc31, double acc32, double acc33,
                                       double acc10, double acc20, double acc30);
      void setAccScale(double);
      void setAccScale(double,double,double);
      void setAccOffset(double,double,double);

    private:
      void parsePacket(uint8_t const *buffer, size_t size);
      inline static double adc2tesla(int16_t);
      //inline static double adc2meter_per_second_squared(int16_t);
      int16_t ax, ay, az, mx, my, mz;
      uint8_t dev_no;
      Eigen::Matrix<double,4,3,Eigen::DontAlign> AccCalibrationMatrix;
  };
}
#endif

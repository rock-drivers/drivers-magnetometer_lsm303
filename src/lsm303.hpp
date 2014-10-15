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
      int16_t getRawMagX(void);
      int16_t getRawMagY(void);
      int16_t getRawMagZ(void);
      int16_t getRawAccX(void);
      int16_t getRawAccY(void);
      int16_t getRawAccZ(void);
      uint8_t getDevNo(void);
      Eigen::Vector3d getAcc(void);
      Eigen::Vector3d getMag(void);
      void readAccCalibrationParameters(char* filename);
      void setAccCalibrationMatrix(Eigen::Matrix<double,4,3,Eigen::DontAlign>); 
      void setMagCalibrationMatrix(Eigen::Matrix<double,4,3,Eigen::DontAlign>); 
      void setAccScale(double);
      void setAccScale(double,double,double);
      void setAccOffset(double,double,double);
      void setMagScale(double);
      void setMagScale(double,double,double);

    private:
      void parsePacket(uint8_t const *buffer, size_t size);
      int16_t ax, ay, az, mx, my, mz;
      uint8_t dev_no;
      Eigen::Matrix<double,4,3,Eigen::DontAlign> AccCalibrationMatrix;
      Eigen::Matrix<double,4,3,Eigen::DontAlign> MagCalibrationMatrix;
  };
}
#endif

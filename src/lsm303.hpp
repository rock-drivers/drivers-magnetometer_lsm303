#ifndef _LSM303_HPP_
#define _LSM303_HPP_

#include <inttypes.h>
#include <iodrivers_base/Driver.hpp>
#include <Eigen/Dense>

namespace magnetometer_lsm303 {

    typedef Eigen::Matrix<double,4,3,Eigen::DontAlign> CalibrationMatrix;
    enum DispersionMetric {MISES_FISHER_K, MISES_FISHER_S2, KENT, BINGHAM};

    Eigen::Vector3d computeDirectionMean(const std::vector<Eigen::Vector3d> &);
    //TODO computeDirectionDispersion for different statistics
    double computeDirectionDispersion(const std::vector<Eigen::Vector3d> &, DispersionMetric = MISES_FISHER_K);

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

            Eigen::Matrix<double,4,3,Eigen::DontAlign> computeAccCalibrationMatrix(const Eigen::MatrixX3d &, const Eigen::MatrixX3d&);

            void setAccCalibrationMatrix(int, Eigen::Matrix<double,4,3,Eigen::DontAlign>); 
            void setMagCalibrationMatrix(int,Eigen::Matrix<double,4,3,Eigen::DontAlign>); 
            void setAccScale(int,double);
            void setAccScale(int,double,double,double);
            void setAccOffset(int,double,double,double);
            void setMagScale(int,double);
            void setMagScale(int,double,double,double);



        private:
            void parsePacket(uint8_t const *buffer, size_t size);
            int16_t ax, ay, az, mx, my, mz;
            uint8_t dev_no;
            std::vector<CalibrationMatrix> AccCalibrationMatrix;
            std::vector<CalibrationMatrix> MagCalibrationMatrix;
    };
}
#endif

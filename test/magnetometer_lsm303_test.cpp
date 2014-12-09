#define BOOST_TEST_MODULE MagnetometerLSM303Test
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <magnetometer_lsm303/lsm303.hpp>

using namespace magnetometer_lsm303;

BOOST_AUTO_TEST_CASE(directionMeanSame){
    Eigen::Vector3d a, b, c, res;

    a << 0 , 0, 2;
    b << 0 , 0, 2;
    res << 0 , 0, 1;

    std::vector<Eigen::Vector3d> v; 
    v.push_back(a);
    v.push_back(b);
    c = computeDirectionMean(v);
    BOOST_CHECK(c.isApprox(res, 0.00001));
}

BOOST_AUTO_TEST_CASE(directionMeanPerpendicular){
    Eigen::Vector3d a, b, c, res;

    a << 0 , 0, 2;
    b << 0 , 2, 0;
    res << 0, 0.7071, 0.7071;

    std::vector<Eigen::Vector3d> v; 
    v.push_back(a);
    v.push_back(b);
    c = computeDirectionMean(v);
    BOOST_CHECK(c.isApprox(res, 0.00001));
}

BOOST_AUTO_TEST_CASE(directionMeanOpposite){
    Eigen::Vector3d a, b, c, res;

    a << 0 , 0, 2;
    b << 0 , 0, -2;
    res << 0, 0, 0;

    std::vector<Eigen::Vector3d> v; 
    v.push_back(a);
    v.push_back(b);
    c = computeDirectionMean(v);
    BOOST_TEST_MESSAGE(c);
    BOOST_CHECK(c.isApprox(res, 0.00001));
}

BOOST_AUTO_TEST_CASE(directionMeanIncludingZeroVector){
    Eigen::Vector3d a, b, c, d, res;

    a << 0 , 0, 2;
    b << 0 , 2, 0;
    c << 0 , 0, 0;
    res << 0, 0.7071, 0.7071;

    std::vector<Eigen::Vector3d> v; 
    v.push_back(a);
    v.push_back(b);
    v.push_back(c);
    c = computeDirectionMean(v);
    BOOST_TEST_MESSAGE(c);
    BOOST_CHECK(c.isApprox(res, 0.00001));
}

BOOST_AUTO_TEST_CASE(directionDispersionMISES_FISHER_K){
    Eigen::Vector3d a, b; 

    a << 0 , 0, 2;
    b << 0 , 0.1, 2.0;
    double res = 1602.7;

    std::vector<Eigen::Vector3d> v; 
    v.push_back(a);
    v.push_back(b);
    double k = computeDirectionDispersion(v);
    BOOST_TEST_MESSAGE(k);
    BOOST_CHECK_CLOSE(k,res, 0.1);
}

BOOST_AUTO_TEST_CASE(directionDispersionSameDirection_MISES_FISHER_K){
    Eigen::Vector3d a, b; 

    a << 0.0 , 0.0, 2.0;
    b << 0.0 , 0.0, 2.0;
    double res = std::numeric_limits<double>::infinity();

    std::vector<Eigen::Vector3d> v; 
    v.push_back(a);
    v.push_back(b);
    double k = computeDirectionDispersion(v);
    BOOST_TEST_MESSAGE(k);
    BOOST_CHECK_EQUAL(k,res);
}

BOOST_AUTO_TEST_CASE(directionDispersion_MISES_FISHER_S2){
    Eigen::Vector3d a, b; 

    a << 0.0 , 0.0, 1.0;
    b << 0.0 , 1.0, 0.0;
    double res = 1.2337;  

    std::vector<Eigen::Vector3d> v; 
    v.push_back(a);
    v.push_back(b);
    double s2 = computeDirectionDispersion(v,MISES_FISHER_S2);
    BOOST_TEST_MESSAGE(s2);
    BOOST_CHECK_CLOSE(s2,res,1e-4);
}

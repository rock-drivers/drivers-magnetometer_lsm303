#define BOOST_TEST_MODULE MagnetometerLSM303Test
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <magnetometer_lsm303/lsm303.hpp>
#include <boost/math/constants/constants.hpp>

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


BOOST_AUTO_TEST_CASE(simpleVectorMean_sameDir){
    Eigen::Vector3d a, b, c, nom, act; 

    a << 1.1 , 0.0, 0.0;
    b << 1.0 , 0.0, 0.0;
    c << 0.9 , 0.0, 0.0;
    nom << 1.0 , 0.0, 0.0;

    std::vector<Eigen::Vector3d> v; 
    v.push_back(a);
    v.push_back(b);
    v.push_back(c);

    act = computeVectorMean(v);
    BOOST_TEST_MESSAGE(act);
    BOOST_CHECK(act.isApprox(nom,1e-8));
}

BOOST_AUTO_TEST_CASE(simpleVectorMean_oppositeDir){
    Eigen::Vector3d a, b, act; 

    a << 2.0 , 2.0, 2.0;
    b << -2.0 , -2.0, -2.0;

    std::vector<Eigen::Vector3d> v; 
    v.push_back(a);
    v.push_back(b);

    act = computeVectorMean(v);
    BOOST_TEST_MESSAGE(act);
    BOOST_CHECK(act.isZero(1e-8));
}

BOOST_AUTO_TEST_CASE(heading_level_allx){
    Eigen::Vector3d acc, mag; 
    double act, nom;
    acc << 0.0,0.0,9.81; // body level with world xy plane 
    mag << 1.0,0.0,0.67; // mag x pointing to magnetic north 
    nom = 0.0;
    act = computeHeading(acc,mag);
    BOOST_CHECK_CLOSE(nom,act,1e-2);
}

BOOST_AUTO_TEST_CASE(heading_level_ally){
    Eigen::Vector3d acc, mag; 
    double act, nom;
    acc << 0.0,0.0,9.81; // body level with world xy plane 
    mag << 0.0,1.0,0.67; // mag x pointing to magnetic north 
    nom = - boost::math::constants::pi<double>() / 2;
    act = computeHeading(acc,mag);
    BOOST_TEST_MESSAGE(act);
    BOOST_CHECK_CLOSE(nom,act,1e-2);
}

BOOST_AUTO_TEST_CASE(heading_level_xy){
    Eigen::Vector3d acc, mag; 
    double act, nom;
    acc << 0.0,0.0,9.81; // body level with world xy plane 
    mag << 1.0,1.0,0.67; // mag x pointing to magnetic north 
    nom = - boost::math::constants::pi<double>() / 4;
    act = computeHeading(acc,mag);
    BOOST_TEST_MESSAGE(act);
    BOOST_CHECK_CLOSE(nom,act,1e-2);
}

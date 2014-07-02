#include <boost/test/unit_test.hpp>
#include <magnetometer_lsm303/Dummy.hpp>

using namespace magnetometer_lsm303;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    magnetometer_lsm303::DummyClass dummy;
    dummy.welcome();
}

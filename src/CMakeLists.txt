rock_library(magnetometer_lsm303
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(magnetometer_lsm303_bin Main.cpp
    DEPS magnetometer_lsm303)


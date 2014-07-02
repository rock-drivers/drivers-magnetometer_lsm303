rock_library(magnetometer_lsm303
    SOURCES lsm303.cpp
    HEADERS lsm303.hpp 
    DEPS_PKGCONFIG iodrivers_base)

rock_executable(lsm303_main main.cpp
    DEPS magnetometer_lsm303)


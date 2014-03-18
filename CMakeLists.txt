cmake_minimum_required(VERSION 2.6)

project(cvra-simu)

# Check for CppUTest and bail out if they don't have it
if(DEFINED ENV{CPPUTEST_HOME})
    message("Using CppUTest found in $ENV{CPPUTEST_HOME}")
else()
    message("CPPUTEST_HOME is not set; You must tell CMake where to find CppUTest")

endif()

set(CMAKE_BUILD_TYPE Debug)


file(GLOB_RECURSE
    modules_source
    modules/*
)

file(GLOB_RECURSE
    tests_source
    tests/*
)

include_directories(include/)
include_directories(module/scheduler)
include_directories(modules/blocking_detection_manager)
include_directories(modules/control_system_manager)
include_directories(modules/cvra_servo)
include_directories(modules/cvra_adc)
include_directories(modules/cvra_bldc)
include_directories(modules/cvra_dc)
include_directories(modules/cvra_logger)
include_directories(modules/dual_quadramp)
include_directories(modules/error)
include_directories(modules/obstacle_avoidance)
include_directories(modules/pid)
include_directories(modules/position_manager)
include_directories(modules/quadramp)
include_directories(modules/robot_system)
include_directories(modules/scheduler)
include_directories(modules/trajectory_manager)
include_directories(modules/math/fast_math)
include_directories(modules/math/fixed_point)
include_directories(modules/math/geometry)
include_directories(modules/math/vect2)
include_directories(modules/uptime)
include_directories(modules/vt100)
include_directories(modules/cirbuf)
include_directories(modules/rdline)
include_directories(modules/parse)
include_directories(modules/ramp)
include_directories(modules/cvra_pio)
include_directories(modules/cvra_servo)
include_directories(modules/commandline)

include_directories($ENV{CPPUTEST_HOME}/include)
link_directories($ENV{CPPUTEST_HOME}/lib)


add_executable(
	debra
    #${debra_source}
    #${modules_source}
    ${tests_source}
)

target_link_libraries(
	debra
	m
    CppUTest
    CppUTestExt
)

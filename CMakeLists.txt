cmake_minimum_required(VERSION 2.6)

project(cvra-simu)



set(CMAKE_BUILD_TYPE Debug)
file(GLOB_RECURSE
    nastya_source
    nastya/arm.c
    nastya/armFunc.c
    nastya/com_balises.c
    nastya/comm_pc.c
    nastya/cvra_cs.c
    nastya/fifo.c
    nastya/hardware.c
    nastya/main.c
    nastya/posFunction.c
    nastya/strat.c
    nastya/move_queue.c
    nastya/commands.c
)

file(GLOB_RECURSE
    debra_source
    debra/arm.c
    debra/armFunc.c
    debra/com_balises.c
    debra/comm_pc.c
    debra/cvra_cs.c
    debra/fifo.c
    debra/hardware.c
    debra/main.c
    debra/posFunction.c
    debra/strat.c
    debra/move_queue.c
    debra/commands.c
)

file(GLOB_RECURSE
    modules_source
    modules/*
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

add_executable(
    debra
    ${debra_source}
    ${modules_source}
)

target_link_libraries(
    debra
    m
)

add_executable(
    nastya
    ${nastya_source}
    ${modules_source}
)

target_link_libraries(
    nastya
    m
)

if(MSVC)
    if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
        string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    endif()
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-long-long -Wno-unused-value")
endif()

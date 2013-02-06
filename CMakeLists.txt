cmake_minimum_required(VERSION 2.6)

project(cvra-simu)


file(GLOB_RECURSE
	source_files
	arm.c
    arm_interpolators.c
	armFunc.c
	avoiding.c
	com_balises.c
	comm_pc.c
	cvra_cs.c
	fifo.c
	hardware.c
	main.c
	posFunction.c
	strat.c
    strat_utils.c
    move_queue.c
    commands.c
    commandline.c
    commands_misc.c
    commands_arm.c
	modules/*
)

include_directories(include/)
include_directories(module/scheduler)
include_directories(modules/blocking_detection_manager)
include_directories(modules/control_system_manager)
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

add_executable(
	simu
	${source_files}
)

target_link_libraries(
	simu
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

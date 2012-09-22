cmake_minimum_required(VERSION 2.6)

project(cvra-simu)


file(GLOB_RECURSE
	source_files
	arm.c
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
	modules/*
)

include_directories(include/)
include_directories(module/scheduler)
include_directories(modules/blocking_detection_manager)
include_directories(modules/control_system_manager)
include_directories(modules/couple_limiter)
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

add_executable(
	simu
	${source_files}
)

target_link_libraries(
	simu
	m
)

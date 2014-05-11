add_executable(
    obstacle_avoidance_comm_test
    obstacle_avoidance_comm_test.c
    ${debra_source}
    ${modules_source}
    ${lwip_source}
)

target_link_libraries(obstacle_avoidance_comm_test m)
target_link_libraries (obstacle_avoidance_comm_test ${CMAKE_THREAD_LIBS_INIT})

add_executable(
    cinematics_test
    ${debra_source}
    ${modules_source}
    main.c
)

target_link_libraries(
	cinematics_test
	m
)

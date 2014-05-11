add_executable(
    cinematics_table
    ${debra_source}
    ${modules_source}
    main.c
)

target_link_libraries(
	cinematics_table
	m
)

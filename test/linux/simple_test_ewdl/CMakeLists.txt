
set(SOURCES ewdl_idle0.c)
add_executable(ewdl_idle0 ${SOURCES})
target_link_libraries(ewdl_idle0 soem)
install(TARGETS ewdl_idle0 DESTINATION bin)

set(SOURCES ewdl_csp1.c)
add_executable(ewdl_csp1 ${SOURCES})
target_link_libraries(ewdl_csp1 soem)
install(TARGETS ewdl_csp1 DESTINATION bin)

set(SOURCES ewdl_csv2.c)
add_executable(ewdl_csv2 ${SOURCES})
target_link_libraries(ewdl_csv2 soem)
install(TARGETS ewdl_csv2 DESTINATION bin)

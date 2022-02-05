
set(SOURCES ewdl_idle.c)
add_executable(ewdl_idle ${SOURCES})
target_link_libraries(ewdl_idle soem)
install(TARGETS ewdl_idle DESTINATION bin)

set(SOURCES ewdl_run.c)
add_executable(ewdl_run ${SOURCES})
target_link_libraries(ewdl_run soem)
install(TARGETS ewdl_run DESTINATION bin)

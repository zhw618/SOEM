
set(SOURCES simple_test_zhw.c)
add_executable(simple_test_zhw ${SOURCES})
target_link_libraries(simple_test_zhw soem)
install(TARGETS simple_test_zhw DESTINATION bin)

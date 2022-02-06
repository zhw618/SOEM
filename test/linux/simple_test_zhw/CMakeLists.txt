
# set(SOURCES simple_test_zhw.c)
# add_executable(simple_test_zhw ${SOURCES})
# target_link_libraries(simple_test_zhw soem)
# install(TARGETS simple_test_zhw DESTINATION bin)

# set(SOURCES pvmode.c)
# add_executable(pvmode ${SOURCES})
# target_link_libraries(pvmode soem)
# install(TARGETS pvmode DESTINATION bin)

set(SOURCES csdn_pv.c)
add_executable(csdn_pv ${SOURCES})
target_link_libraries(csdn_pv soem)
install(TARGETS csdn_pv DESTINATION bin)

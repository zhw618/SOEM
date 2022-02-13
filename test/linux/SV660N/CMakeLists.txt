
# set(SOURCES simple_test_zhw.c)
# add_executable(simple_test_zhw ${SOURCES})
# target_link_libraries(simple_test_zhw soem)
# install(TARGETS simple_test_zhw DESTINATION bin)

# set(SOURCES pvmode.c)
# add_executable(pvmode ${SOURCES})
# target_link_libraries(pvmode soem)
# install(TARGETS pvmode DESTINATION bin)

# set(SOURCES csdn_pv.c)
# add_executable(csdn_pv ${SOURCES})
# target_link_libraries(csdn_pv soem)
# install(TARGETS csdn_pv DESTINATION bin)

set(SOURCES sv660n_run.c servoPDO.h servoshm.h)
add_executable(sv660n_run ${SOURCES})
target_link_libraries(sv660n_run soem)
install(TARGETS sv660n_run DESTINATION bin)

#共享内存区域查看器,不进行大小端转换!
set(SOURCES shmViewer.c servoPDO.h servoshm.h)
add_executable(shmViewer ${SOURCES})
target_link_libraries(shmViewer soem)
install(TARGETS shmViewer DESTINATION bin)

#共享内存区域查看器,转化大小端后再查看!
set(SOURCES shmViewer2LE.c servoPDO.h servoshm.h)
add_executable(shmViewer2LE ${SOURCES})
target_link_libraries(shmViewer2LE soem)
install(TARGETS shmViewer2LE DESTINATION bin)
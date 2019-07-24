add_executable(test_web_server EXCLUDE_FROM_ALL test_web_server.cpp)
target_link_libraries(test_web_server ${PROJECT_NAME} ${catkin_LIBRARIES})

if(TARGET tests)
  add_dependencies(tests
    test_web_server
    )
endif()

add_rostest(tests.test)

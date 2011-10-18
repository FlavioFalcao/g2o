find_package(GTest)

if(NOT GTEST_FOUND)
  message(STATUS "Disabling testing since gtest was not found.")
  return()
endif()

include_directories(${GTEST_INCLUDE_DIRS})

add_executable(g2o-test
  main.cpp
  sba_test.cpp
  )

target_link_libraries(g2o-test
  ${GTEST_LIBRARIES}
  pthread
  )

add_test(g2o_tests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/g2o-test)

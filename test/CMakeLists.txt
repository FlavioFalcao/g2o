find_package(GTest)

if(NOT GTEST_FOUND)
  message(STATUS "Disabling testing since gtest was not found.")
  return()
endif()

include_directories(${GTEST_INCLUDE_DIRS})

add_executable(g2o_test
  main.cpp
  sba_test.cpp
  )

target_link_libraries(g2o_test
  ${GTEST_LIBRARIES}
  pthread
  core solver_cholmod math_groups types_sba types_slam3d types_icp ${OPENGL_LIBRARIES} ${CHOLMOD_LIBRARIES}
  g2o_ectomodule
  )

# add_test(g2o_tests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/g2o_test)

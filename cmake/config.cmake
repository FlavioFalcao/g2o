set(g2o_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include 
  ${PROJECT_SOURCE_DIR} /usr/include/suitesparse)
# FIXME: suitesparse???

set(g2o_LIBS ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/g2o.so
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2o_core.so
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2o_solver_csparse.so
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2o_stuff.so
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2o_types_sim3.so
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2o_math_groups.so
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2o_solver_dense.so
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2o_types_icp.so
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2o_types_slam2d.so
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2o_solver_cholmod.so
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2o_solver_pcg.so
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2o_types_sba.so
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libg2o_types_slam3d.so)

configure_file(cmake/g2oConfig.cmake.in
  ${CMAKE_BINARY_DIR}/g2oConfig.cmake
  @ONLY
)
configure_file(cmake/g2oConfig-version.cmake.in
  ${CMAKE_BINARY_DIR}/g2oConfig-version.cmake
  @ONLY
)

#this is a bit simple.
set(g2o_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include/${prefix})
set(g2o_LIBRARIES ${CMAKE_INSTALL_PREFIX}/lib/libg2o.so)
configure_file(cmake/g2oConfig.cmake.in
  ${PROJECT_BINARY_DIR}/share/g2oConfig.cmake
  @ONLY
)
configure_file(cmake/g2oConfig-version.cmake.in
  ${CMAKE_BINARY_DIR}/share/g2oConfig-version.cmake
  @ONLY
)

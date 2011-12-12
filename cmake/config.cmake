set(g2o_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include #g2o/sba.h
    ${PROJECT_BINARY_DIR} #g2o/config.h
    /usr/include/suitesparse
    )
# FIXME: suitesparse??? This should be done with a find_package(...)

set(g2o_LIBS  
  core
  solver_csparse
  stuff
  types_sim3
  math_groups
  solver_dense
  types_icp
  types_slam2d
  solver_cholmod
  solver_pcg
  types_sba
  types_slam3d
  )

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

configure_file(cmake/g2oConfig.cmake.in
  ${PROJECT_BINARY_DIR}/share/g2oConfig.cmake
  @ONLY
)
configure_file(cmake/g2oConfig-version.cmake.in
  ${CMAKE_BINARY_DIR}/share/g2oConfig-version.cmake
  @ONLY
)

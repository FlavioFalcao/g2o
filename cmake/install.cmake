set(prefix g2o-${g2o_VERSION})
install(DIRECTORY ${g2o_SOURCE_DIR}/include/
  DESTINATION include/${prefix}
  COMPONENT main
  )

#install the unix_install
install(DIRECTORY ${g2o_BINARY_DIR}/share/
  DESTINATION share/${prefix}
  COMPONENT main
  )


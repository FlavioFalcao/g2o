find_package(ecto)
if(NOT ecto_FOUND)
  return()
endif()

ecto_python_env_gen(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

ectomodule(g2o
   module.cpp
   Sba.cpp
   SbaDisparity.cpp
)

link_ecto(g2o
  core
  solver_cholmod
  types_icp
)

install_ecto_module(g2o)

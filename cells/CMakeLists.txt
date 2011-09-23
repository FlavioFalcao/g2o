find_package(ecto)
if(NOT ecto_FOUND)
  return()
endif()

ecto_python_env_gen(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

ectomodule(g2o
   module.cpp
   SBA.cpp
)

link_ecto(g2o
  core
)


